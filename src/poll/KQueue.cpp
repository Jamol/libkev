/* Copyright (c) 2016-2025, Fengping Bao <jamol@live.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "IOPoll.h"
#include "Notifier.h"
#include "utils/kmtrace.h"

#include <sys/types.h>
#include <sys/event.h>
#include <sys/time.h>

KEV_NS_BEGIN

#define MAX_EVENT_NUM   256

class KQueue : public IOPoll
{
public:
    KQueue();
    ~KQueue();
    
    bool init() override;
    Result registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb) override;
    Result unregisterFd(SOCKET_FD fd) override;
    Result updateFd(SOCKET_FD fd, KMEvent events) override;
    Result wait(uint32_t wait_time_ms) override;
    void notify() override;
    PollType getType() const override { return PollType::KQUEUE; }
    
    // can be false on ET mode, but return true to removing the write event
    // and thus reduce the kqueue eventlist size
    bool isLevelTriggered() const override { return true; }

    Result registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb, IOPollData *&data) override;
    Result updateFd(SOCKET_FD fd, KMEvent events, IOPollData *data) override;
    Result unregisterFd(SOCKET_FD fd, IOPollData *&data) override;

protected:
#ifdef IOPOLL_ITEMS_USE_DATA
    IOPollData* allocPollData();
    void freePollData(IOPollData *data);
    void processPendingPollData();
#endif

private:
    int             kqueue_fd_ { -1 };
    NotifierPtr     notifier_;
    IOPollData*     notifier_data_ { nullptr };
    
    // on ET mode (EV_CLEAR is set), it seems EVFILT_READ won't be triggered
    // if EVFILT_READ is set after data arrived
    bool            work_on_et_mode_ { false };
#ifdef IOPOLL_ITEMS_USE_DATA
    std::unique_ptr<IoPollDataManager<IOPollData>> poll_data_mgr_;
    std::mutex      poll_data_mutex_;
#else
    std::unique_ptr<IOPollItemManager<IOPollItem>> poll_item_mgr_;
#endif
};

KQueue::KQueue()
{
	
}

KQueue::~KQueue()
{
    if(INVALID_FD != kqueue_fd_) {
        ::close(kqueue_fd_);
        kqueue_fd_ = INVALID_FD;
    }
#ifdef IOPOLL_ITEMS_USE_DATA
    if (notifier_data_) {
        freePollData(notifier_data_);
        notifier_data_ = nullptr;
    }
#endif
}

bool KQueue::init()
{
    if (INVALID_FD != kqueue_fd_) {
        return true;
    }
    kqueue_fd_ = ::kqueue();
    if(INVALID_FD == kqueue_fd_) {
        KLOGE("KQueue::init, kqueue() failed, errno=" << errno);
        return false;
    }
#if defined(EVFILT_USER) && defined(NOTE_TRIGGER)
    struct kevent ev;
    EV_SET(&ev, 0, EVFILT_USER, EV_ADD|EV_CLEAR, 0, 0, 0);
    if (::kevent(kqueue_fd_, &ev, 1, 0, 0, 0) != -1) {
        notifier_.reset();
    } else 
#endif
    {
        notifier_ = Notifier::createNotifier();
    }
    if (notifier_ && !notifier_->ready()) {
        if(!notifier_->init()) {
            KLOGE("KQueue::init, notifier init failed");
            ::close(kqueue_fd_);
            kqueue_fd_ = INVALID_FD;
            return false;
        }
        IOCallback cb ([this](SOCKET_FD, KMEvent ev, void*, size_t) { notifier_->onEvent(ev); });
        registerFd(notifier_->getReadFD(), kEventRead|kEventError, std::move(cb), notifier_data_);
    }
    return true;
}

Result KQueue::registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb)
{
#if !defined(IOPOLL_ITEMS_USE_DATA)
    if (fd < 0) {
        return Result::INVALID_PARAM;
    }
    if (!poll_item_mgr_) {
        poll_item_mgr_ = std::make_unique<IOPollItemManager<IOPollItem>>();
    }
    auto *poll_item = poll_item_mgr_->getPollItem(fd, true);
    if (!poll_item) {
        KLOGE("KQueue::registerFd no poll item, fd=" << fd << ", sz=" << poll_item_mgr_->getPollItemSize());
        return Result::BUFFER_TOO_SMALL;
    }
    poll_item->fd = fd;
    poll_item->events = 0;
    poll_item->cb = std::move(cb);
    auto ret = updateFd(fd, events);
    if (ret != Result::OK) {
        poll_item->reset();
    }
    KLOGI("KQueue::registerFd, fd="<<fd<<", ev="<<events<<", ret="<<(int)ret);
    return ret;
#else
    return Result::NOT_IMPLEMENTED;
#endif
}

Result KQueue::unregisterFd(SOCKET_FD fd)
{
#if !defined(IOPOLL_ITEMS_USE_DATA)
    auto sz = poll_item_mgr_->getPollItemSize();
    KLOGI("KQueue::unregisterFd, fd="<<fd<<", sz="<<sz);
    auto *poll_item = poll_item_mgr_->getPollItem(fd);
    if (!poll_item) {
        KLOGE("KQueue::unregisterFd failed, fd=" << fd);
        return Result::INVALID_PARAM;
    }
    struct kevent kevents[2];
    int nchanges = 0;
    if (poll_item->events & kEventRead) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_READ, EV_DELETE, 0, 0, 0);
    }
    if (poll_item->events & kEventWrite) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_WRITE, EV_DELETE, 0, 0, 0);
    }
    if (nchanges) {
        ::kevent(kqueue_fd_, kevents, nchanges, 0, 0, 0);
    }
    poll_item_mgr_->clearPollItem(fd);
    return Result::OK;
#else
    return Result::NOT_IMPLEMENTED;
#endif
}

Result KQueue::updateFd(SOCKET_FD fd, KMEvent events)
{
#if !defined(IOPOLL_ITEMS_USE_DATA)
    auto *poll_item = poll_item_mgr_->getPollItem(fd);
    if (!poll_item || INVALID_FD == poll_item->fd) {
        return Result::INVALID_PARAM;
    }
    
    struct kevent kevents[2];
    int nchanges = 0;
    if (!!(poll_item->events & kEventRead) && !(events & kEventRead)) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_READ, EV_DELETE, 0, 0, 0);
        poll_item->events &= ~kEventRead;
    }
    if (!!(poll_item->events & kEventWrite) && !(events & kEventWrite)) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_WRITE, EV_DELETE, 0, 0, 0);
        poll_item->events &= ~kEventWrite;
    }
    if (nchanges) { // remove events
        ::kevent(kqueue_fd_, kevents, nchanges, 0, 0, 0);
    }
    if (poll_item->events == events) {
        return Result::OK;
    }
    nchanges = 0;
    unsigned short op = EV_ADD;
    if (work_on_et_mode_) {
        op |= EV_CLEAR;
    }
    if (events & kEventRead) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_READ, op, 0, 0, 0);
    }
    if (events & kEventWrite) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_WRITE, op, 0, 0, 0);
    }
    if(nchanges && ::kevent(kqueue_fd_, kevents, nchanges, 0, 0, 0) == -1) {
        KLOGE("KQueue::updateFd error, fd="<<fd<<", errno="<<errno);
        return Result::FAILED;
    }
    poll_item->events = events;
    //KLOGI("KQueue::updateFd, fd="<<fd<<", ev="<<events);
    return Result::OK;
#else
    return Result::NOT_IMPLEMENTED;
#endif
}

#if defined(IOPOLL_ITEMS_USE_DATA)
IOPollData* KQueue::allocPollData()
{
    {
        std::lock_guard<std::mutex> g(poll_data_mutex_);
        if (!poll_data_mgr_) {
            poll_data_mgr_ = std::make_unique<IoPollDataManager<IOPollData>>();
        }
        auto *data = poll_data_mgr_->getFreePollData();
        if (data) {
            return data;
        }
    }
    return poll_data_mgr_->createPollData();
}

void KQueue::freePollData(IOPollData *data)
{
    if (!data) {
        return;
    }
    data->reset();
    {
        std::lock_guard<std::mutex> g(poll_data_mutex_);
        if (poll_data_mgr_) {
            poll_data_mgr_->freePollData(data);
            return;
        }
    }
    delete data;
}

void KQueue::processPendingPollData()
{
    std::lock_guard<std::mutex> g(poll_data_mutex_);
    if (poll_data_mgr_) {
        poll_data_mgr_->processPendingPollData();
    }
}
#endif // defined(IOPOLL_ITEMS_USE_DATA)

Result KQueue::registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb, IOPollData *&data)
{
#if defined(IOPOLL_ITEMS_USE_DATA)
    if (fd < 0) {
        return Result::INVALID_PARAM;
    }
    IOPollData *poll_data = data;
    if (!poll_data) {
        poll_data = allocPollData();
        if (!poll_data) {
            KLOGE("KQueue::registerFd no poll data, fd=" << fd);
            return Result::BUFFER_TOO_SMALL;
        }
    } else if (fd == poll_data->fd) {
        
    } else if (INVALID_FD != poll_data->fd) {
        KLOGW("KQueue::registerFd different fd: " << fd << " " << poll_data->fd);
    }
    {
        std::lock_guard<std::recursive_mutex> g(poll_data->rmtx);
        poll_data->fd = fd;
        poll_data->events = 0;
        poll_data->cb = std::move(cb);
    }
    auto ret = updateFd(fd, events, poll_data);
    if (!data) { // poll_data was allocated here
        if (ret == Result::OK) {
            data = poll_data;
        } else {
            freePollData(poll_data);
        }
    }
    KLOGI("KQueue::registerFd, fd="<<fd<<", ev="<<events<<", ret="<<(int)ret << ", data=" << data);
    return ret;
#else
    data = nullptr;
    return registerFd(fd, events, std::move(cb));
#endif
}

Result KQueue::unregisterFd(SOCKET_FD fd, IOPollData *&data)
{
#if defined(IOPOLL_ITEMS_USE_DATA)
    KLOGI("KQueue::unregisterFd, fd="<<fd<<", data="<<data);
    if (!data) {
        return Result::INVALID_PARAM;
    }
    struct kevent kevents[2];
    int nchanges = 0;
    if (data->events & kEventRead) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_READ, EV_DELETE, 0, 0, 0);
    }
    if (data->events & kEventWrite) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_WRITE, EV_DELETE, 0, 0, 0);
    }
    if (nchanges) {
        ::kevent(kqueue_fd_, kevents, nchanges, 0, 0, 0);
    }
    freePollData(data);
    data = nullptr;
    return Result::OK;
#else
    return unregisterFd(fd);
#endif
}

Result KQueue::updateFd(SOCKET_FD fd, KMEvent events, IOPollData *data)
{
#if defined(IOPOLL_ITEMS_USE_DATA)
    if (!data) {
        return Result::INVALID_PARAM;
    }
    if (fd != data->fd) {
        KLOGW("KQueue::updateFd different fd: " << fd << " " << data->fd);
        return Result::INVALID_PARAM;
    }
    if (data->events == events) {
        return Result::OK;
    }

    auto poll_events = data->events;
    
    struct kevent kevents[2];
    int nchanges = 0;
    if (!!(poll_events & kEventRead) && !(events & kEventRead)) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_READ, EV_DELETE, 0, 0, 0);
        poll_events &= ~kEventRead;
    }
    if (!!(poll_events & kEventWrite) && !(events & kEventWrite)) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_WRITE, EV_DELETE, 0, 0, 0);
        poll_events &= ~kEventWrite;
    }
    if (nchanges) { // remove events
        ::kevent(kqueue_fd_, kevents, nchanges, 0, 0, 0);
    }
    if (poll_events == events) {
        if (poll_events != data->events) {
            std::lock_guard<std::recursive_mutex> g(data->rmtx);
            data->events = poll_events;
        }
        return Result::OK;
    }
    nchanges = 0;
    unsigned short op = EV_ADD;
    if (work_on_et_mode_) {
        op |= EV_CLEAR;
    }
    if (events & kEventRead) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_READ, op, 0, 0, data);
    }
    if (events & kEventWrite) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_WRITE, op, 0, 0, data);
    }
    if(nchanges && ::kevent(kqueue_fd_, kevents, nchanges, 0, 0, 0) == -1) {
        KLOGE("KQueue::updateFd error, fd="<<fd<<", errno="<<errno);
        return Result::FAILED;
    }
    {
        std::lock_guard<std::recursive_mutex> g(data->rmtx);
        data->events = events;
    }
    //KLOGI("KQueue::updateFd, fd="<<fd<<", ev="<<events);
    return Result::OK;
#else
    return updateFd(fd, events);
#endif
}

Result KQueue::wait(uint32_t wait_ms)
{
    timespec tval = { 0, 0 };
    if(wait_ms != -1) {
        tval.tv_sec = wait_ms/1000;
        tval.tv_nsec = (wait_ms - tval.tv_sec*1000)*1000*1000;
    }
    struct kevent kevents[MAX_EVENT_NUM];
    int nevents = kevent(kqueue_fd_, 0, 0, kevents, MAX_EVENT_NUM, wait_ms == -1 ? NULL : &tval);
    if (nevents < 0) {
        if(errno != EINTR) {
            KLOGE("KQueue::wait, errno="<<errno);
        }
        //KLOGI("KQueue::wait, nevents="<<nevents<<", errno="<<errno);
    } else {
#if !defined(IOPOLL_ITEMS_USE_DATA)
        std::pair<SOCKET_FD, size_t> fds[MAX_EVENT_NUM];
        int nfds = 0;
        for (int i=0; i<nevents; ++i) {
            SOCKET_FD fd = (SOCKET_FD)kevents[i].ident;
            auto *poll_item = poll_item_mgr_->getPollItem(fd);
            if(poll_item) {
                KMEvent revents = 0;
                size_t io_size = 0;
                if (kevents[i].filter == EVFILT_READ) {
                    revents |= kEventRead;
                    io_size = kevents[i].data;
                } else if (kevents[i].filter == EVFILT_WRITE) {
                    revents |= kEventWrite;
                    io_size = kevents[i].data;
                }
#if defined(EVFILT_USER)
                else if (kevents[i].filter == EVFILT_USER) {
                    continue;
                }
#endif
                if (kevents[i].flags & EV_ERROR) {
                    revents |= kEventError;
                }
                if (!revents) {
                    continue;
                }
                if (poll_item->revents == 0) {
                    fds[nfds++] = {fd, io_size};
                }
                poll_item->revents = revents;
            }
        }
        for (int i=0; i<nfds; ++i) {
            SOCKET_FD fd = fds[i].first;
            auto *poll_item = getPollItem(fd);
            if (poll_item) {
                uint32_t revents = poll_item->revents;
                poll_item->revents = 0;
                // in case a processed event may modify this event
                revents &= poll_item->events;
                if (revents) {
                    auto &cb = poll_item->cb;
                    if(cb) cb(fd, revents, nullptr, fds[i].second);
                }
            }
        }
#else
        for (int i=0; i<nevents; ++i) {
            SOCKET_FD fd = (SOCKET_FD)kevents[i].ident;
            auto *data = static_cast<IOPollData*>(kevents[i].udata);
            if (!data) {
#if defined(EVFILT_USER)
                if (kevents[i].filter == EVFILT_USER) {
                    continue;
                }
#endif
                KLOGW("KQueue::wait no poll data for fd: " << fd);
                continue;
            }
            if (data->fd != fd) {
                KLOGW("KQueue::wait fd mismatch: " << fd << " " << data->fd);
                continue;
            }
            KMEvent revents = 0;
            size_t io_size = 0;
            if (kevents[i].filter == EVFILT_READ) {
                revents |= kEventRead;
                io_size = kevents[i].data;
            } else if (kevents[i].filter == EVFILT_WRITE) {
                revents |= kEventWrite;
                io_size = kevents[i].data;
            }
#if defined(EVFILT_USER)
            else if (kevents[i].filter == EVFILT_USER) {
                continue;
            }
#endif
            if (kevents[i].flags & EV_ERROR) {
                revents |= kEventError;
            }
            if (!revents) {
                continue;
            }
            {
                std::lock_guard<std::recursive_mutex> g(data->rmtx);
                revents &= data->events;
                if (revents) {
                    auto &cb = data->cb;
                    if(cb) cb(data->fd, revents, nullptr, io_size);
                }
            }
        }
#endif
    }
#if defined(IOPOLL_ITEMS_USE_DATA)
    processPendingPollData();
#endif
    return Result::OK;
}

void KQueue::notify()
{
    if (notifier_) {
        notifier_->notify();
    } else {
#if defined(EVFILT_USER) && defined(NOTE_TRIGGER)
        struct kevent ev;
        EV_SET(&ev, 0, EVFILT_USER, 0, NOTE_TRIGGER, 0, 0);
        while (::kevent(kqueue_fd_, &ev, 1, 0, 0, 0) == -1 && errno == EINTR) ;
#endif
    }
}

IOPoll* createKQueue() {
    return new KQueue();
}

KEV_NS_END
