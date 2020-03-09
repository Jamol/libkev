/* Copyright (c) 2016, Fengping Bao <jamol@live.com>
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
#include "util/kmtrace.h"

#include <sys/types.h>
#include <sys/event.h>
#include <sys/time.h>

KEV_NS_BEGIN

#define MAX_EVENT_NUM   500

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
    
private:
    int             kqueue_fd_ { -1 };
    NotifierPtr     notifier_ { Notifier::createNotifier() };
    
    // on ET mode (EV_CLEAR is set), it seems EVFILT_READ won't be triggered
    // if EVFILT_READ is set after data arrived
    bool            work_on_et_mode_ { false };
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
}

bool KQueue::init()
{
    kqueue_fd_ = ::kqueue();
    if(INVALID_FD == kqueue_fd_) {
        return false;
    }
    if (!notifier_->ready()) {
        if(!notifier_->init()) {
            return false;
        }
        IOCallback cb ([this](KMEvent ev, void*, size_t) { notifier_->onEvent(ev); });
        registerFd(notifier_->getReadFD(), kEventRead|kEventError, std::move(cb));
    }
    return true;
}

Result KQueue::registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb)
{
    if (fd < 0) {
        return Result::INVALID_PARAM;
    }
    resizePollItems(fd);
    struct kevent kevents[2];
    int nchanges = 0;
    if (INVALID_FD != poll_items_[fd].fd) {
        if (!!(poll_items_[fd].events & kEventRead) && !(events & kEventRead)) {
            EV_SET(&kevents[nchanges++], fd, EVFILT_READ, EV_DELETE, 0, 0, 0);
            poll_items_[fd].events &= ~kEventRead;
        }
        if (!!(poll_items_[fd].events & kEventWrite) && !(events & kEventWrite)) {
            EV_SET(&kevents[nchanges++], fd, EVFILT_WRITE, EV_DELETE, 0, 0, 0);
            poll_items_[fd].events &= ~kEventWrite;
        }
        ::kevent(kqueue_fd_, kevents, nchanges, 0, 0, 0);
        if (poll_items_[fd].events == events) {
            poll_items_[fd].cb = std::move(cb);
            return Result::OK;
        }
    }
    nchanges = 0;
    unsigned short op = EV_ADD;
    if (work_on_et_mode_) {
        op |= EV_CLEAR;
    }
    if (events & kEventRead) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_READ, op , 0, 0, 0);
    }
    if (events & kEventWrite) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_WRITE, op , 0, 0, 0);
    }
    poll_items_[fd].fd = fd;
    poll_items_[fd].events = events;
    poll_items_[fd].cb = std::move(cb);
    
    if(::kevent(kqueue_fd_, kevents, nchanges, 0, 0, 0) == -1) {
        KM_ERRTRACE("KQueue::registerFd error, fd=" << fd << ", ev=" << events << ", errno=" << errno);
        return Result::FAILED;
    }
    KM_INFOTRACE("KQueue::registerFd, fd=" << fd << ", ev=" << events);

    return Result::OK;
}

Result KQueue::unregisterFd(SOCKET_FD fd)
{
    int max_fd = int(poll_items_.size() - 1);
    KM_INFOTRACE("KQueue::unregisterFd, fd="<<fd<<", max_fd="<<max_fd);
    if (fd < 0 || fd > max_fd) {
        KM_WARNTRACE("KQueue::unregisterFd, failed, max_fd=" << max_fd);
        return Result::INVALID_PARAM;
    }
    struct kevent kevents[2];
    int nchanges = 0;
    if (poll_items_[fd].events & kEventRead) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_READ, EV_DELETE, 0, 0, 0);
    }
    if (poll_items_[fd].events & kEventWrite) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_WRITE, EV_DELETE, 0, 0, 0);
    }
    ::kevent(kqueue_fd_, kevents, nchanges, 0, 0, 0);
    if(fd < max_fd) {
        poll_items_[fd].reset();
    } else if (fd == max_fd) {
        poll_items_.pop_back();
    }
    return Result::OK;
}

Result KQueue::updateFd(SOCKET_FD fd, KMEvent events)
{
    if(fd < 0 || fd >= poll_items_.size() || INVALID_FD == poll_items_[fd].fd) {
        return Result::FAILED;
    }
    
    struct kevent kevents[2];
    int nchanges = 0;
    if (!!(poll_items_[fd].events & kEventRead) && !(events & kEventRead)) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_READ, EV_DELETE, 0, 0, 0);
        poll_items_[fd].events &= ~kEventRead;
    }
    if (!!(poll_items_[fd].events & kEventWrite) && !(events & kEventWrite)) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_WRITE, EV_DELETE, 0, 0, 0);
        poll_items_[fd].events &= ~kEventWrite;
    }
    if (nchanges) { // remove events
        ::kevent(kqueue_fd_, kevents, nchanges, 0, 0, 0);
    }
    if (poll_items_[fd].events == events) {
        return Result::OK;
    }
    nchanges = 0;
    unsigned short op = EV_ADD;
    if (work_on_et_mode_) {
        op |= EV_CLEAR;
    }
    if (events & kEventRead) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_READ, op , 0, 0, 0);
    }
    if (events & kEventWrite) {
        EV_SET(&kevents[nchanges++], fd, EVFILT_WRITE, op , 0, 0, 0);
    }
    if(nchanges && ::kevent(kqueue_fd_, kevents, nchanges, 0, 0, 0) == -1) {
        KM_ERRTRACE("KQueue::updateFd error, fd="<<fd<<", errno="<<errno);
        return Result::FAILED;
    }
    poll_items_[fd].events = events;
    //KM_INFOTRACE("KQueue::updateFd, fd="<<fd<<", ev="<<events);
    return Result::OK;
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
            KM_ERRTRACE("KQueue::wait, errno="<<errno);
        }
        KM_INFOTRACE("KQueue::wait, nevents="<<nevents<<", errno="<<errno);
    } else {
        SOCKET_FD fds[MAX_EVENT_NUM] = { INVALID_FD };
        int nfds = 0;
        int max_fd = int(poll_items_.size() - 1);
        for (int i=0; i<nevents; ++i) {
            SOCKET_FD fd = (SOCKET_FD)kevents[i].ident;
            if(fd >= 0 && fd <= max_fd) {
                KMEvent revents = 0;
                if (kevents[i].filter == EVFILT_READ) {
                    revents |= kEventRead;
                } else if (kevents[i].filter == EVFILT_WRITE) {
                    revents |= kEventWrite;
                }
                if (kevents[i].flags & EV_ERROR) {
                    revents |= kEventError;
                }
                if (!revents) {
                    continue;
                }
                if (poll_items_[fd].revents == 0) {
                    fds[nfds++] = fd;
                }
                poll_items_[fd].revents = revents;
            }
        }
        for (int i=0; i<nfds; ++i) {
            SOCKET_FD fd = fds[i];
            if (fd < poll_items_.size()) {
                uint32_t revents = poll_items_[fd].revents;
                poll_items_[fd].revents = 0;
                // in case a processed event may modify this event
                revents &= poll_items_[fd].events;
                if (revents) {
                    auto &cb = poll_items_[fd].cb;
                    if(cb) cb(revents, nullptr, 0);
                }
            }
        }
    }
    return Result::OK;
}

void KQueue::notify()
{
    notifier_->notify();
}

IOPoll* createKQueue() {
    return new KQueue();
}

KEV_NS_END
