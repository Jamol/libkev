/* Copyright (c) 2014-2025, Fengping Bao <jamol@live.com>
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

#include <sys/epoll.h>

KEV_NS_BEGIN

#define MAX_EPOLL_FDS   5000
#define MAX_EVENT_NUM   500

class EPoll : public IOPoll
{
public:
    EPoll();
    ~EPoll();

    bool init() override;
    Result registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb) override;
    Result unregisterFd(SOCKET_FD fd) override;
    Result updateFd(SOCKET_FD fd, KMEvent events) override;
    Result wait(uint32_t wait_time_ms) override;
    void notify() override;
    PollType getType() const override { return PollType::EPOLL; }
    bool isLevelTriggered() const override { return false; }

private:
    uint32_t get_events(KMEvent kuma_events) const;
    KMEvent get_kuma_events(uint32_t events) const;

private:
    int             epoll_fd_ { INVALID_FD };
    NotifierPtr     notifier_ { std::move(Notifier::createNotifier()) };
};

EPoll::EPoll()
{

}

EPoll::~EPoll()
{
    if(INVALID_FD != epoll_fd_) {
        close(epoll_fd_);
        epoll_fd_ = INVALID_FD;
    }
}

bool EPoll::init()
{
    if(INVALID_FD != epoll_fd_) {
        return true;
    }
    epoll_fd_ = epoll_create(MAX_EPOLL_FDS);
    if(INVALID_FD == epoll_fd_) {
        KM_ERRTRACE("EPoll::init, epoll_create failed, errno=" << errno);
        return false;
    }
    if (!notifier_->ready()) {
        if(!notifier_->init()) {
            KM_ERRTRACE("EPoll::init, notifier init failed");
            close(epoll_fd_);
            epoll_fd_ = INVALID_FD;
            return false;
        }
        IOCallback cb ([this](SOCKET_FD, KMEvent ev, void*, size_t) { notifier_->onEvent(ev); });
        registerFd(notifier_->getReadFD(), kEventRead | kEventError, std::move(cb));
    }
    return true;
}

uint32_t EPoll::get_events(KMEvent kuma_events) const
{
    uint32_t ev = EPOLLET;//EPOLLIN | EPOLLOUT | EPOLLERR | EPOLLHUP | EPOLLET;
    if(kuma_events & kEventRead) {
        ev |= EPOLLIN;
    }
    if(kuma_events & kEventWrite) {
        ev |= EPOLLOUT;
    }
    if(kuma_events & kEventError) {
        ev |= EPOLLERR | EPOLLHUP;
    }
    return ev;
}

KMEvent EPoll::get_kuma_events(uint32_t events) const
{
    KMEvent ev = 0;
    if(events & EPOLLIN) {
        ev |= kEventRead;
    }
    if(events & EPOLLOUT) {
        ev |= kEventWrite;
    }
    if(events & (EPOLLERR | EPOLLHUP)) {
        ev |= kEventError;
    }
    return ev;
}

Result EPoll::registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb)
{
    if (fd < 0) {
        return Result::INVALID_PARAM;
    }
    resizePollItems(fd);
    int epoll_op = EPOLL_CTL_ADD;
    if (INVALID_FD != poll_items_[fd].fd) {
        epoll_op = EPOLL_CTL_MOD;
    }
    poll_items_[fd].fd = fd;
    poll_items_[fd].events = events;
    poll_items_[fd].cb = std::move(cb);
    struct epoll_event evt = {0};
    evt.data.ptr = (void*)(long)fd;
    evt.events = get_events(events);//EPOLLIN | EPOLLOUT | EPOLLERR | EPOLLHUP | EPOLLET;
    if(epoll_ctl(epoll_fd_, epoll_op, fd, &evt) < 0) {
        KM_ERRTRACE("EPoll::registerFd error, fd=" << fd << ", ev=" << evt.events << ", errno=" << errno);
        poll_items_[fd].reset();
        return Result::FAILED;
    }
    KM_INFOTRACE("EPoll::registerFd, fd=" << fd << ", ev=" << evt.events);

    return Result::OK;
}

Result EPoll::unregisterFd(SOCKET_FD fd)
{
    int max_fd = int(poll_items_.size() - 1);
    KM_INFOTRACE("EPoll::unregisterFd, fd="<<fd<<", max_fd="<<max_fd);
    if (fd < 0 || fd > max_fd) {
        KM_WARNTRACE("EPoll::unregisterFd, failed, max_fd=" << max_fd);
        return Result::INVALID_PARAM;
    }
    epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, NULL);
    if(fd < max_fd) {
        poll_items_[fd].reset();
    } else if (fd == max_fd) {
        poll_items_.pop_back();
    }
    return Result::OK;
}

Result EPoll::updateFd(SOCKET_FD fd, KMEvent events)
{
    if(fd < 0 || fd >= poll_items_.size() || INVALID_FD == poll_items_[fd].fd) {
        return Result::FAILED;
    }
    
    if (poll_items_[fd].events == events) {
        return Result::OK;
    }
    
    struct epoll_event evt = {0};
    evt.data.ptr = (void*)(long)fd;
    evt.events = get_events(events);
    if(epoll_ctl(epoll_fd_, EPOLL_CTL_MOD, fd, &evt) < 0) {
        KM_ERRTRACE("EPoll::updateFd error, fd="<<fd<<", errno="<<errno);
        return Result::FAILED;
    }
    poll_items_[fd].events = events;
    return Result::OK;
}

Result EPoll::wait(uint32_t wait_ms)
{
    struct epoll_event events[MAX_EVENT_NUM];
    int nfds = epoll_wait(epoll_fd_, events, MAX_EVENT_NUM, wait_ms);
    if (nfds < 0) {
        if(errno != EINTR) {
            KM_ERRTRACE("EPoll::wait, errno="<<errno);
        }
        KM_INFOTRACE("EPoll::wait, nfds="<<nfds<<", errno="<<errno);
    } else {
        for (int i=0; i<nfds; ++i) {
            SOCKET_FD fd = (SOCKET_FD)(long)events[i].data.ptr;
            if(fd < poll_items_.size()) {
                auto revents = get_kuma_events(events[i].events);
                revents &= poll_items_[fd].events;
                if (revents) {
                    auto &cb = poll_items_[fd].cb;
                    if(cb) cb(fd, revents, nullptr, 0);
                }
            }
        }
    }
    return Result::OK;
}

void EPoll::notify()
{
    notifier_->notify();
}

IOPoll* createEPoll() {
    return new EPoll();
}

KEV_NS_END
