/* Copyright (c) 2014, Fengping Bao <jamol@live.com>
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
#include "utils/skutils.h"

#include <algorithm>

#ifdef KUMA_OS_WIN
# include <Ws2tcpip.h>
#elif defined(KUMA_OS_OHOS)
# include <poll.h>
#else
# include <sys/poll.h>
#endif

KEV_NS_BEGIN

class VPoll : public IOPoll
{
public:
    VPoll();
    ~VPoll();
    
    bool init() override;
    Result registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb) override;
    Result unregisterFd(SOCKET_FD fd) override;
    Result updateFd(SOCKET_FD fd, KMEvent events) override;
    Result wait(uint32_t wait_ms) override;
    void notify() override;
    PollType getType() const override { return PollType::POLL; }
    bool isLevelTriggered() const override { return true; }
    
private:
    uint32_t get_events(KMEvent kuma_events) const;
    KMEvent get_kuma_events(uint32_t events) const;
    
private:
    using PollFdVector = std::vector<pollfd>;
    NotifierPtr     notifier_ { Notifier::createNotifier() };
    PollFdVector    poll_fds_;
};

VPoll::VPoll()
{
    
}

VPoll::~VPoll()
{
    poll_fds_.clear();
    poll_items_.clear();
}

bool VPoll::init()
{
    if (!notifier_->ready()) {
        if (!notifier_->init()) {
            return false;
        }
        IOCallback cb([this](SOCKET_FD, KMEvent ev, void*, size_t) { notifier_->onEvent(ev); });
        registerFd(notifier_->getReadFD(), kEventRead | kEventError, std::move(cb));
    }
    return true;
}

uint32_t VPoll::get_events(KMEvent kuma_events) const
{
    uint32_t ev = 0;
    if(kuma_events & kEventRead) {
        ev |= POLLIN;
#ifndef KUMA_OS_WIN
        ev |= POLLPRI;
#endif
    }
    if(kuma_events & kEventWrite) {
        ev |= POLLOUT;
#ifndef KUMA_OS_WIN
        ev |= POLLWRBAND;
#endif
    }
    if(kuma_events & kEventError) {
#ifndef KUMA_OS_WIN
        ev |= POLLERR | POLLHUP | POLLNVAL;
#endif
    }
    return ev;
}

KMEvent VPoll::get_kuma_events(uint32_t events) const
{
    KMEvent ev = 0;
    if(events & (POLLIN | POLLPRI)) {
        ev |= kEventRead;
    }
    if(events & (POLLOUT | POLLWRBAND)) {
        ev |= kEventWrite;
    }
    if(events & (POLLERR | POLLHUP | POLLNVAL)) {
        ev |= kEventError;
    }
    return ev;
}

Result VPoll::registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb)
{
    if (fd < 0) {
        return Result::INVALID_PARAM;
    }
    resizePollItems(fd);
    int idx = -1;
    if (INVALID_FD == poll_items_[fd].fd || -1 == poll_items_[fd].idx) { // new
        pollfd pfd;
        pfd.fd = fd;
        pfd.events = get_events(events);
        pfd.revents = 0;
        poll_fds_.push_back(pfd);
        idx = static_cast<int>(poll_fds_.size() - 1);
        poll_items_[fd].idx = idx;
    }
    poll_items_[fd].fd = fd;
    poll_items_[fd].events = events;
    poll_items_[fd].cb = std::move(cb);
    KM_INFOTRACE("VPoll::registerFd, fd="<<fd<<", events="<<events<<", index="<<idx);
    
    return Result::OK;
}

Result VPoll::unregisterFd(SOCKET_FD fd)
{
    auto max_fd = SOCKET_FD(poll_items_.size() - 1);
    KM_INFOTRACE("VPoll::unregisterFd, fd="<<fd<<", max_fd="<<max_fd);
    if (fd < 0 || -1 == max_fd || fd > max_fd) {
        KM_WARNTRACE("VPoll::unregisterFd, failed, max_fd="<<max_fd);
        return Result::INVALID_PARAM;
    }
    int idx = poll_items_[fd].idx;
    if(fd < max_fd) {
        poll_items_[fd].reset();
    } else if (fd == max_fd) {
        poll_items_.pop_back();
    }
    
    int last_idx = static_cast<int>(poll_fds_.size() - 1);
    if (idx > last_idx || -1 == idx) {
        return Result::OK;
    }
    if (idx != last_idx) {
        std::iter_swap(poll_fds_.begin()+idx, poll_fds_.end()-1);
        poll_items_[poll_fds_[idx].fd].idx = idx;
    }
    poll_fds_.pop_back();
    return Result::OK;
}

Result VPoll::updateFd(SOCKET_FD fd, KMEvent events)
{
    auto max_fd = SOCKET_FD(poll_items_.size() - 1);
    if (fd < 0 || -1 == max_fd || fd > max_fd) {
        KM_WARNTRACE("VPoll::updateFd, failed, fd="<<fd<<", max_fd="<<max_fd);
        return Result::INVALID_PARAM;
    }
    if(poll_items_[fd].fd != fd) {
        KM_WARNTRACE("VPoll::updateFd, failed, fd="<<fd<<", item_fd="<<poll_items_[fd].fd);
        return Result::INVALID_PARAM;
    }
    int idx = poll_items_[fd].idx;
    if (idx < 0 || idx >= static_cast<int>(poll_fds_.size())) {
        KM_WARNTRACE("VPoll::updateFd, failed, index=" << idx);
        return Result::INVALID_STATE;
    }
    if(poll_fds_[idx].fd != fd) {
        KM_WARNTRACE("VPoll::updateFd, failed, fd="<<fd<<", pfds_fd="<<poll_fds_[idx].fd);
        return Result::INVALID_PARAM;
    }
    poll_fds_[idx].events = get_events(events);
    poll_items_[fd].events = events;
    return Result::OK;
}

Result VPoll::wait(uint32_t wait_ms)
{
#ifdef KUMA_OS_WIN
    int num_revts = WSAPoll(&poll_fds_[0], static_cast<ULONG>(poll_fds_.size()), wait_ms);
#else
    int num_revts = poll(&poll_fds_[0], static_cast<nfds_t>(poll_fds_.size()), wait_ms);
#endif
    if (-1 == num_revts) {
        if(EINTR == errno) {
            errno = 0;
        } else {
            KM_ERRTRACE("VPoll::wait, err="<<SKUtils::getLastError());
        }
        return Result::INVALID_STATE;
    }

    // copy poll fds since event handler may unregister fd
    PollFdVector poll_fds = poll_fds_;
    for (size_t i = 0; i < poll_fds.size() && num_revts > 0; ++i) {
        if(poll_fds[i].revents) {
            --num_revts;
            auto fd = poll_fds[i].fd;
            if(fd >= 0 && fd < static_cast<SOCKET_FD>(poll_items_.size())) {
                auto &item = poll_items_[fd];
                auto revents = get_kuma_events(poll_fds[i].revents);
                revents &= item.events;
                if (revents && item.cb) {
                    item.cb(fd, revents, nullptr, 0);
                }
            }
        }
    }
    return Result::OK;
}

void VPoll::notify()
{
    notifier_->notify();
}

IOPoll* createVPoll() {
    return new VPoll();
}

KEV_NS_END
