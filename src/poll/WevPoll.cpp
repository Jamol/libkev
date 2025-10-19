/* Copyright (c) 2025, Fengping Bao <jamol@live.com>
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
#include "utils/kmtrace.h"
#include "utils/skutils.h"

#include <Ws2tcpip.h>

KEV_NS_BEGIN

class WevPoll : public IOPoll
{
public:
    WevPoll();
    ~WevPoll();
    
    bool init() override;
    Result registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb) override;
    Result unregisterFd(SOCKET_FD fd) override;
    Result updateFd(SOCKET_FD fd, KMEvent events) override;
    Result wait(uint32_t wait_ms) override;
    void notify() override;
    PollType getType() const override { return PollType::WSAEV; }
    bool isLevelTriggered() const override { return true; }
    
private:
    uint32_t get_events(KMEvent kuma_events) const;
    KMEvent get_kuma_events(uint32_t events) const;
    void onSocketEvent();
    void onSignalEvent();
    
private:
    using PollFdVector = std::vector<pollfd>;
    WSAEVENT        socket_ev_{ WSA_INVALID_EVENT };
    WSAEVENT        signal_ev_{ WSA_INVALID_EVENT };
    PollFdVector    poll_fds_;
};

WevPoll::WevPoll()
{
    socket_ev_ = WSACreateEvent();
    signal_ev_ = WSACreateEvent();
}

WevPoll::~WevPoll()
{
    poll_fds_.clear();
    poll_items_.clear();
    if (socket_ev_ != WSA_INVALID_EVENT) {
        WSACloseEvent(socket_ev_);
        socket_ev_ = WSA_INVALID_EVENT;
    }
    if (signal_ev_ != WSA_INVALID_EVENT) {
        WSACloseEvent(signal_ev_);
        signal_ev_ = WSA_INVALID_EVENT;
    }
}

bool WevPoll::init()
{
    return true;
}

uint32_t WevPoll::get_events(KMEvent kuma_events) const
{
    uint32_t ev = 0;
    if(kuma_events & kEventRead) {
        ev |= FD_READ;
    }
    if(kuma_events & kEventWrite) {
        ev |= FD_WRITE;
    }
    if(kuma_events & kEventError) {
        ev |= FD_CLOSE;
    }
    return ev;
}

KMEvent WevPoll::get_kuma_events(uint32_t events) const
{
    KMEvent ev = 0;
    if(events & (FD_READ | FD_CONNECT)) {
        ev |= kEventRead;
    }
    if(events & FD_WRITE) {
        ev |= kEventWrite;
    }
    if(events & FD_CLOSE) {
        ev |= kEventError;
    }
    return ev;
}

Result WevPoll::registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb)
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
        {
            int type = SOCK_STREAM;
            socklen_t len = sizeof(type);
            getsockopt(fd, SOL_SOCKET, SO_TYPE, (char*)&type, &len);
            if (type == SOCK_STREAM) {
                pfd.events |= FD_CONNECT;
            }
        }
        poll_fds_.push_back(pfd);
        idx = static_cast<int>(poll_fds_.size() - 1);
        poll_items_[fd].idx = idx;
    }
    poll_items_[fd].fd = fd;
    poll_items_[fd].events = events;
    poll_items_[fd].cb = std::move(cb);
    KM_INFOTRACE("WevPoll::registerFd, fd="<<fd<<", events="<<events<<", index="<<idx);
    
    return Result::OK;
}

Result WevPoll::unregisterFd(SOCKET_FD fd)
{
    auto max_fd = SOCKET_FD(poll_items_.size() - 1);
    KM_INFOTRACE("WevPoll::unregisterFd, fd="<<fd<<", max_fd="<<max_fd);
    if (fd < 0 || -1 == max_fd || fd > max_fd) {
        KM_WARNTRACE("WevPoll::unregisterFd, failed, max_fd="<<max_fd);
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
    WSAEventSelect(fd, socket_ev_, 0);
    return Result::OK;
}

Result WevPoll::updateFd(SOCKET_FD fd, KMEvent events)
{
    auto max_fd = SOCKET_FD(poll_items_.size() - 1);
    if (fd < 0 || -1 == max_fd || fd > max_fd) {
        KM_WARNTRACE("WevPoll::updateFd, failed, fd="<<fd<<", max_fd="<<max_fd);
        return Result::INVALID_PARAM;
    }
    if(poll_items_[fd].fd != fd) {
        KM_WARNTRACE("WevPoll::updateFd, failed, fd="<<fd<<", item_fd="<<poll_items_[fd].fd);
        return Result::INVALID_PARAM;
    }
    int idx = poll_items_[fd].idx;
    if (idx < 0 || idx >= static_cast<int>(poll_fds_.size())) {
        KM_WARNTRACE("WevPoll::updateFd, failed, index=" << idx);
        return Result::INVALID_STATE;
    }
    if(poll_fds_[idx].fd != fd) {
        KM_WARNTRACE("WevPoll::updateFd, failed, fd="<<fd<<", pfds_fd="<<poll_fds_[idx].fd);
        return Result::INVALID_PARAM;
    }
    poll_fds_[idx].events = get_events(events);
    poll_items_[fd].events = events;
    return Result::OK;
}

Result WevPoll::wait(uint32_t wait_ms)
{
    WSAEVENT events[2] = { socket_ev_, signal_ev_ };
    for (auto const &pfd : poll_fds_) {
        WSAEventSelect(pfd.fd, socket_ev_, pfd.events);
    }
    DWORD ret = WSAWaitForMultipleEvents(ARRAYSIZE(events), events, FALSE, wait_ms, FALSE);
    if (ret == WSA_WAIT_FAILED) {
        KM_ERRTRACE("WevPoll::wait, err=" << WSAGetLastError());
        return Result::POLL_ERROR;
    } else if (ret == WSA_WAIT_TIMEOUT) {
        return Result::OK;
    } else {
        int index = ret - WSA_WAIT_EVENT_0;
        if (index == 0) {
            onSocketEvent();
        } else if (index == 1) {
            onSignalEvent();
        }
        WSAResetEvent(socket_ev_);
    }
    return Result::OK;
}

void WevPoll::onSocketEvent()
{
    for (auto const &pfd : poll_fds_) {
        if (pfd.events == 0 || pfd.fd >= poll_items_.size()) {
            continue;
        }
        WSANETWORKEVENTS net_events;
        if (WSAEnumNetworkEvents(pfd.fd, socket_ev_, &net_events) == SOCKET_ERROR) {
            continue;
        }
        KMEvent revents = 0;
        if (net_events.lNetworkEvents & FD_READ) {
            revents |= kEventRead;
            if (net_events.iErrorCode[FD_READ_BIT] != 0) {
                KM_WARNTRACE("WevPoll::onSocketEvent, FD_READ error, fd="<<pfd.fd
                            <<", err="<<net_events.iErrorCode[FD_READ_BIT]);
            }
        }
        if (net_events.lNetworkEvents & FD_WRITE) {
            revents |= kEventWrite;
            if (net_events.iErrorCode[FD_WRITE_BIT] != 0) {
                KM_WARNTRACE("WevPoll::onSocketEvent, FD_WRITE error, fd="<<pfd.fd
                            <<", err="<<net_events.iErrorCode[FD_WRITE_BIT]);
            }
        }
        if (net_events.lNetworkEvents & FD_CONNECT) {
            if (net_events.iErrorCode[FD_CONNECT_BIT] == 0) {
                revents |= kEventRead;
            } else {
                revents |= kEventError;
                KM_WARNTRACE("WevPoll::onSocketEvent, FD_CONNECT error, fd="<<pfd.fd
                            <<", err="<<net_events.iErrorCode[FD_CONNECT_BIT]);
            }
        }
        if (net_events.lNetworkEvents & FD_ACCEPT) {
            revents |= kEventRead;
        }
        if (net_events.lNetworkEvents & FD_CLOSE) {
            revents |= kEventError;
            if (net_events.iErrorCode[FD_CLOSE_BIT] != 0) {
                KM_WARNTRACE("WevPoll::onSocketEvent, FD_CLOSE error, fd="<<pfd.fd
                            <<", err="<<net_events.iErrorCode[FD_CLOSE_BIT]);
            }
        }
        auto &item = poll_items_[pfd.fd];
        revents &= item.events;
        if (revents != 0 && item.cb) {
            item.cb(pfd.fd, revents, nullptr, 0);
        }
    }
}

void WevPoll::onSignalEvent()
{
    WSAResetEvent(signal_ev_);
}

void WevPoll::notify()
{
    if (signal_ev_ != WSA_INVALID_EVENT) {
        WSASetEvent(signal_ev_);
    }
}

IOPoll* createWevPoll() {
    return new WevPoll();
}

KEV_NS_END
