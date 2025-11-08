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

const size_t kBatchSize = 16;

KEV_NS_BEGIN

class WevPoll : public IOPoll, public IOPollItem<PollItem>
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
    void onSocketEvent(WSAEVENT h, const std::vector<SOCKET_FD> &fds);
    void onSignalEvent();
    int getNextEventIndex() const {
        int ev_idx = -1;
        size_t prev_size = 0;
        for (int i = 1; i < WSA_MAXIMUM_WAIT_EVENTS; ++i) {
            if (event_fds_[i].size() < kBatchSize) {
                ev_idx = i;
                break;
            }
            if (i == 1) {
                prev_size = event_fds_[i].size();
            } else {
                if (event_fds_[i].size() < prev_size) {
                    ev_idx = i;
                    break;
                }
            }
        }
        if (ev_idx == -1) {
            ev_idx = 1;
        }
        return ev_idx;
    }
    
private:
    using PollFdVector = std::vector<std::pair<pollfd, int>>;
    WSAEVENT        events_[WSA_MAXIMUM_WAIT_EVENTS];
    std::vector<SOCKET_FD> event_fds_[WSA_MAXIMUM_WAIT_EVENTS];
    PollFdVector    poll_fds_;
};

WevPoll::WevPoll()
{
    for(auto &h : events_) {
        h = WSA_INVALID_EVENT;
    }
    events_[0] = WSACreateEvent();
}

WevPoll::~WevPoll()
{
    poll_fds_.clear();
    for (auto &h : events_) {
        if (h != WSA_INVALID_EVENT) {
            WSACloseEvent(h);
            h = WSA_INVALID_EVENT;
        }
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
    auto *poll_item = getPollItem(fd, true);
    if (!poll_item) {
        KM_ERRTRACE("WevPoll::registerFd no poll item, fd=" << fd << ", sz=" << getPollItemSize());
        return Result::BUFFER_TOO_SMALL;
    }
    int idx = -1;
    if (INVALID_FD == poll_item->fd || -1 == poll_item->idx) { // new
        pollfd pfd{fd, (short)get_events(events), 0};
        {
            int type = SOCK_STREAM;
            socklen_t len = sizeof(type);
            getsockopt(fd, SOL_SOCKET, SO_TYPE, (char*)&type, &len);
            if (type == SOCK_STREAM) {
                pfd.events |= FD_CONNECT;
            }
        }
        int ev_idx = getNextEventIndex();
        if (events_[ev_idx] == WSA_INVALID_EVENT) {
            events_[ev_idx] = WSACreateEvent();
        }
        WSAEventSelect(fd, events_[ev_idx], pfd.events);
        event_fds_[ev_idx].emplace_back(fd);
        poll_fds_.emplace_back(std::pair<pollfd, int>{pfd, ev_idx});
        poll_item->idx = static_cast<int>(poll_fds_.size() - 1);
    }
    poll_item->fd = fd;
    poll_item->events = events;
    poll_item->cb = std::move(cb);
    KM_INFOTRACE("WevPoll::registerFd, fd="<<fd<<", events="<<events<<", index="<<idx);
    
    return Result::OK;
}

Result WevPoll::unregisterFd(SOCKET_FD fd)
{
    auto sz = getPollItemSize();
    KM_INFOTRACE("WevPoll::unregisterFd, fd="<<fd<<", sz="<<sz);
    auto *poll_item = getPollItem(fd);
    if (!poll_item) {
        KM_ERRTRACE("WevPoll::unregisterFd failed, fd=" << fd);
        return Result::INVALID_PARAM;
    }
    int idx = poll_item->idx;
    clearPollItem(fd);
    
    int last_idx = static_cast<int>(poll_fds_.size() - 1);
    if (idx > last_idx || -1 == idx) {
        return Result::OK;
    }
    WSAEventSelect(fd, NULL, 0);
    int ev_idx = poll_fds_[idx].second;
    if (ev_idx > 0 && ev_idx < WSA_MAXIMUM_WAIT_EVENTS) {
        event_fds_[ev_idx].erase(
            std::remove(event_fds_[ev_idx].begin(), event_fds_[ev_idx].end(), fd),
            event_fds_[ev_idx].end()
        );
    }
    poll_fds_[idx].second = -1;
    if (idx != last_idx) {
        std::iter_swap(poll_fds_.begin()+idx, poll_fds_.end()-1);
        auto *pi = getPollItem(poll_fds_[idx].first.fd);
        if (pi) {
            pi->idx = idx;
        }
    }
    poll_fds_.pop_back();
    return Result::OK;
}

Result WevPoll::updateFd(SOCKET_FD fd, KMEvent events)
{
    auto *poll_item = getPollItem(fd);
    if (!poll_item || INVALID_FD == poll_item->fd) {
        return Result::INVALID_PARAM;
    }
    if(poll_item->fd != fd) {
        KM_WARNTRACE("WevPoll::updateFd, failed, fd="<<fd<<", item_fd="<<poll_item->fd);
        return Result::INVALID_PARAM;
    }
    int idx = poll_item->idx;
    if (idx < 0 || idx >= static_cast<int>(poll_fds_.size())) {
        KM_WARNTRACE("WevPoll::updateFd, failed, index=" << idx);
        return Result::INVALID_STATE;
    }
    if(poll_fds_[idx].first.fd != fd) {
        KM_WARNTRACE("WevPoll::updateFd, failed, fd="<<fd<<", pfds_fd="<<poll_fds_[idx].first.fd);
        return Result::INVALID_PARAM;
    }
    poll_fds_[idx].first.events = get_events(events);
    poll_item->events = events;
    int ev_idx = poll_fds_[idx].second;
    if (ev_idx > 0 && ev_idx < WSA_MAXIMUM_WAIT_EVENTS) {
        WSAEventSelect(fd, events_[ev_idx], poll_fds_[idx].first.events);
    }
    return Result::OK;
}

Result WevPoll::wait(uint32_t wait_ms)
{
    size_t ev_count = 1;
    if (!poll_fds_.empty()) {
        size_t fd_count = 0;
        for (int i = 0; i < WSA_MAXIMUM_WAIT_EVENTS; ++i) {
            if (event_fds_[i].size() > 0) {
                ev_count = i + 1;
                fd_count += event_fds_[i].size();
                if (fd_count >= poll_fds_.size()) {
                    break;
                }
            }
        }
    }
    DWORD ret = WSAWaitForMultipleEvents((DWORD)ev_count, events_, FALSE, wait_ms, FALSE);
    if (ret == WSA_WAIT_FAILED) {
        KM_ERRTRACE("WevPoll::wait, err=" << WSAGetLastError());
        return Result::POLL_ERROR;
    } else if (ret == WSA_WAIT_TIMEOUT) {
        return Result::OK;
    } else {
        int index = ret - WSA_WAIT_EVENT_0;
        if (index == 0) {
            onSignalEvent();
        } else if (index < WSA_MAXIMUM_WAIT_EVENTS) {
            auto fds = event_fds_[index];
            onSocketEvent(events_[index], fds);
        }
    }
    return Result::OK;
}

void WevPoll::onSocketEvent(WSAEVENT h, const std::vector<SOCKET_FD> &fds)
{
    for (auto fd : fds) {
        auto *poll_item = getPollItem(fd);
        if (!poll_item) {
            continue;
        }
        WSANETWORKEVENTS net_events;
        if (WSAEnumNetworkEvents(fd, h, &net_events) == SOCKET_ERROR) {
            continue;
        }
        KMEvent revents = 0;
        if (net_events.lNetworkEvents & FD_READ) {
            revents |= kEventRead;
            if (net_events.iErrorCode[FD_READ_BIT] != 0) {
                KM_WARNTRACE("WevPoll::onSocketEvent, FD_READ error, fd="<<fd
                            <<", err="<<net_events.iErrorCode[FD_READ_BIT]);
            }
        }
        if (net_events.lNetworkEvents & FD_WRITE) {
            revents |= kEventWrite;
            if (net_events.iErrorCode[FD_WRITE_BIT] != 0) {
                KM_WARNTRACE("WevPoll::onSocketEvent, FD_WRITE error, fd="<<fd
                            <<", err="<<net_events.iErrorCode[FD_WRITE_BIT]);
            }
        }
        if (net_events.lNetworkEvents & FD_CONNECT) {
            if (net_events.iErrorCode[FD_CONNECT_BIT] == 0) {
                revents |= kEventRead;
            } else {
                revents |= kEventError;
                KM_WARNTRACE("WevPoll::onSocketEvent, FD_CONNECT error, fd="<<fd
                            <<", err="<<net_events.iErrorCode[FD_CONNECT_BIT]);
            }
        }
        if (net_events.lNetworkEvents & FD_ACCEPT) {
            revents |= kEventRead;
        }
        if (net_events.lNetworkEvents & FD_CLOSE) {
            revents |= kEventError;
            if (net_events.iErrorCode[FD_CLOSE_BIT] != 0) {
                KM_WARNTRACE("WevPoll::onSocketEvent, FD_CLOSE error, fd="<<fd
                            <<", err="<<net_events.iErrorCode[FD_CLOSE_BIT]);
            }
        }
        revents &= poll_item->events;
        if (revents != 0 && poll_item->cb) {
            poll_item->cb(fd, revents, nullptr, 0);
        }
    }
}

void WevPoll::onSignalEvent()
{
    WSAResetEvent(events_[0]);
}

void WevPoll::notify()
{
    if (events_[0] != WSA_INVALID_EVENT) {
        WSASetEvent(events_[0]);
    }
}

IOPoll* createWevPoll() {
    return new WevPoll();
}

KEV_NS_END
