/* Copyright (c) 2014~2025, Fengping Bao <jamol@live.com>
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

#include <algorithm>

KEV_NS_BEGIN

#ifdef KUMA_OS_WIN
class FdSet
{
public:
    FdSet() {
        size_t new_size = sizeof(ws_fd_set) + sizeof(SOCKET_FD) * (capacity_ - 1);
        fd_set_ = (ws_fd_set*)malloc(new_size);
        if (fd_set_) {
            fd_set_->fd_count = 0;
        }
    }
    FdSet(const FdSet& other) {
        capacity_ = other.capacity_;
        size_t new_size = sizeof(ws_fd_set) + sizeof(SOCKET_FD) * (capacity_ - 1);
        fd_set_ = (ws_fd_set*)malloc(new_size);
        if (fd_set_) {
            fd_set_->fd_count = other.fd_set_->fd_count;
            memcpy(&fd_set_->fd_array[0], &other.fd_set_->fd_array[0],
                   sizeof(SOCKET_FD) * (fd_set_->fd_count));
        }
    }
    FdSet(FdSet&& other) noexcept {
        fd_set_ = other.fd_set_;
        capacity_ = other.capacity_;
        other.fd_set_ = nullptr;
        other.capacity_ = 0;
    }
    ~FdSet() {
        if (fd_set_) {
            free(fd_set_);
            fd_set_ = nullptr;
        }
        capacity_ = 0;
    }

    FdSet& operator=(const FdSet& other) {
        if (this != &other) {
            if (capacity_ != other.capacity_) {
                if (fd_set_) {
                    free(fd_set_);
                }
                capacity_ = other.capacity_;
                size_t new_size = sizeof(ws_fd_set) + sizeof(SOCKET_FD) * (capacity_ - 1);
                fd_set_ = (ws_fd_set*)malloc(new_size);
            }
            if (fd_set_) {
                fd_set_->fd_count = other.fd_set_->fd_count;
                memcpy(&fd_set_->fd_array[0], &other.fd_set_->fd_array[0],
                    sizeof(SOCKET_FD) * (fd_set_->fd_count));
            }
        }
        return *this;
    }
    FdSet& operator=(FdSet&& other) {
        if (this != &other) {
            if (fd_set_) {
                free(fd_set_);
            }
            fd_set_ = other.fd_set_;
            capacity_ = other.capacity_;
            other.fd_set_ = nullptr;
            other.capacity_ = 0;
        }
        return *this;
    }
    operator fd_set*() {
        return reinterpret_cast<fd_set*>(fd_set_);
    }

    bool set(SOCKET_FD fd) {
        for (u_int i = 0; i < fd_set_->fd_count; ++i) {
            if (fd_set_->fd_array[i] == fd) {
                return true;
            }
        }
        if (reserve(fd_set_->fd_count + 1)) {
            fd_set_->fd_array[fd_set_->fd_count++] = fd;
            return true;
        }
        KM_ERRTRACE("FD_SET failed, fd=" << fd << ", fd_count=" << fd_set_->fd_count);
        return false;
    }

    void clear(SOCKET_FD fd) {
        for (u_int i = 0; i < fd_set_->fd_count; ++i) {
            if (fd_set_->fd_array[i] == fd) {
                if (i != fd_set_->fd_count - 1) {
                    fd_set_->fd_array[i] = fd_set_->fd_array[fd_set_->fd_count - 1];
                }
                fd_set_->fd_count--;
                return;
            }
        }
    }

    void zero() {
        if (fd_set_) {
            fd_set_->fd_count = 0;
        }
    }

    bool is_set(SOCKET_FD fd) const {
        return FD_ISSET(fd, const_cast<fd_set*>(reinterpret_cast<const fd_set*>(fd_set_)));
    }
private:
    struct ws_fd_set {
        u_int fd_count;
        SOCKET_FD fd_array[1];
    };
    bool reserve(u_int n) {
        if (n <= capacity_) {
            return true;
        }
        u_int new_cap = capacity_ + 128;
        if (n > new_cap) {
            new_cap = n;
        }
        size_t new_size = sizeof(ws_fd_set) + sizeof(SOCKET_FD) * (new_cap - 1);
        ws_fd_set* new_set = (ws_fd_set*)realloc(fd_set_, new_size);
        if (new_set) {
            fd_set_ = new_set;
            capacity_ = new_cap;
            return true;
        }
        return false;
    }
private:
    ws_fd_set* fd_set_ { nullptr };
    u_int      capacity_ { 64 };
};
#else
class FdSet
{
public:
    FdSet() {
        FD_ZERO(&fd_set_);
    }
    FdSet(const FdSet& other) {
        fd_set_ = other.fd_set_;
    }
    ~FdSet() {}

    FdSet& operator=(const FdSet& other) {
        if (this != &other) {
            fd_set_ = other.fd_set_;
        }
        return *this;
    }
    operator fd_set*()
    {
        return &fd_set_;
    }
    
    bool set(SOCKET_FD fd) {
        if (fd < FD_SETSIZE) {
            FD_SET(fd, &fd_set_);
            return true;
        }
        KM_ERRTRACE("FD_SET failed, fd=" << fd << ", FD_SETSIZE=" << FD_SETSIZE);
        return false;
    }
    
    void clear(SOCKET_FD fd) {
        FD_CLR(fd, &fd_set_);
    }
    
    void zero() {
        FD_ZERO(&fd_set_);
    }
    
    bool is_set(SOCKET_FD fd) const {
        return FD_ISSET(fd, const_cast<fd_set*>(&fd_set_));
    }

private:
    fd_set fd_set_;
};
#endif

class SelectPoll : public IOPoll, public IOPollItem<PollItem>
{
public:
    SelectPoll();
    ~SelectPoll();
    
    bool init() override;
    Result registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb) override;
    Result unregisterFd(SOCKET_FD fd) override;
    Result updateFd(SOCKET_FD fd, KMEvent events) override;
    Result wait(uint32_t wait_time_ms) override;
    void notify() override;
    PollType getType() const override { return PollType::SELECT; }
    bool isLevelTriggered() const override { return true; }

private:
    void updateFdSet(SOCKET_FD fd, KMEvent events);
    
private:
    struct PollFD {
        SOCKET_FD fd = INVALID_FD;
        KMEvent events = 0;
    };

private:
    typedef std::vector<PollFD> PollFdVector;
    NotifierPtr     notifier_ { Notifier::createNotifier() };
    PollFdVector    poll_fds_;
    
    FdSet           read_fds_;
    FdSet           write_fds_;
    FdSet           except_fds_;
    SOCKET_FD       max_fd_;
};

SelectPoll::SelectPoll()
    : max_fd_(0)
{

}

SelectPoll::~SelectPoll()
{
    poll_fds_.clear();
}

bool SelectPoll::init()
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

Result SelectPoll::registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb)
{
    if (fd < 0) {
        return Result::INVALID_PARAM;
    }
    KM_INFOTRACE("SelectPoll::registerFd, fd=" << fd);
    auto *poll_item = getPollItem(fd, true);
    if (!poll_item) {
        KM_ERRTRACE("SelectPoll::registerFd no poll item, fd=" << fd << ", sz=" << getPollItemSize());
        return Result::BUFFER_TOO_SMALL;
    }
    if (INVALID_FD == poll_item->fd || -1 == poll_item->idx) {
        PollFD pfd{fd, events};
        poll_fds_.emplace_back(pfd);
        poll_item->idx = static_cast<int>(poll_fds_.size() - 1);
    }
    poll_item->fd = fd;
    poll_item->events = events;
    poll_item->cb = std::move(cb);
    updateFdSet(fd, events);
    return Result::OK;
}

Result SelectPoll::unregisterFd(SOCKET_FD fd)
{
    auto sz = getPollItemSize();
    KM_INFOTRACE("SelectPoll::unregisterFd, fd="<<fd<<", sz="<<sz);
    auto *poll_item = getPollItem(fd);
    if (!poll_item) {
        KM_ERRTRACE("SelectPoll::unregisterFd failed, fd=" << fd);
        return Result::INVALID_PARAM;
    }
    updateFdSet(fd, 0);
    int idx = poll_item->idx;
    clearPollItem(fd);
    int last_idx = static_cast<int>(poll_fds_.size() - 1);
    if (idx > last_idx || idx == -1) {
        return Result::OK;
    }
    if (idx != last_idx) {
        std::iter_swap(poll_fds_.begin() + idx, poll_fds_.end() - 1);
        auto *pi = getPollItem(poll_fds_[idx].fd);
        if (pi) {
            pi->idx = idx;
        }
    }
    poll_fds_.pop_back();
    return Result::OK;
}

Result SelectPoll::updateFd(SOCKET_FD fd, KMEvent events)
{
    auto *poll_item = getPollItem(fd);
    if (!poll_item || INVALID_FD == poll_item->fd) {
        return Result::INVALID_PARAM;
    }
    if (poll_item->fd != fd) {
        KM_WARNTRACE("SelectPoll::updateFd, failed, fd="<<fd<<", item_fd="<<poll_item->fd);
        return Result::INVALID_PARAM;
    }
    int idx = poll_item->idx;
    if (idx < 0 || idx >= static_cast<int>(poll_fds_.size())) {
        KM_WARNTRACE("SelectPoll::updateFd, failed, index="<<idx);
        return Result::INVALID_STATE;
    }
    if (poll_fds_[idx].fd != fd) {
        KM_WARNTRACE("SelectPoll::updateFd, failed, fd="<<fd<<", pfds_fd="<<poll_fds_[idx].fd);
        return Result::INVALID_PARAM;
    }
    poll_fds_[idx].events = events;
    poll_item->events = events;
    updateFdSet(fd, events);
    return Result::OK;
}

void SelectPoll::updateFdSet(SOCKET_FD fd, KMEvent events)
{
    if (events != 0) {
        if (events & kEventRead) {
            read_fds_.set(fd);
        } else {
            read_fds_.clear(fd);
        }
        if (events & kEventWrite) {
            write_fds_.set(fd);
        } else {
            write_fds_.clear(fd);
        }
        if (events & kEventError) {
            except_fds_.set(fd);
        } else {
            except_fds_.clear(fd);
        }
        if (fd > max_fd_) {
            max_fd_ = fd;
        }
    } else {
        read_fds_.clear(fd);
        write_fds_.clear(fd);
        except_fds_.clear(fd);
        if (max_fd_ == fd) {
            auto it = std::max_element(poll_fds_.begin(), poll_fds_.end(), [](const PollFD& pf1, const PollFD& pf2) {
                return pf1.fd < pf2.fd;
            });
            max_fd_ = (it != poll_fds_.end()) ? it->fd : 0;
        }
    }
}

Result SelectPoll::wait(uint32_t wait_ms)
{
    FdSet readfds(read_fds_), writefds(write_fds_), exceptfds(except_fds_);
    struct timeval tval { 0, 0 };
    if (wait_ms != static_cast<uint32_t>(-1)) {
        tval.tv_sec = wait_ms / 1000;
        tval.tv_usec = (wait_ms - tval.tv_sec * 1000) * 1000;
    }
    int nready = ::select(static_cast<int>(max_fd_ + 1), readfds, writefds, exceptfds, wait_ms == static_cast<uint32_t>(-1) ? nullptr : &tval);
    if (nready <= 0) {
        return Result::OK;
    }
    // Copy poll_fds_ since callbacks may unregister fds during iteration
    const PollFdVector poll_fds = poll_fds_;
    int fds_count = static_cast<int>(poll_fds.size());
    for (int i = 0; i < fds_count && nready > 0; ++i) {
        KMEvent revents = 0;
        SOCKET_FD fd = poll_fds[i].fd;
        if (readfds.is_set(fd)) {
            revents |= kEventRead;
            --nready;
        }
        if (nready > 0 && writefds.is_set(fd)) {
            revents |= kEventWrite;
            --nready;
        }
        if (nready > 0 && exceptfds.is_set(fd)) {
            revents |= kEventError;
            --nready;
        }
        auto *poll_item = getPollItem(fd);
        if (poll_item) {
            revents &= poll_item->events;
            if (revents) {
                auto& cb = poll_item->cb;
                if (cb) cb(fd, revents, nullptr, 0);
            }
        }
    }
    return Result::OK;
}

void SelectPoll::notify()
{
    notifier_->notify();
}

IOPoll* createSelectPoll() {
    return new SelectPoll();
}

KEV_NS_END
