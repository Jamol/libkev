/* Copyright (c) 2014, Fengping Bao <jamol@live.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef __IOPoll_H__
#define __IOPoll_H__

#include "kevdefs.h"
#include "kevops.h"
#include "utils/utils.h"
#include "utils/kmtrace.h"
#include "utils/kmilist.h"

#ifdef KUMA_OS_WIN
# include <Ws2tcpip.h>
# include <windows.h>
# include <time.h>
#elif defined(KUMA_OS_LINUX) || defined(KUMA_OS_OHOS)
# include <string.h>
# include <pthread.h>
# include <unistd.h>
# include <fcntl.h>
# include <sys/types.h>
# include <sys/stat.h>
# include <sys/time.h>
# include <sys/socket.h>
# include <netdb.h>
# include <signal.h>
# include <arpa/inet.h>
# include <netinet/tcp.h>
# include <netinet/in.h>

#elif defined(KUMA_OS_MAC)
# include <string.h>
# include <pthread.h>
# include <unistd.h>

# include <sys/types.h>
# include <sys/socket.h>
# include <sys/ioctl.h>
# include <sys/fcntl.h>
# include <sys/time.h>
# include <sys/uio.h>
# include <netinet/tcp.h>
# include <netinet/in.h>
# include <arpa/inet.h>
# include <netdb.h>
# include <ifaddrs.h>

#else
# error "UNSUPPORTED OS"
#endif

#include <stdarg.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <vector>
#include <unordered_map>
#include <mutex>

//#define IOPOLL_ITEMS_USE_MAP

KEV_NS_BEGIN

class IOPoll
{
public:
    virtual ~IOPoll() {}
    
    virtual bool init() = 0;
    virtual Result registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb) = 0;
    virtual Result unregisterFd(SOCKET_FD fd) = 0;
    virtual Result updateFd(SOCKET_FD fd, KMEvent events) = 0;
    virtual Result wait(uint32_t wait_time_ms) = 0;
    virtual void notify() = 0;
    virtual PollType getType() const = 0;
    virtual bool isLevelTriggered() const = 0;

    virtual Result registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb, IOPollData *&data)
    {
        data = nullptr;
        return registerFd(fd, events, std::move(cb));
    }
    virtual Result updateFd(SOCKET_FD fd, KMEvent events, IOPollData *data)
    {
        return updateFd(fd, events);
    }
    virtual Result unregisterFd(SOCKET_FD fd, IOPollData *&data)
    {
        data = nullptr;
        return unregisterFd(fd);
    }

    virtual Result submitOp(SOCKET_FD fd, const Op &op)
    {
        return Result::NOT_SUPPORTED;
    }
};

struct IOPollItem
{
    void reset() {
        fd = INVALID_FD;
        idx = -1;
        events = 0;
        revents = 0;
        cb = nullptr;
    }
    SOCKET_FD fd { INVALID_FD };
    int idx { -1 };
    KMEvent events { 0 }; // kuma events registered
    KMEvent revents { 0 }; // kuma events received
    IOCallback cb;
};

template<typename ItemType>
class IOPollItemManager
{
public:
#ifdef IOPOLL_ITEMS_USE_MAP
    using PollItems = std::unordered_map<SOCKET_FD, ItemType>;
    ItemType* getPollItem(SOCKET_FD fd, bool create_if_not_exist = false) {
        auto it = poll_items_.find(fd);
        if (it == poll_items_.end()) {
            if (!create_if_not_exist) {
                return nullptr;
            }
            auto res = poll_items_.emplace(fd, ItemType{});
            if (!res.second) {
                return nullptr;
            }
            it = res.first;
        }
        return &it->second;
    }
    void clearPollItem(SOCKET_FD fd) {
        poll_items_.erase(fd);
    }
#else
    using PollItems = std::vector<ItemType>;
    bool resizePollItem(SOCKET_FD fd) {
        auto count = poll_items_.size();
        if (fd >= count) {
            if(fd > count + 1024) {
                poll_items_.resize(fd+1);
            } else {
                poll_items_.resize(count + 1024);
            }
            if (fd >= static_cast<SOCKET_FD>(poll_items_.size())) {
                return false;
            }
        }
        return true;
    }
    ItemType* getPollItem(SOCKET_FD fd, bool create_if_not_exist = false) {
        if (create_if_not_exist) {
            resizePollItem(fd);
        }
        if (fd < 0 || fd >= static_cast<SOCKET_FD>(poll_items_.size())) {
            return nullptr;
        }
        return &poll_items_[fd];
    }
    void clearPollItem(SOCKET_FD fd) {
        auto max_fd = static_cast<SOCKET_FD>(poll_items_.size() - 1);
        if (fd < 0 || fd > max_fd) {
            KTLOGW("IOPoll::clearPollItem, failed, fd=" << fd << ", max_fd=" << max_fd);
            return;
        }
        if(fd < max_fd) {
            poll_items_[fd].reset();
        } else if (fd == max_fd) {
            poll_items_.pop_back();
        }
    }
#endif

    size_t getPollItemSize() const {
        return poll_items_.size();
    }

private:
    PollItems  poll_items_;
};

struct IOPollData : public inode<IOPollData>
{
    SOCKET_FD fd { INVALID_FD };
    KMEvent events { 0 };
    IOCallback cb;
    std::recursive_mutex rmtx;

    void reset() {
        std::lock_guard<std::recursive_mutex> g(rmtx);
        fd = INVALID_FD;
        events = 0;
        cb = {};
    }
};

template<typename PollDataType>
class IoPollDataManager
{
public:
    virtual ~IoPollDataManager() {
        while (!pending_list_.empty()) {
            auto *data = &pending_list_.front();
            pending_list_.pop_front();
            delete data;
        }
        while (!free_list_.empty()) {
            auto *data = &free_list_.front();
            free_list_.pop_front();
            delete data;
        }
    }

    PollDataType* createPollData() {
        return new PollDataType();
    }

    PollDataType* getFreePollData() {
        if (!free_list_.empty()) {
            auto *data = &free_list_.front();
            free_list_.pop_front();
            return data;
        }
        return nullptr;
    }

    PollDataType* allocPollData() {
        auto *data = getFreePollData();
        return data ? data : new PollDataType();
    }

    void freePollData(PollDataType* data) {
        if (data) {
            pending_list_.push_back(data);
        }
    }

    void processPendingPollData() {
#if 1
        free_list_.splice(pending_list_);
#else
        while (!pending_list_.empty()) {
            auto *data = &pending_list_.front();
            pending_list_.pop_front();
            delete data;
        }
#endif
    }

private:
    ilist<inode<IOPollData>> free_list_;
    ilist<inode<IOPollData>> pending_list_;
};

KEV_NS_END

#endif
