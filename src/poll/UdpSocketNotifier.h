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

#ifndef __UdpSocketNotifier_H__
#define __UdpSocketNotifier_H__

#include "utils/utils.h"
#include "utils/defer.h"
#include "utils/skutils.h"
#include "Notifier.h"

KEV_NS_BEGIN

class UdpSocketNotifier : public Notifier
{
public:
    enum {
        READ_FD = 0,
        WRITE_FD
    };
    ~UdpSocketNotifier() {
        cleanup();
    }
    bool init() override {
        cleanup();

        fds_[WRITE_FD] = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (INVALID_FD == fds_[WRITE_FD]) {
            cleanup();
            return false;
        }
        fds_[READ_FD] = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (INVALID_FD == fds_[READ_FD]) {
            cleanup();
            return false;
        }

        sin_.sin_family = AF_INET;
        sin_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        sin_.sin_port = 0;
        if (::bind(fds_[WRITE_FD], (struct sockaddr*)&sin_, sizeof(sin_)) < 0) {
            cleanup();
            return false;
        }
        sin_.sin_port = 0;
        if (::bind(fds_[READ_FD], (struct sockaddr*)&sin_, sizeof(sin_)) < 0) {
            cleanup();
            return false;
        }
        socklen_t sl = sizeof(sin_);
        if (::getsockname(fds_[READ_FD], (struct sockaddr*)&sin_, &sl) < 0) {
            cleanup();
            return false;
        }

        set_nonblocking(fds_[READ_FD]);
        set_nonblocking(fds_[WRITE_FD]);
        return true;
    }
    bool ready() override {
        return fds_[READ_FD] != INVALID_FD && fds_[WRITE_FD] != INVALID_FD;
    }
    void notify() {
        char c = 1;
        SKUtils::sendto(fds_[WRITE_FD], &c, sizeof(c), 0, (struct sockaddr*)&sin_, sizeof(sin_));
    }
    
    SOCKET_FD getReadFD() override {
        return fds_[READ_FD];
    }
    
    Result onEvent(KMEvent ev) override {
        char buf[16];
        ssize_t ret = 0;
        do {
            ret = SKUtils::recv(fds_[READ_FD], buf, sizeof(buf), 0);
        } while(ret > 0);
        return Result::OK;
    }
private:
    void cleanup() {
        if (fds_[READ_FD] != INVALID_FD) {
            SKUtils::close(fds_[READ_FD]);
            fds_[READ_FD] = INVALID_FD;
        }
        if (fds_[WRITE_FD] != INVALID_FD) {
            SKUtils::close(fds_[WRITE_FD]);
            fds_[WRITE_FD] = INVALID_FD;
        }
    }

    SOCKET_FD fds_[2] { INVALID_FD, INVALID_FD };
    struct sockaddr_in sin_{0};
    
};

KEV_NS_END

#endif
