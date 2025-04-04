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

#ifndef __PipeNotifier_H__
#define __PipeNotifier_H__

#include "utils/utils.h"
#include "Notifier.h"

KEV_NS_BEGIN

class PipeNotifier : public Notifier
{
public:
    enum {
        READ_FD = 0,
        WRITE_FD
    };
    ~PipeNotifier() {
        cleanup();
    }
    bool init() override {
        cleanup();
        if(::pipe(fds_) != 0) {
            cleanup();
            return false;
        } else {
            ::fcntl(fds_[READ_FD], F_SETFL, O_RDONLY | O_NONBLOCK);
            ::fcntl(fds_[WRITE_FD], F_SETFL, O_WRONLY | O_NONBLOCK);
#ifdef FD_CLOEXEC
            ::fcntl(fds_[READ_FD], F_SETFD, ::fcntl(fds_[READ_FD], F_GETFD) | FD_CLOEXEC);
            ::fcntl(fds_[WRITE_FD], F_SETFD, ::fcntl(fds_[WRITE_FD], F_GETFD) | FD_CLOEXEC);
#endif
#ifdef KUMA_OS_MAC
            ::fcntl(fds_[READ_FD], F_SETNOSIGPIPE, 1);
            ::fcntl(fds_[WRITE_FD], F_SETNOSIGPIPE, 1);
#endif
            return true;
        }
    }
    bool ready() override {
        return fds_[READ_FD] != INVALID_FD && fds_[WRITE_FD] != INVALID_FD;
    }
    void notify() override {
        if(fds_[WRITE_FD] != INVALID_FD) {
            ssize_t ret = 0;
            do {
                char c = 1;
                ret = ::write(fds_[WRITE_FD], &c, sizeof(c));
            } while(ret < 0 && errno == EINTR);
        }
    }
    
    SOCKET_FD getReadFD() override {
        return fds_[READ_FD];
    }
    
    Result onEvent(KMEvent ev) override {
        char buf[64];
        ssize_t ret = 0;
        do {
            ret = ::read(fds_[READ_FD], buf, sizeof(buf));
        } while(ret == sizeof(buf) || (ret < 0 && errno == EINTR));
        return Result::OK;
    }
private:
    void cleanup() {
        if (fds_[READ_FD] != INVALID_FD) {
            ::close(fds_[READ_FD]);
            fds_[READ_FD] = INVALID_FD;
        }
        if (fds_[WRITE_FD] != INVALID_FD) {
            ::close(fds_[WRITE_FD]);
            fds_[WRITE_FD] = INVALID_FD;
        }
    }

    SOCKET_FD fds_[2] { INVALID_FD, INVALID_FD };
};

KEV_NS_END

#endif
