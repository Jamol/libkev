/* Copyright (c) 2023-2025, Fengping Bao <jamol@live.com>
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

#ifdef KEV_HAS_RUNLOOP

#include "IOPoll.h"
#include "utils/kmtrace.h"

#include <CoreFoundation/CoreFoundation.h>

KEV_NS_BEGIN

static void KevSocketCallBack(CFSocketRef s, 
                              CFSocketCallBackType type,
                              CFDataRef address,
                              const void *data,
                              void *info);

struct SockItem
{
    void reset() {
        fd = INVALID_FD;
        sock = nil;
        source = nil;
        events = 0;
        revents = 0;
        cb = nullptr;
    }
    SOCKET_FD fd { INVALID_FD };
    CFSocketRef sock {nil};
    CFRunLoopSourceRef source {nil};
    KMEvent events { 0 }; // kuma events registered
    KMEvent revents { 0 }; // kuma events received
    IOCallback cb;
};

class RunLoopMac : public IOPoll, public IOPollItem<SockItem>
{
public:
    ~RunLoopMac();
    bool init() override;
    Result registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb) override;
    Result unregisterFd(SOCKET_FD fd) override;
    Result updateFd(SOCKET_FD fd, KMEvent events) override;
    Result wait(uint32_t wait_ms) override;
    void notify() override;
    PollType getType() const override { return PollType::RUNLOOP; }
    bool isLevelTriggered() const override { return true; }
    
    void onSocketCallBack(CFSocketRef s, CFSocketCallBackType type);

private:
    CFSocketCallBackType get_events(KMEvent kuma_events) const;
    KMEvent get_kuma_events(CFSocketCallBackType events) const;

private:
    CFRunLoopRef loopref_ = nil;
    CFRunLoopSourceRef notifier_ = nil;
    //CFRunLoopObserverRef observer_ = nil;
};

RunLoopMac::~RunLoopMac()
{
    if (notifier_) {
        if (loopref_) {
            CFRunLoopRemoveSource(loopref_, notifier_, kCFRunLoopDefaultMode);
        }
        CFRelease(notifier_);
        notifier_ = nil;
    }
    if (loopref_) {
        CFRunLoopWakeUp(loopref_);
        CFRunLoopStop(loopref_);
        CFRelease(loopref_);
        loopref_ = nil;
    }
}

bool RunLoopMac::init()
{
    auto loopref = CFRunLoopGetCurrent();
    if (loopref == loopref_) {
        return true;
    }
    std::swap(loopref, loopref_);
    CFRetain(loopref_);
    if (loopref) {
        if (notifier_) {
            CFRunLoopRemoveSource(loopref, notifier_, kCFRunLoopDefaultMode);
        }
        CFRelease(loopref);
    }

    if (!notifier_) {
        CFRunLoopSourceContext context;
        bzero(&context, sizeof(context));
        notifier_ = CFRunLoopSourceCreate(kCFAllocatorDefault, 0, &context);
        if (notifier_ == nil) {
            CFRelease(loopref_);
            loopref_ = nil;
            return false;
        }
    }
    CFRunLoopAddSource(loopref_, notifier_, kCFRunLoopDefaultMode);
    
    return true;
}

CFSocketCallBackType RunLoopMac::get_events(KMEvent kuma_events) const
{
    CFSocketCallBackType ev = kCFSocketNoCallBack;
    if(kuma_events & kEventRead) {
        ev |= kCFSocketReadCallBack;
    }
    if(kuma_events & kEventWrite) {
        ev |= kCFSocketWriteCallBack;
    }
    if(kuma_events & kEventError) {
        ev |= kCFSocketReadCallBack;
    }
    return ev;
}

KMEvent RunLoopMac::get_kuma_events(CFSocketCallBackType events) const
{
    KMEvent ev = 0;
    if(events & kCFSocketReadCallBack) {
        ev |= kEventRead;
    }
    if(events & kCFSocketWriteCallBack) {
        ev |= kEventWrite;
    }
    return ev;
}

Result RunLoopMac::registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb)
{
    if (fd < 0) {
        return Result::INVALID_PARAM;
    }
    
    auto cb_types = kCFSocketReadCallBack | kCFSocketWriteCallBack;
    CFSocketContext context = {0, this, nil, nil, nil};
    auto sock = CFSocketCreateWithNative(nil, fd, cb_types,
                                    KevSocketCallBack, &context);
    if (sock == nil) {
        KM_ERRTRACE("RunLoop::registerFd error, fd=" << fd << ", ev=" << events);
        return Result::FAILED;
    }
    auto *poll_item = getPollItem(fd, true);
    if (!poll_item) {
        KM_ERRTRACE("RunLoop::registerFd no poll item, fd=" << fd << ", sz=" << getPollItemSize());
        CFSocketInvalidate(sock);
        CFRelease(sock);
        return Result::BUFFER_TOO_SMALL;
    }
    poll_item->fd = fd;
    poll_item->events = events;
    poll_item->cb = std::move(cb);
    auto flags = CFSocketGetSocketFlags (sock);
    flags |= kCFSocketAutomaticallyReenableReadCallBack;
    flags &= ~kCFSocketCloseOnInvalidate;
    CFSocketSetSocketFlags(sock, flags);
    CFSocketCallBackType disabled_types = kCFSocketNoCallBack;
    if (!(events & kEventRead)) {
        disabled_types |= kCFSocketReadCallBack;
    }
    if (!(events & kEventWrite)) {
        disabled_types |= kCFSocketWriteCallBack;
    }
    CFSocketDisableCallBacks(sock, disabled_types);
    auto sockSource = CFSocketCreateRunLoopSource(NULL, sock, 0);
    poll_item->sock = sock;
    poll_item->source = sockSource;
    CFRunLoopAddSource(loopref_, sockSource, kCFRunLoopDefaultMode);

    KM_INFOTRACE("RunLoop::registerFd, fd=" << fd << ", ev=" << events);

    return Result::OK;
}

Result RunLoopMac::unregisterFd(SOCKET_FD fd)
{
    auto sz = getPollItemSize();
    KM_INFOTRACE("RunLoop::unregisterFd, fd="<<fd<<", sz="<<sz);
    auto *poll_item = getPollItem(fd);
    if (!poll_item) {
        KM_ERRTRACE("RunLoop::unregisterFd failed, fd=" << fd);
        return Result::INVALID_PARAM;
    }
    auto sock = poll_item->sock;
    auto sockSource = poll_item->source;
    poll_item->sock = nil;
    poll_item->source = nil;
    if (sockSource) {
        CFRunLoopRemoveSource(loopref_, sockSource, kCFRunLoopDefaultMode);
        CFRelease(sockSource);
    }
    //@synchronized (sock) {
    if (sock) {    
        CFSocketInvalidate(sock);
        CFRelease(sock);
    }
    //}
    clearPollItem(fd);
    return Result::OK;
}

Result RunLoopMac::updateFd(SOCKET_FD fd, KMEvent events)
{
    auto *poll_item = getPollItem(fd);
    if (!poll_item || INVALID_FD == poll_item->fd || poll_item->sock == nil) {
        KM_ERRTRACE("RunLoop::updateFd failed, fd=" << fd);
        return Result::INVALID_PARAM;
    }
    CFSocketCallBackType disabled_types = kCFSocketNoCallBack;
    CFSocketCallBackType enabled_types = kCFSocketNoCallBack;
    if ((events & kEventRead) && !(poll_item->events & kEventRead)) {
        enabled_types |= kCFSocketReadCallBack;
    } else if (!(events & kEventRead) && (poll_item->events & kEventRead)) {
        disabled_types |= kCFSocketReadCallBack;
    }
    if ((events & kEventWrite) && !(poll_item->events & kEventWrite)) {
        enabled_types |= kCFSocketWriteCallBack;
    } else if (!(events & kEventWrite) && (poll_item->events & kEventWrite)) {
        disabled_types |= kCFSocketWriteCallBack;
    }
    if (disabled_types != kCFSocketNoCallBack) {
        CFSocketDisableCallBacks(poll_item->sock, disabled_types);
    }
    if (enabled_types != kCFSocketNoCallBack) {
        CFSocketEnableCallBacks(poll_item->sock, enabled_types);
    }
    poll_item->events = events;
    return Result::OK;
}


Result RunLoopMac::wait(uint32_t wait_ms)
{
    CFTimeInterval seconds = wait_ms / 1000.0;
    auto result = CFRunLoopRunInMode(kCFRunLoopDefaultMode, seconds, YES);
    if (result == kCFRunLoopRunStopped) {
        ;
    }
    return Result::OK;
}

void RunLoopMac::notify()
{
    if (notifier_) {
        CFRunLoopSourceSignal(notifier_);
    }
    if (loopref_) {
        CFRunLoopWakeUp(loopref_);
    }
}

void RunLoopMac::onSocketCallBack(CFSocketRef s, CFSocketCallBackType type)
{
    SOCKET_FD fd = CFSocketGetNative(s);
    auto *poll_item = getPollItem(fd);
    if(poll_item) {
        auto revents = get_kuma_events(type);
        revents &= poll_item->events;
        if (revents) {
            auto &cb = poll_item->cb;
            if(cb) cb(fd, revents, nullptr, 0);
        }
    }
}

IOPoll* createRunLoop()
{
    return new RunLoopMac();
}


static void KevSocketCallBack(CFSocketRef s, 
                              CFSocketCallBackType type,
                              CFDataRef address,
                              const void *data,
                              void *info)
{
    RunLoopMac* runLoop = reinterpret_cast<RunLoopMac*>(info);
    runLoop->onSocketCallBack(s, type);
}

KEV_NS_END

#endif // KEV_HAS_RUNLOOP
