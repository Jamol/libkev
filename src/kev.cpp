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

#include "kev.h"
#include "EventLoopImpl.h"
#include "TimerManager.h"

KUMA_NS_BEGIN

template <typename Impl>
struct ImplHelper {
    using ImplPtr = std::shared_ptr<Impl>;
    Impl impl;
    ImplPtr ptr;

    template <typename... Args>
    ImplHelper(Args&... args)
        : impl(args...)
    {
        ptr.reset(&impl, [](Impl* p) {
            auto h = reinterpret_cast<ImplHelper*>(p);
            delete h;
        });
    }

    template <typename... Args>
    ImplHelper(Args&&... args)
        : impl(std::forward<Args...>(args)...)
    {
        ptr.reset(&impl, [](Impl* p) {
            auto h = reinterpret_cast<ImplHelper*>(p);
            delete h;
        });
    }
    
    template <typename... Args>
    static Impl* create(Args&... args)
    {
        auto *ih = new ImplHelper(args...);
        return &ih->impl;
    }
    
    template <typename... Args>
    static Impl* create(Args&&... args)
    {
        auto *ih = new ImplHelper(std::forward<Args...>(args)...);
        return &ih->impl;
    }
    
    static void destroy(Impl *pimpl)
    {
        if (pimpl) {
            auto *ih = reinterpret_cast<ImplHelper*>(pimpl);
            ih->ptr.reset();
        }
    }
    
    static ImplPtr implPtr(Impl *pimpl)
    {
        if (pimpl) {
            auto h = reinterpret_cast<ImplHelper*>(pimpl);
            return h->ptr;
        }
        return ImplPtr();
    }

private:
    ~ImplHelper() = default;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// class EventLoop
using EventLoopHelper = ImplHelper<EventLoop::Impl>;
EventLoop::EventLoop(PollType poll_type)
: pimpl_(EventLoopHelper::create(std::move(poll_type)))
{
    
}

EventLoop::~EventLoop()
{
    EventLoopHelper::destroy(pimpl_);
}

bool EventLoop::init()
{
    return pimpl_->init();
}

EventLoop::Token EventLoop::createToken()
{
    Token t;
    t.pimpl()->eventLoop(EventLoopHelper::implPtr(pimpl()));
    return t;
}

PollType EventLoop::getPollType() const
{
    return pimpl_->getPollType();
}

bool EventLoop::isPollLT() const
{
    return  pimpl_->isPollLT();
}

KMError EventLoop::registerFd(SOCKET_FD fd, uint32_t events, IOCallback cb)
{
    return pimpl_->registerFd(fd, events, std::move(cb));
}

KMError EventLoop::updateFd(SOCKET_FD fd, uint32_t events)
{
    return pimpl_->updateFd(fd, events);
}

KMError EventLoop::unregisterFd(SOCKET_FD fd, bool close_fd)
{
    return pimpl_->unregisterFd(fd, close_fd);
}

void EventLoop::loopOnce(uint32_t max_wait_ms)
{
    pimpl_->loopOnce(max_wait_ms);
}

void EventLoop::loop(uint32_t max_wait_ms)
{
    pimpl_->loop(max_wait_ms);
}

void EventLoop::stop()
{
    pimpl_->stop();
}

EventLoop::Impl* EventLoop::pimpl()
{
    return pimpl_;
}

bool EventLoop::inSameThread() const
{
    return pimpl_->inSameThread();
}

KMError EventLoop::sync(Task task)
{
    return pimpl_->sync(std::move(task));
}

KMError EventLoop::async(Task task, Token *token)
{
    return pimpl_->async(std::move(task), token?token->pimpl():nullptr);
}

KMError EventLoop::post(Task task, Token *token)
{
    return pimpl_->post(std::move(task), token?token->pimpl():nullptr);
}

void EventLoop::wakeup()
{
    pimpl_->wakeup();
}

void EventLoop::cancel(Token *token)
{
    if (token) {
        pimpl_->removeTask(token->pimpl());
    }
}

EventLoop::Token::Token()
: pimpl_(new Impl())
{
    
}

EventLoop::Token::Token(Token &&other)
: pimpl_(other.pimpl_)
{
    other.pimpl_ = nullptr;
}

EventLoop::Token::~Token()
{
    delete pimpl_;
}

EventLoop::Token& EventLoop::Token::operator=(Token &&other)
{
    if (this != &other) {
        if (pimpl_) {
            delete pimpl_;
        }
        pimpl_ = other.pimpl_;
        other.pimpl_ = nullptr;
    }
    return *this;
}

void EventLoop::Token::reset()
{
    if (pimpl_) {
        pimpl_->reset();
    }
}

EventLoop::Token::Impl* EventLoop::Token::pimpl()
{
    return pimpl_;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////

Timer::Timer(EventLoop* loop)
: pimpl_(new Impl(loop->pimpl()->getTimerMgr()))
{
    
}

Timer::~Timer()
{
    delete pimpl_;
}

bool Timer::schedule(uint32_t delay_ms, TimerMode mode, TimerCallback cb)
{
    return pimpl_->schedule(delay_ms, mode, std::move(cb));
}

void Timer::cancel()
{
    pimpl_->cancel();
}

Timer::Impl* Timer::pimpl()
{
    return pimpl_;
}

KUMA_NS_END
