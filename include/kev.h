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

#ifndef __KUMAAPI_H__
#define __KUMAAPI_H__

#include "kmdefs.h"
#include "evdefs.h"

#include <stdint.h>

KUMA_NS_BEGIN


class KUMA_API EventLoop
{
public:
    using Task = std::function<void(void)>;
    
    class Token {
    public:
        Token();
        Token(Token &&other);
        Token(const Token &other) = delete;
        ~Token();
        
        Token& operator=(Token &&other);
        Token& operator=(const Token &other) = delete;
        
        void reset();
        
        class Impl;
        Impl* pimpl();
        
    private:
        Impl* pimpl_;
    };
    
public:
    EventLoop(PollType poll_type = PollType::NONE);
    ~EventLoop();
    
public:
    bool init();
    
    /* NOTE: cb must be valid untill unregisterFd called
     * this API is thread-safe
     */
    KMError registerFd(SOCKET_FD fd, uint32_t events, IOCallback cb);
    /*
     * this API is thread-safe
     */
    KMError updateFd(SOCKET_FD fd, uint32_t events);
    /*
     * this API is thread-safe
     */
    KMError unregisterFd(SOCKET_FD fd, bool close_fd);
    
    PollType getPollType() const;
    bool isPollLT() const; // level trigger

public:
    bool inSameThread() const;
    
    /* create a token, it can be used to cancel the tasks that are scheduled with it
     * if caller can guarantee the resources used by tasks are valid when task running,
     * then token is no need, otherwise the caller should cancel the tasks queued in loop
     * before the resources are unavailable
     */
    Token createToken();
    
    /* run the task in loop thread and wait untill task is executed.
     * the task will be executed at once if called on loop thread
     * token is always unnecessary for sync task
     *
     * @param f the task to be executed. it will always be executed when call success
     *
     * @return return the result of f()
     */
    template<typename Callable>
    auto invoke(Callable &&f)
    {
        if (inSameThread()) {
            return f();
        }
        KMError err;
        return Invoker<decltype(f()), Callable>::sync(std::forward<Callable>(f), this, err);
    }

    /* run the task in loop thread and wait untill task is executed.
     * the task will be executed at once if called on loop thread
     * token is always unnecessary for sync task
     *
     * @param f the task to be executed. it will always be executed when call success
     * @param err when f is executed, err is KMError::NOERR, otherwise is KMError
     *
     * @return return the result of f()
     */
    template<typename Callable>
    auto invoke(Callable &&f, KMError &err)
    {
        if (inSameThread()) {
            return f();
        }
        return Invoker<decltype(f()), Callable>::sync(std::forward<Callable>(f), this, err);
    }

    /* run the task in loop thread and wait untill task is executed.
     * the task will be executed at once if called on loop thread
     * token is always unnecessary for sync task
     *
     * @param task the task to be executed. it will always be executed when call success
     */
    KMError sync(Task task);
    
    /* run the task in loop thread.
     * the task will be executed at once if called on loop thread
     *
     * @param task the task to be executed. it will always be executed when call success
     * @param token to be used to cancel the task. If token is null, the caller should
     *              make sure the resources referenced by task are valid when task running
     */
    KMError async(Task task, Token *token=nullptr);
    
    /* run the task in loop thread at next time.
     *
     * @param task the task to be executed. it will always be executed when call success
     * @param token to be used to cancel the task. If token is null, the caller should
     *              make sure the resources referenced by task are valid when task running
     */
    KMError post(Task task, Token *token=nullptr);
    
    /* cancel the tasks that are scheduled with token. you cannot cancel the task that is in running,
     * but will wait untill the task completion
     *
     * @param token token of the tasks
     * this API is thread-safe
     */
    void cancel(Token *token);
    
    void loopOnce(uint32_t max_wait_ms);
    void loop(uint32_t max_wait_ms = -1);
    void stop();
    
    class Impl;
    Impl* pimpl();

protected:
    template <typename ReturnType, typename Callable>
    struct Invoker
    {
        template<typename LoopType>
        static ReturnType sync(Callable &&f, LoopType *loop, KMError &err) {
            ReturnType retval;
            auto task_sync = [&] { retval = f(); };
            err = loop->sync(std::move(task_sync));
            return retval;
        }
    };

    template <typename Callable>
    struct Invoker<void, Callable>
    {
        template<typename LoopType>
        static void sync(Callable &&f, LoopType *loop, KMError &err) {
            err = loop->sync(std::forward<Callable>(f));
        }
    };

private:
    Impl* pimpl_;
};

class KUMA_API Timer
{
public:
    using TimerCallback = std::function<void(void)>;
    
    Timer(EventLoop *loop);
    ~Timer();
    
    /**
     * Schedule the timer. This API is thread-safe
     */
    bool schedule(uint32_t delay_ms, TimerMode mode, TimerCallback cb);
    
    /**
     * Cancel the scheduled timer. This API is thread-safe
     */
    void cancel();
    
    class Impl;
    Impl* pimpl();
    
private:
    Impl* pimpl_;
};

using TraceFunc = std::function<void(int, const char*)>; // (level, msg)

KUMA_API void init(const char *path = nullptr);
KUMA_API void fini();
KUMA_API void setTraceFunc(TraceFunc func);

KUMA_NS_END

#endif
