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

#ifndef __KEVAPI_H__
#define __KEVAPI_H__

#include "kevdefs.h"
#include "kmtypes.h"

#include <stdint.h>

KEV_NS_BEGIN


class KEV_API EventLoop
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

        friend class EventLoop;
    };
    
public:
    EventLoop(PollType poll_type = PollType::NONE);
    EventLoop(const EventLoop &) = delete;
    EventLoop(EventLoop &&other);
    ~EventLoop();
    
    EventLoop& operator=(const EventLoop &) = delete;
    EventLoop& operator=(EventLoop &&other);
    
public:
    bool init();
    
    /* NOTE: cb must be valid untill unregisterFd called
     * this API is thread-safe
     */
    Result registerFd(SOCKET_FD fd, uint32_t events, IOCallback cb);
    /*
     * this API is thread-safe
     */
    Result updateFd(SOCKET_FD fd, uint32_t events);
    /*
     * this API is thread-safe
     */
    Result unregisterFd(SOCKET_FD fd, bool close_fd);
    
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
    template<typename F>
    auto invoke(F &&f)
    {
        Result err;
        return invoke(std::forward<F>(f), err);
    }

    /* run the task in loop thread and wait untill task is executed.
     * the task will be executed at once if called on loop thread
     * token is always unnecessary for sync task
     *
     * @param f the task to be executed. it will always be executed when call success
     * @param err when f is executed, err is Result::OK, otherwise is Result
     *
     * @return return the result of f()
     */
    template<typename F, std::enable_if_t<!std::is_same<decltype(std::declval<F>()()), void>{}, int> = 0>
    auto invoke(F &&f, Result &err)
    {
        static_assert(!std::is_same<decltype(f()), void>{}, "is void");
        if (inSameThread()) {
            return f();
        }
        using ReturnType = decltype(f());
        ReturnType retval;
        auto task_sync = [&] { retval = f(); };
        err = sync(std::move(task_sync));
        return retval;
    }

    template<typename F, std::enable_if_t<std::is_same<decltype(std::declval<F>()()), void>{}, int> = 0>
    void invoke(F &&f, Result &err)
    {
        static_assert(std::is_same<decltype(f()), void>{}, "not void");
        if (inSameThread()) {
            return f();
        }
        err = sync(std::forward<F>(f));
    }

    /* run the task in loop thread and wait untill task is executed.
     * the task will be executed at once if called on loop thread
     * token is always unnecessary for sync task
     *
     * @param task the task to be executed. it will always be executed when call success
     */
    template<typename F, std::enable_if_t<!std::is_copy_constructible<F>{}, int> = 0>
    Result sync(F &&f)
    {
        lambda_wrapper<F> wf{std::forward<F>(f)};
        return sync(Task(std::move(wf)));
    }
    Result sync(Task task);
    
    /* run the task in loop thread.
     * the task will be executed at once if called on loop thread
     *
     * @param task the task to be executed. it will always be executed when call success
     * @param token to be used to cancel the task. If token is null, the caller should
     *              make sure the resources referenced by task are valid when task running
     * @param debugStr debug message of the f, e.g. file name and line where f is generated
     */
    template<typename F, std::enable_if_t<!std::is_copy_constructible<F>{}, int> = 0>
    Result async(F &&f, Token *token=nullptr, const char *debugStr=nullptr)
    {
        lambda_wrapper<F> wf{std::forward<F>(f)};
        return async(Task(std::move(wf)), token, debugStr);
    }
    Result async(Task task, Token *token=nullptr, const char *debugStr=nullptr);
    
    /* run the task in loop thread at next time.
     *
     * @param task the task to be executed. it will always be executed when call success
     * @param token to be used to cancel the task. If token is null, the caller should
     *              make sure the resources referenced by task are valid when task running
     */
    template<typename F, std::enable_if_t<!std::is_copy_constructible<F>{}, int> = 0>
    Result post(F &&f, Token *token=nullptr, const char *debugStr=nullptr)
    {
        lambda_wrapper<F> wf{std::forward<F>(f)};
        return post(Task(std::move(wf)), token, debugStr);
    }
    Result post(Task task, Token *token=nullptr, const char *debugStr=nullptr);
    
    template<typename F, std::enable_if_t<!std::is_copy_constructible<F>{}, int> = 0>
    Result postDelayed(uint32_t delay_ms, F &&f, Token *token=nullptr, const char *debugStr=nullptr)
    {
        lambda_wrapper<F> wf{std::forward<F>(f)};
        return postDelayed(delay_ms, Task(std::move(wf)), token, debugStr);
    }
    Result postDelayed(uint32_t delay_ms, Task task, Token *token=nullptr, const char *debugStr=nullptr);

    void wakeup();
    
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

private:
    Impl* pimpl_;
};

class KEV_API Timer
{
public:
    using TimerCallback = std::function<void(void)>;
    
    Timer(EventLoop *loop);
    Timer(const Timer &) = delete;
    Timer(Timer &&other);
    ~Timer();
    
    Timer& operator=(const Timer &) = delete;
    Timer& operator=(Timer &&other);
    
    /**
     * Schedule the timer. This API is thread-safe
     */
    template<typename F, std::enable_if_t<!std::is_copy_constructible<F>{}, int> = 0>
    bool schedule(uint32_t delay_ms, TimerMode mode, F &&f)
    {
        lambda_wrapper<F> wf{std::forward<F>(f)};
        return schedule(delay_ms, mode, TimerCallback(std::move(wf)));
    }
    // TimerCallback will be reset when:
    // 1. timer is cancelled
    // 2. timer is executed and TimerMode is ONE_SHOT
    // 3. the TimerManager in EventLoop is destroyed and timer is still in queue
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


// msg is null-terminated and msg_len doesn't include '\0'
using LogCallback = void(*)(int level, const char* msg, size_t msg_len);
KEV_API void setLogCallback(LogCallback cb);

KEV_NS_END

#endif
