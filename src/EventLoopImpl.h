/* Copyright (c) 2014-2022, Fengping Bao <jamol@live.com>
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

#ifndef __EventLoopImpl_H__
#define __EventLoopImpl_H__

#include "../include/kev.h"
#include "../include/kevdefs.h"
#include "utils/kmqueue.h"
#include "TimerManager.h"
#include "utils/kmobject.h"

#ifdef KUMA_OS_WIN
# include <Ws2tcpip.h>
#endif
#include <stdint.h>
#include <thread>
#include <list>
#include <atomic>
#include <mutex>

KEV_NS_BEGIN

class IOPoll;
using EventLoopToken = EventLoop::Token::Impl;
using EventLoopPtr = EventLoop::ImplPtr;
using EventLoopWeakPtr = std::weak_ptr<EventLoop::Impl>;

class TaskSlot
{
public:
    TaskSlot(EventLoop::Task &&t, std::string debug_str)
    : task(std::move(t)), debug_str(std::move(debug_str)) {}
    virtual ~TaskSlot() {}
    virtual void operator() ()
    {
        if (task) std::exchange(task, nullptr)();
    }
    void clearTask()
    {
        std::exchange(task, nullptr);
    }
    bool isActive() const
    {
        return bool(task);
    }
    EventLoop::Task task;
    std::string     debug_str;
};
using TaskSlotPtr = std::shared_ptr<TaskSlot>;
using TaskQueue = std::list<TaskSlotPtr>;

class TokenTaskSlot final : public TaskSlot
{
public:
    TokenTaskSlot(EventLoop::Task &&t, std::string debug_str)
    : TaskSlot(std::move(t), std::move(debug_str)) {}
    void operator() () override
    {
        auto expected = State::ACTIVE;
        std::lock_guard<std::mutex> g(mlock);
        if (state_.compare_exchange_strong(expected, State::RUNNING)) {
            TaskSlot::operator()();
            state_.exchange(State::INACTIVE);
        }
    }
    void cancel(bool inLoopThread)
    {
        auto expected = State::ACTIVE;
        if (state_.compare_exchange_strong(expected, State::INACTIVE)) {
            clearTask();
        } else if (expected == State::RUNNING) {
            if (!inLoopThread) {
                // wait for running complete
                mlock.lock();
                mlock.unlock();
            }
        }
    }

    enum class State
    {
        ACTIVE,
        RUNNING,
        INACTIVE
    };
    
    std::atomic<State>  state_{ State::ACTIVE };
    mutable std::mutex  mlock;
};
using TokenTaskSlotPtr = std::shared_ptr<TokenTaskSlot>;
using TokenTaskQueue = std::list<TokenTaskSlotPtr>;

class DelayedTaskSlot final : public TaskSlot
{
public:
    DelayedTaskSlot(EventLoop::Impl *loop, EventLoop::Task &&t, std::string debug_str);
    void cancel()
    {
        timer.cancel();
        clearTask();
    }

public:
    Timer::Impl     timer;
};
using DelayedTaskSlotPtr = std::shared_ptr<DelayedTaskSlot>;
using DelayedTaskQueue = std::list<DelayedTaskSlotPtr>;

enum class LoopActivity {
    EXIT,
};
using ObserverCallback = std::function<void(LoopActivity)>;
using ObserverToken = std::weak_ptr<DLQueue<ObserverCallback>::DLNode>;

/**
 * PendingObject is used to cache the IOCP context when destroying IocpSocket that 
 * has pending operations. It will be removed after all pending operatios are completed,
 * or the loop exited
 */
class PendingObject
{
public:
    virtual ~PendingObject() {}
    virtual bool isPending() const = 0;
    virtual void onLoopExit() = 0;

public:
    PendingObject* next_ = nullptr;
    PendingObject* prev_ = nullptr;
};

class EventLoop::Impl : public KMObject
{
public:
    Impl(PollType poll_type);
    Impl(const Impl &other) = delete;
    Impl(Impl &&other) = delete;
    ~Impl();

    Impl& operator= (const Impl &other) = delete;
    Impl& operator= (Impl &&other) = delete;

public:
    bool init();
    Result registerFd(SOCKET_FD fd, uint32_t events, IOCallback cb);
    Result updateFd(SOCKET_FD fd, uint32_t events);
    Result unregisterFd(SOCKET_FD fd, bool close_fd);
    Result submitOp(SOCKET_FD fd, const Op &op);
    TimerManager::Ptr getTimerMgr() const { return timer_mgr_; }
    
    PollType getPollType() const;
    bool isPollLT() const; // level trigger
    
    Result appendObserver(ObserverCallback cb, EventLoopToken *token);
    Result removeObserver(EventLoopToken *token);
    
public:
    bool inSameThread() const { return std::this_thread::get_id() == thread_id_; }
    std::thread::id threadId() const { return thread_id_; }
    Result appendTask(Task task, EventLoopToken *token, const char *debug_str);
    Result appendDelayedTask(uint32_t delay_ms, Task task, EventLoopToken *token, const char *debug_str);

    template<typename F>
    auto invoke(F &&f, EventLoopToken *token=nullptr, const char *debug_str=nullptr)
    {
        Result err;
        return invoke(std::forward<F>(f), err, token, debug_str);
    }

    template<typename F, std::enable_if_t<!std::is_void<decltype(std::declval<F>()())>{}, int> = 0>
    auto invoke(F &&f, Result &err, EventLoopToken *token=nullptr, const char *debug_str=nullptr)
    {
        static_assert(!std::is_same<decltype(f()), void>{}, "is void");
        if (inSameThread()) {
            return f();
        }
        using ReturnType = decltype(f());
        ReturnType retval;
        auto task_sync = [&] { retval = f(); };
        err = sync(std::move(task_sync), token, debug_str);
        return retval;
    }

    template<typename F, std::enable_if_t<std::is_void<decltype(std::declval<F>()())>{}, int> = 0>
    void invoke(F &&f, Result &err, EventLoopToken *token=nullptr, const char *debug_str=nullptr)
    {
        static_assert(std::is_same<decltype(f()), void>{}, "not void");
        if (inSameThread()) {
            return f();
        }
        err = sync(std::forward<F>(f), token, debug_str);
    }

    template<typename F, std::enable_if_t<!std::is_copy_constructible<F>{}, int> = 0>
    Result sync(F &&f, EventLoopToken *token=nullptr, const char *debug_str=nullptr)
    {
        lambda_wrapper<F> wf{std::forward<F>(f)};
        return sync(Task(std::move(wf)), token, debug_str);
    }
    Result sync(Task task, EventLoopToken *token=nullptr, const char *debug_str=nullptr);

    template<typename F, std::enable_if_t<!std::is_copy_constructible<F>{}, int> = 0>
    Result async(F &&f, EventLoopToken *token=nullptr, const char *debug_str=nullptr)
    {
        lambda_wrapper<F> wf{std::forward<F>(f)};
        return async(Task(std::move(wf)), token, debug_str);
    }
    Result async(Task task, EventLoopToken *token=nullptr, const char *debug_str=nullptr);

    template<typename F, std::enable_if_t<!std::is_copy_constructible<F>{}, int> = 0>
    Result post(F &&f, EventLoopToken *token=nullptr, const char *debug_str=nullptr)
    {
        lambda_wrapper<F> wf{std::forward<F>(f)};
        return post(Task(std::move(wf)), token, debug_str);
    }
    Result post(Task task, EventLoopToken *token=nullptr, const char *debug_str=nullptr);

    template<typename F, std::enable_if_t<!std::is_copy_constructible<F>{}, int> = 0>
    Result postDelayed(uint32_t delay_ms, F &&f, EventLoopToken *token=nullptr, const char *debug_str=nullptr)
    {
        lambda_wrapper<F> wf{std::forward<F>(f)};
        return postDelayed(delay_ms, Task(std::move(wf)), token, debug_str);
    }
    Result postDelayed(uint32_t delay_ms, Task task, EventLoopToken *token=nullptr, const char *debug_str=nullptr);

    void wakeup();
    
    void loopOnce(uint32_t max_wait_ms);
    void loop(uint32_t max_wait_ms = -1);
    void stop();
    bool stopped() const { return stop_loop_; }
    void reset() { stop_loop_ = false; }

    void appendPendingObject(PendingObject *obj);
    void removePendingObject(PendingObject *obj);

protected:
    void processTasks();
    
protected:
    using ObserverQueue = DLQueue<ObserverCallback>;
    using LockType = std::mutex;
    using LockGuard = std::lock_guard<LockType>;
    
    IOPoll*             poll_;
    std::atomic<bool>   stop_loop_{ false };
    std::thread::id     thread_id_;
    
    TaskQueue           task_queue_;
    LockType            task_mutex_;
    
    ObserverQueue       obs_queue_;
    LockType            obs_mutex_;
    
    TimerManager::Ptr   timer_mgr_;

    PendingObject*      pending_objects_ = nullptr;
};

class EventLoop::Token::Impl
{
public:
    Impl();
    ~Impl();
    
    void eventLoop(const EventLoopPtr &loop);
    EventLoopPtr eventLoop() const;
    
    void appendTaskNode(TokenTaskSlotPtr &node);
    void clearInactiveTasks();
    void appendDelayedTaskNode(DelayedTaskSlotPtr &node);
    void clearInactiveDelayedTasks();
    void clearAllTasks();
    
    bool expired() const;
    void reset();
    
protected:
    using LockType = std::mutex;
    using LockGuard = std::lock_guard<LockType>;
    friend class EventLoop::Impl;
    EventLoopWeakPtr loop_;
    
    TokenTaskQueue ttask_nodes_;
    DelayedTaskQueue dtask_nodes_;
    LockType mutex_;
    // when reset() is called from multiple different threads, 
    // all of them need wait for completion of all pending tasks
    TokenTaskQueue pending_ttask_nodes_;
    DelayedTaskQueue pending_dtask_nodes_;
    
    bool observed = false;
    ObserverToken obs_token_;
};

KEV_NS_END

#endif
