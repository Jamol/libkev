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

#ifndef __EventLoopImpl_H__
#define __EventLoopImpl_H__

#include "../include/kev.h"
#include "../include/kevdefs.h"
#include "util/kmqueue.h"
#include "TimerManager.h"
#include "util/kmobject.h"

#ifdef KUMA_OS_WIN
# include <Ws2tcpip.h>
#endif
#include <stdint.h>
#include <thread>
#include <list>
#include <atomic>

KEV_NS_BEGIN

class IOPoll;
using EventLoopToken = EventLoop::Token::Impl;

class TaskSlot
{
public:
    TaskSlot(EventLoop::Task &&t, std::string debugStr)
    : task(std::move(t)), debugStr(std::move(debugStr)) {}
    virtual ~TaskSlot() {}
    virtual void operator() ()
    {
        if (task) {
            task_running = true;
            task();
            task = nullptr;
            task_running = false;
        }
    }
    virtual void cancel()
    {
        task = nullptr;
    }
    bool isActive() const
    {
        return bool(task);
    }
    bool isRunning() const
    {
        return task_running;
    }
    EventLoop::Task task;
    bool            task_running = false;
    std::string     debugStr;
};

class TokenTaskSlot : public TaskSlot
{
public:
    TokenTaskSlot(EventLoop::Task &&t, std::string debugStr)
    : TaskSlot(std::move(t), std::move(debugStr)) {}
    void operator() () override
    {
        std::lock_guard<std::mutex> g(mlock);
        TaskSlot::operator()();
    }
    void cancel() override
    {
        std::lock_guard<std::mutex> g(mlock);
        TaskSlot::cancel();
    }
    std::mutex      mlock;
};

using TaskSlotPtr = std::shared_ptr<TaskSlot>;
using TaskQueue = std::list<TaskSlotPtr>;

class DelayedTaskSlot : public TaskSlot
{
public:
    DelayedTaskSlot(EventLoop::Impl *loop, EventLoop::Task &&t, std::string debugStr);
    void cancel()
    {
        timer.cancel();
        TaskSlot::cancel();
    }

public:
    Timer::Impl     timer;
};
using DelayedTaskSlotPtr = std::shared_ptr<DelayedTaskSlot>;

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
    Impl(PollType poll_type = PollType::NONE);
    ~Impl();

public:
    bool init();
    Result registerFd(SOCKET_FD fd, uint32_t events, IOCallback cb);
    Result updateFd(SOCKET_FD fd, uint32_t events);
    Result unregisterFd(SOCKET_FD fd, bool close_fd);
    TimerManagerPtr getTimerMgr() { return timer_mgr_; }
    
    PollType getPollType() const;
    bool isPollLT() const; // level trigger
    
    Result appendObserver(ObserverCallback cb, EventLoopToken *token);
    Result removeObserver(EventLoopToken *token);
    
public:
    bool inSameThread() const { return std::this_thread::get_id() == thread_id_; }
    std::thread::id threadId() const { return thread_id_; }
    Result appendTask(Task task, EventLoopToken *token, const char *debugStr);
    Result removeTask(EventLoopToken *token);
    Result appendDelayedTask(uint32_t delay_ms, Task task, EventLoopToken *token, const char *debugStr);
    Result removeDelayedTask(EventLoopToken *token);

    template<typename F>
    auto invoke(F &&f)
    {
        Result err;
        return invoke(std::forward<F>(f), err);
    }

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

    template<typename F, std::enable_if_t<!std::is_copy_constructible<F>{}, int> = 0>
    Result sync(F &&f)
    {
        lambda_wrapper<F> wf{std::forward<F>(f)};
        return sync(Task(std::move(wf)));
    }
    Result sync(Task task);

    template<typename F, std::enable_if_t<!std::is_copy_constructible<F>{}, int> = 0>
    Result async(F &&f, EventLoopToken *token=nullptr, const char *debugStr=nullptr)
    {
        lambda_wrapper<F> wf{std::forward<F>(f)};
        return async(Task(std::move(wf)), token, debugStr);
    }
    Result async(Task task, EventLoopToken *token=nullptr, const char *debugStr=nullptr);

    template<typename F, std::enable_if_t<!std::is_copy_constructible<F>{}, int> = 0>
    Result post(F &&f, EventLoopToken *token=nullptr, const char *debugStr=nullptr)
    {
        lambda_wrapper<F> wf{std::forward<F>(f)};
        return post(Task(std::move(wf)), token, debugStr);
    }
    Result post(Task task, EventLoopToken *token=nullptr, const char *debugStr=nullptr);

    template<typename F, std::enable_if_t<!std::is_copy_constructible<F>{}, int> = 0>
    Result postDelayed(uint32_t delay_ms, F &&f, EventLoopToken *token=nullptr, const char *debugStr=nullptr)
    {
        lambda_wrapper<F> wf{std::forward<F>(f)};
        return postDelayed(delay_ms, Task(std::move(wf)), token, debugStr);
    }
    Result postDelayed(uint32_t delay_ms, Task task, EventLoopToken *token=nullptr, const char *debugStr=nullptr);

    void wakeup();
    
    void loopOnce(uint32_t max_wait_ms);
    void loop(uint32_t max_wait_ms = -1);
    void stop();
    bool stopped() const { return stop_loop_; }

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
    
    TimerManagerPtr     timer_mgr_;

    PendingObject*      pending_objects_ = nullptr;
};
using EventLoopPtr = std::shared_ptr<EventLoop::Impl>;
using EventLoopWeakPtr = std::weak_ptr<EventLoop::Impl>;

class EventLoop::Token::Impl
{
public:
    Impl();
    ~Impl();
    
    void eventLoop(const EventLoopPtr &loop);
    EventLoopPtr eventLoop();
    
    void appendTaskNode(TaskSlotPtr &node);
    void clearInactiveTask();
    void appendDelayedTaskNode(DelayedTaskSlotPtr &node);
    void clearInactiveDelayedTask();
    
    bool expired();
    void reset();
    
protected:
    friend class EventLoop::Impl;
    EventLoopWeakPtr loop_;
    
    // task_nodes_ and dtask_nodes_ are protected by EventLoop task_mutex_
    std::list<TaskSlotPtr> task_nodes_;
    std::list<DelayedTaskSlotPtr> dtask_nodes_;
    
    bool observed = false;
    ObserverToken obs_token_;
};

KEV_NS_END

#endif
