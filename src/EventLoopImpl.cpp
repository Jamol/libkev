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

#include "EventLoopImpl.h"
#include "poll/IOPoll.h"
#include "util/kmqueue.h"
#include "util/kmtrace.h"
#include "util/skutils.h"
#include <thread>
#include <condition_variable>

KEV_NS_BEGIN

IOPoll* createIOPoll(PollType poll_type);

EventLoop::Impl::Impl(PollType poll_type)
: poll_(createIOPoll(poll_type))
, timer_mgr_(new TimerManager(this))
{
    KM_SetObjKey("EventLoop");
}

EventLoop::Impl::~Impl()
{
    while (pending_objects_) {
        auto obj = pending_objects_;
        pending_objects_ = pending_objects_->next_;
        obj->onLoopExit();
    }
    ObserverCallback cb;
    while (obs_queue_.dequeue(cb)) {
        cb(LoopActivity::EXIT);
    }
    if(poll_) {
        delete poll_;
        poll_ = nullptr;
    }
}

bool EventLoop::Impl::init()
{
    if(!poll_->init()) {
        return false;
    }
    stop_loop_ = false;
    thread_id_ = std::this_thread::get_id();
    return true;
}

PollType EventLoop::Impl::getPollType() const
{
    if(poll_) {
        return poll_->getType();
    }
    return PollType::NONE;
}

bool EventLoop::Impl::isPollLT() const
{
    if(poll_) {
        return poll_->isLevelTriggered();
    }
    return false;
}

Result EventLoop::Impl::registerFd(SOCKET_FD fd, uint32_t events, IOCallback cb)
{
    if(inSameThread()) {
        return poll_->registerFd(fd, events, std::move(cb));
    }
    return async([=, cb=std::move(cb)] () mutable {
        auto ret = poll_->registerFd(fd, events, cb);
        if(ret != Result::OK) {
            return ;
        }
    });
}

Result EventLoop::Impl::updateFd(SOCKET_FD fd, uint32_t events)
{
    if(inSameThread()) {
        return poll_->updateFd(fd, events);
    }
    return async([=] {
        auto ret = poll_->updateFd(fd, events);
        if(ret != Result::OK) {
            return ;
        }
    });
}

Result EventLoop::Impl::unregisterFd(SOCKET_FD fd, bool close_fd)
{
    if(inSameThread()) {
        auto ret = poll_->unregisterFd(fd);
        if(close_fd) {
            SKUtils::close(fd);
        }
        return ret;
    } else {
        auto ret = sync([=] {
            poll_->unregisterFd(fd);
            if(close_fd) {
                SKUtils::close(fd);
            }
        });
        if(Result::OK != ret) {
            return ret;
        }
        return Result::OK;
    }
}

Result EventLoop::Impl::appendObserver(ObserverCallback cb, EventLoopToken *token)
{
    if (token && token->eventLoop().get() != this) {
        return Result::INVALID_PARAM;
    }
    LockGuard g(obs_mutex_);
    if (stop_loop_) {
        return Result::INVALID_STATE;
    }
    auto obs_node = obs_queue_.enqueue(std::move(cb));
    if (token) {
        token->obs_token_ = obs_node;
        token->observed = true;
    }
    return Result::OK;
}

Result EventLoop::Impl::removeObserver(EventLoopToken *token)
{
    if (token) {
        if (token->eventLoop().get() != this) {
            return Result::INVALID_STATE;
        }
        auto node = token->obs_token_.lock();
        if (node) {
            LockGuard g(obs_mutex_);
            obs_queue_.remove(node);
        }
        token->obs_token_.reset();
        token->observed = false;
    }
    return Result::OK;
}

void EventLoop::Impl::appendPendingObject(PendingObject *obj)
{
    KM_ASSERT(inSameThread());
    if (pending_objects_) {
        obj->next_ = pending_objects_;
        pending_objects_->prev_ = obj;
    }
    pending_objects_ = obj;
}

void EventLoop::Impl::removePendingObject(PendingObject *obj)
{
    KM_ASSERT(inSameThread());
    if (pending_objects_ == obj) {
        pending_objects_ = obj->next_;
    }
    if (obj->prev_) {
        obj->prev_->next_ = obj->next_;
    }
    if (obj->next_) {
        obj->next_->prev_ = obj->prev_;
    }
}

void EventLoop::Impl::processTasks()
{
    TaskQueue tq;
    std::unique_lock<LockType> ul(task_mutex_);
    task_queue_.swap(tq);
    
    while (auto node = tq.front_node()) {
        tq.pop_front();
        auto &task_slot = node->element();
        task_slot.state = TaskSlot::State::RUNNING;
        ul.unlock();
        {// execute the task
            LockGuard g(task_run_mutex_);
            if (task_slot.state != TaskSlot::State::INACTIVE) {
                task_slot();
                task_slot.state = TaskSlot::State::INACTIVE;
            }
        }
        ul.lock();
        if (task_slot.token) {
            task_slot.token->removeTaskNode(node);
        }
    }
}

void EventLoop::Impl::loopOnce(uint32_t max_wait_ms)
{
    processTasks();
    unsigned long wait_ms = max_wait_ms;
    timer_mgr_->checkExpire(&wait_ms);
    if(wait_ms > max_wait_ms) {
        wait_ms = max_wait_ms;
    }
    poll_->wait((uint32_t)wait_ms);
}

void EventLoop::Impl::loop(uint32_t max_wait_ms)
{
    while (!stop_loop_) {
        loopOnce(max_wait_ms);
    }
    processTasks();
    
    while (pending_objects_) {
        auto obj = pending_objects_;
        pending_objects_ = pending_objects_->next_;
        obj->onLoopExit();
    }
    {
        LockGuard g(obs_mutex_);
        ObserverCallback cb;
        while (obs_queue_.dequeue(cb)) {
            cb(LoopActivity::EXIT);
        }
    }
    KM_INFOXTRACE("loop, stopped");
}

void EventLoop::Impl::stop()
{
    KM_INFOXTRACE("stop");
    stop_loop_ = true;
    wakeup();
}

Result EventLoop::Impl::appendTask(Task task, EventLoopToken *token, const char *debugStr)
{
    if (token && token->eventLoop().get() != this) {
        return Result::INVALID_PARAM;
    }
    std::string dstr{debugStr ? debugStr : ""};
    auto node = std::make_shared<TaskQueue::DLNode>(std::move(task), token, std::move(dstr));
    LockGuard g(task_mutex_);
    if (stop_loop_) {
        return Result::INVALID_STATE;
    }
    task_queue_.enqueue(node);
    if (token) {
        token->appendTaskNode(node);
    }
    return Result::OK;
}

Result EventLoop::Impl::removeTask(EventLoopToken *token)
{
    if (!token || token->eventLoop().get() != this) {
        return Result::INVALID_PARAM;
    }
    bool is_running = false;
    {
        LockGuard g(task_mutex_);
        for (auto &node : token->task_nodes_) {
            auto &task_slot = node->element();
            if (task_slot.state == TaskSlot::State::RUNNING) {
                is_running = true;
                task_slot.state = TaskSlot::State::INACTIVE;
                task_slot.token = nullptr;
            }
            task_queue_.remove(node);
        }
        token->task_nodes_.clear();
    }
    if (is_running && !inSameThread()) {
        // wait for end of running
        task_run_mutex_.lock();
        task_run_mutex_.unlock();
    }
    return Result::OK;
}


Result EventLoop::Impl::appendDelayedTask(uint32_t delay_ms, Task task, EventLoopToken *token, const char *debugStr)
{
    if (token && token->eventLoop().get() != this) {
        return Result::INVALID_PARAM;
    }
    std::string dstr{debugStr ? debugStr : ""};
    auto node = std::make_shared<DelayedTaskQueue::DLNode>(this, token, std::move(dstr));
    {
        LockGuard g(task_mutex_);
        if (stop_loop_) {
            return Result::INVALID_STATE;
        }
        dtask_queue_.enqueue(node);
        if (token) {
            token->appendDelayedTaskNode(node);
        }
    }
    std::weak_ptr<DelayedTaskQueue::DLNode> weak_node = node;
    node->element().schedule(delay_ms, [this, t=std::move(task), wn=std::move(weak_node)] () mutable {
        if (t) t();
        auto node = wn.lock();
        if (node) {
            removeDelayedTaskNode(node);
        }
    });
    return Result::OK;
}

Result EventLoop::Impl::removeDelayedTask(EventLoopToken *token)
{
    if (!token || token->eventLoop().get() != this) {
        return Result::INVALID_PARAM;
    }
    LockGuard g(task_mutex_);
    for (auto &node : token->dtask_nodes_) {
        auto &dtask_slot = node->element();
        dtask_slot.token = nullptr;
        dtask_slot.cancel();
        dtask_queue_.remove(node);
    }
    token->dtask_nodes_.clear();
    return Result::OK;
}

Result EventLoop::Impl::removeDelayedTaskNode(DelayedTaskNodePtr &node)
{
    LockGuard g(task_mutex_);
    auto &dtask_slot = node->element();
    dtask_slot.cancel();
    if (dtask_slot.token) {
        dtask_slot.token->removeDelayedTaskNode(node);
        dtask_slot.token = nullptr;
    }
    dtask_queue_.remove(node);
    return Result::OK;
}

Result EventLoop::Impl::sync(Task task)
{
    if(inSameThread()) {
        task();
    } else {
        std::mutex m;
        std::condition_variable cv;
        bool ready = false;
        Task task_sync([&] {
            task();
            std::unique_lock<std::mutex> lk(m);
            ready = true;
            cv.notify_one(); // the waiting thread may block again since m is not released
            lk.unlock();
        });
        auto ret = post(std::move(task_sync));
        if (ret != Result::OK) {
            return ret;
        }
        std::unique_lock<std::mutex> lk(m);
        cv.wait(lk, [&ready] { return ready; });
    }
    return Result::OK;
}

Result EventLoop::Impl::async(Task task, EventLoopToken *token, const char *debugStr)
{
    if(inSameThread()) {
        task();
        return Result::OK;
    } else {
        return post(std::move(task), token, debugStr);
    }
}

Result EventLoop::Impl::post(Task task, EventLoopToken *token, const char *debugStr)
{
    auto ret = appendTask(std::move(task), token, debugStr);
    if (ret != Result::OK) {
        return ret;
    }
    wakeup();
    return Result::OK;
}

Result EventLoop::Impl::postDelayed(uint32_t delay_ms, Task task, EventLoopToken *token, const char *debugStr)
{
    return  appendDelayedTask(delay_ms, std::move(task), token, debugStr);
}

void EventLoop::Impl::wakeup()
{
    poll_->notify();
}

/////////////////////////////////////////////////////////////////
// DelayedTaskSlot
DelayedTaskSlot::DelayedTaskSlot(EventLoop::Impl *loop, EventLoopToken *token, std::string debugStr)
    : token(token), debugStr(std::move(debugStr)), timer(loop->getTimerMgr())
{

}

/////////////////////////////////////////////////////////////////
// EventLoop::Token::Impl
EventLoop::Token::Impl::Impl()
{
    
}

EventLoop::Token::Impl::~Impl()
{
    reset();
}

void EventLoop::Token::Impl::eventLoop(const EventLoopPtr &loop)
{
    loop_ = loop;
}

EventLoopPtr EventLoop::Token::Impl::eventLoop()
{
    return loop_.lock();
}

void EventLoop::Token::Impl::appendTaskNode(TaskNodePtr &node)
{
    task_nodes_.emplace_back(node);
}

void EventLoop::Token::Impl::removeTaskNode(TaskNodePtr &node)
{
    for (auto it = task_nodes_.begin(); it != task_nodes_.end(); ++it) {
        if (*it == node) {
            task_nodes_.erase(it);
            break;
        }
    }
}

void EventLoop::Token::Impl::appendDelayedTaskNode(DelayedTaskNodePtr &node)
{
    dtask_nodes_.emplace_back(node);
}

void EventLoop::Token::Impl::removeDelayedTaskNode(DelayedTaskNodePtr &node)
{
    for (auto it = dtask_nodes_.begin(); it != dtask_nodes_.end(); ++it) {
        if (*it == node) {
            dtask_nodes_.erase(it);
            break;
        }
    }
}

bool EventLoop::Token::Impl::expired()
{
    return loop_.expired() || (observed && obs_token_.expired());
}

void EventLoop::Token::Impl::reset()
{
    auto loop = loop_.lock();
    if (loop) {
        if (!task_nodes_.empty()) {
            loop->removeTask(this);
        }
        if (!dtask_nodes_.empty()) {
            loop->removeDelayedTask(this);
        }
        if (!obs_token_.expired()) {
            loop->removeObserver(this);
            obs_token_.reset();
        }
        loop_.reset();
    } else {
        task_nodes_.clear();
    }
}

KEV_NS_END

/////////////////////////////////////////////////////////////////
//

#ifdef KUMA_OS_WIN
# include <MSWSock.h>
extern LPFN_CONNECTEX connect_ex;
extern LPFN_ACCEPTEX accept_ex;
#endif

KEV_NS_BEGIN

IOPoll* createEPoll();
IOPoll* createVPoll();
IOPoll* createKQueue();
IOPoll* createSelectPoll();
IOPoll* createIocpPoll();

IOPoll* createDefaultIOPoll()
{
#ifdef KUMA_OS_WIN
    if (connect_ex && accept_ex) {
        return createIocpPoll();
    }
    return createSelectPoll();
#elif defined(KUMA_OS_LINUX)
    return createEPoll();
#elif defined(KUMA_OS_MAC)
    return createKQueue();
    //return createVPoll();
#else
    return createSelectPoll();
#endif
}

IOPoll* createIOPoll(PollType poll_type)
{
    switch (poll_type)
    {
        case PollType::POLL:
            return createVPoll();
        case PollType::SELECT:
            return createSelectPoll();
        case PollType::KQUEUE:
#ifdef KUMA_OS_MAC
            return createKQueue();
#else
            return createDefaultIOPoll();
#endif
        case PollType::EPOLL:
#ifdef KUMA_OS_LINUX
            return createEPoll();
#else
            return createDefaultIOPoll();
#endif
        case PollType::IOCP:
#ifdef KUMA_OS_WIN
            return createIocpPoll();
#else
            return createDefaultIOPoll();
#endif
        default:
            return createDefaultIOPoll();
    }
}

KEV_NS_END
