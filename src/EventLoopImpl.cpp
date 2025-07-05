/* Copyright (c) 2014-2025, Fengping Bao <jamol@live.com>
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
#include "utils/kmqueue.h"
#include "utils/kmtrace.h"
#include "utils/skutils.h"
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
    cleanupPendingObjects();
    notifyObservers(LoopActivity::EXIT);
    // Shutdown timer manager before its weak pointers expire
    timer_mgr_->shutdown();
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
    thread_id_ = std::this_thread::get_id();
    timer_mgr_->setRunningThreadId(thread_id_);
    return true;
}

PollType EventLoop::Impl::getPollType() const
{
    return poll_ ? poll_->getType() : PollType::DEFAULT;
}

bool EventLoop::Impl::isPollLT() const
{
    return poll_ ? poll_->isLevelTriggered() : false;
}

Result EventLoop::Impl::registerFd(SOCKET_FD fd, uint32_t events, IOCallback cb)
{
    if (getPollType() == PollType::STLCV) {
        return Result::NOT_SUPPORTED;
    }
    if(inSameThread()) {
        return poll_->registerFd(fd, events, std::move(cb));
    }
    return async([=, cb{std::move(cb)}] () mutable {
        auto ret = poll_->registerFd(fd, events, std::move(cb));
        if(ret != Result::OK) {
            return ;
        }
    });
}

Result EventLoop::Impl::updateFd(SOCKET_FD fd, uint32_t events)
{
    if (getPollType() == PollType::STLCV) {
        return Result::NOT_SUPPORTED;
    }
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
    if (getPollType() == PollType::STLCV) {
        return Result::NOT_SUPPORTED;
    }
    if(inSameThread()) {
        auto ret = poll_->unregisterFd(fd);
        if(close_fd) {
            SKUtils::close(fd);
        }
        return ret;
    }
    return sync([=] {
        poll_->unregisterFd(fd);
        if(close_fd) {
            SKUtils::close(fd);
        }
    });
}

Result EventLoop::Impl::submitOp(SOCKET_FD fd, const Op &op)
{
    return poll_->submitOp(fd, op);
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

void EventLoop::Impl::notifyObservers(LoopActivity activity)
{
    ObserverCallback cb;
    while (obs_queue_.dequeue(cb)) {
        if (cb) {
            cb(activity);
        }
    }
}

void EventLoop::Impl::appendPendingObject(PendingObject *obj)
{
    KM_ASSERT(inSameThread());
    KM_ASSERT(obj != nullptr);
    if (pending_objects_) {
        obj->next_ = pending_objects_;
        pending_objects_->prev_ = obj;
    }
    pending_objects_ = obj;
}

void EventLoop::Impl::removePendingObject(PendingObject *obj)
{
    KM_ASSERT(inSameThread());
    KM_ASSERT(obj != nullptr);
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

void EventLoop::Impl::cleanupPendingObjects()
{
    while (pending_objects_) {
        auto* obj = pending_objects_;
        pending_objects_ = pending_objects_->next_;
        obj->onLoopExit();
    }
}

void EventLoop::Impl::processTasks()
{
    TaskQueue tq;
    {
        std::unique_lock<LockType> ul(task_mutex_);
        task_queue_.swap(tq);
    }

    for (auto &ts : tq) {
        (*ts)();
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
    if (!task_queue_.empty()) {
        wait_ms = 0;
    }
    poll_->wait((uint32_t)wait_ms);
}

void EventLoop::Impl::loop(uint32_t max_wait_ms)
{
    if (init()) {
        while (!stop_loop_) {
            loopOnce(max_wait_ms);
        }
    }
    
    processTasks();
    
    cleanupPendingObjects();
    {
        LockGuard g(obs_mutex_);
        notifyObservers(LoopActivity::EXIT);
    }
    KM_INFOXTRACE("loop, stopped");
}

void EventLoop::Impl::stop()
{
    stop_loop_ = true;
    wakeup();
}

Result EventLoop::Impl::appendTask(Task task, EventLoopToken *token, const char *debug_str)
{
    if (token && token->eventLoop().get() != this) {
        return Result::INVALID_PARAM;
    }
    if (stop_loop_) {
        return Result::INVALID_STATE;
    }
    std::string dstr{debug_str ? debug_str : ""};
    TaskSlotPtr ptr;
    if (token) {
        auto p = std::make_shared<TokenTaskSlot>(std::move(task), std::move(dstr));
        token->appendTaskNode(p);
        ptr = std::move(p);
    } else {
        ptr = std::make_shared<TaskSlot>(std::move(task), std::move(dstr));
    }
    bool need_wakeup = false;
    {
        LockGuard g(task_mutex_);
        need_wakeup = task_queue_.empty();
        task_queue_.emplace_back(std::move(ptr));
    }
    if (need_wakeup) {
        wakeup();
    }
    return Result::OK;
}

Result EventLoop::Impl::appendDelayedTask(uint32_t delay_ms, Task task, EventLoopToken *token, const char *debug_str)
{
    if (token && token->eventLoop().get() != this) {
        return Result::INVALID_PARAM;
    }
    if (stop_loop_) {
        return Result::INVALID_STATE;
    }
    std::string dstr{debug_str ? debug_str : ""};
    auto ptr = std::make_shared<DelayedTaskSlot>(this, std::move(task), std::move(dstr));
    if (token) {
        token->appendDelayedTaskNode(ptr);
    }
    // NOTE: ptr is stored in TimerCallback, so no queue is needed for DelayedTaskSlot.
    // ptr will be released after timer cancelled or executed, or TimerManager destructed
    ptr->timer.schedule(delay_ms, Timer::Mode::ONE_SHOT, [ptr] () mutable {
        // this closure will be released when user reset the token 
        (*std::exchange(ptr, nullptr))();
    });
    return Result::OK;
}

template <typename Callable>
class auto_clean
{
public:
    auto_clean(Callable &&c) : c_(std::move(c)) {}
    auto_clean(auto_clean&& other)
    : c_(std::move(other.c_)), cleared_(other.cleared_)
    {
        other.cleared_ = true;
    }
    auto_clean(const auto_clean&) = delete;
    ~auto_clean() { if (!cleared_) c_(); }
    
    auto_clean& operator=(const auto_clean&) = delete;
    auto_clean& operator=(auto_clean&&) = delete;

private:
    Callable c_;
    bool cleared_ = false;
};
Result EventLoop::Impl::sync(Task task, EventLoopToken *token, const char *debug_str)
{
    if(inSameThread()) {
        task();
        return Result::OK;
    } else {
        std::mutex m;
        std::condition_variable cv;
        bool ready = false;
        bool executed = false;
        auto clean = [&] {
            std::lock_guard<std::mutex> g(m);
            ready = true;
            cv.notify_one(); // the waiting thread may block again since m is not released
        };
        auto_clean<decltype(clean)> sc(std::move(clean));
        auto task_sync = [&, sc{std::move(sc)}] {
            task();
            executed = true;
        };
        auto ret = post(std::move(task_sync), token, debug_str);
        if (ret != Result::OK) {
            return ret;
        }
        std::unique_lock<std::mutex> lk(m);
        cv.wait(lk, [&ready] { return ready; });
        return executed ? Result::OK : Result::ABORTED;
    }
}

Result EventLoop::Impl::async(Task task, EventLoopToken *token, const char *debug_str)
{
    if(inSameThread()) {
        task();
        return Result::OK;
    }
    return post(std::move(task), token, debug_str);
}

Result EventLoop::Impl::post(Task task, EventLoopToken *token, const char *debug_str)
{
    return appendTask(std::move(task), token, debug_str);
}

Result EventLoop::Impl::postDelayed(uint32_t delay_ms, Task task, EventLoopToken *token, const char *debug_str)
{
    return  appendDelayedTask(delay_ms, std::move(task), token, debug_str);
}

void EventLoop::Impl::wakeup()
{
    poll_->notify();
}

/////////////////////////////////////////////////////////////////
// DelayedTaskSlot
DelayedTaskSlot::DelayedTaskSlot(EventLoop::Impl *loop, EventLoop::Task &&t, std::string debug_str)
    : TaskSlot(std::move(t), std::move(debug_str)), timer(loop->getTimerMgr())
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

EventLoopPtr EventLoop::Token::Impl::eventLoop() const
{
    return loop_.lock();
}

void EventLoop::Token::Impl::appendTaskNode(TokenTaskSlotPtr &node)
{
    LockGuard g(mutex_);
    clearInactiveTasks();
    ttask_nodes_.emplace_back(node);
}

void EventLoop::Token::Impl::clearInactiveTasks()
{
    for (auto it = ttask_nodes_.begin(); it != ttask_nodes_.end(); ) {
        if (!(*it)->isActive()) {
            it = ttask_nodes_.erase(it);
        } else {
            break;
        }
    }
}

void EventLoop::Token::Impl::appendDelayedTaskNode(DelayedTaskSlotPtr &node)
{
    LockGuard g(mutex_);
    clearInactiveDelayedTasks();
    dtask_nodes_.emplace_back(node);
}

void EventLoop::Token::Impl::clearInactiveDelayedTasks()
{
    for (auto it = dtask_nodes_.begin(); it != dtask_nodes_.end(); ) {
        if (!(*it)->isActive()) {
            //(*it)->cancel();
            it = dtask_nodes_.erase(it);
        } else {
            break;
        }
    }
}

void EventLoop::Token::Impl::clearAllTasks()
{
    auto loop = loop_.lock();

    std::unique_lock<std::mutex> ul(mutex_);
    pending_ttask_nodes_.splice(pending_ttask_nodes_.end(), std::move(ttask_nodes_));
    pending_dtask_nodes_.splice(pending_dtask_nodes_.end(), std::move(dtask_nodes_));

    while(!pending_ttask_nodes_.empty()) {
        bool pop_task = true;
        auto ts = pending_ttask_nodes_.front();
        if (ts->isActive()) {
            ul.unlock();
            ts->cancel(loop && loop->inSameThread());
            ul.lock();
            pop_task = !pending_ttask_nodes_.empty() && ts == pending_ttask_nodes_.front();
        }
        if (pop_task) {
            pending_ttask_nodes_.pop_front();
        }
    }

    while(!pending_dtask_nodes_.empty()) {
        bool pop_task = true;
        auto ds = pending_dtask_nodes_.front();
        if (ds->isActive()) {
            ul.unlock();
            ds->cancel();
            ul.lock();
            pop_task = !pending_dtask_nodes_.empty() && ds == pending_dtask_nodes_.front();
        }
        if (pop_task) {
            pending_dtask_nodes_.pop_front();
        }
    }
}

bool EventLoop::Token::Impl::expired() const
{
    return loop_.expired() || (observed && obs_token_.expired());
}

void EventLoop::Token::Impl::reset()
{
    auto loop = loop_.lock();
    if (loop) {
        if (!obs_token_.expired()) {
            loop->removeObserver(this);
            obs_token_.reset();
        }
    }
    clearAllTasks();
}

KEV_NS_END

/////////////////////////////////////////////////////////////////
//

#ifdef KUMA_OS_WIN
# include <MSWSock.h>
#endif

extern bool kev_init();

KEV_NS_BEGIN

#ifdef KUMA_OS_WIN
extern LPFN_CONNECTEX connect_ex;
extern LPFN_ACCEPTEX accept_ex;
#endif

IOPoll* createEPoll();
IOPoll* createVPoll();
IOPoll* createKQueue();
IOPoll* createSelectPoll();
IOPoll* createIocpPoll();
IOPoll* createIOUring();
IOPoll* createRunLoop();
IOPoll* createCVPoll();

IOPoll* createDefaultIOPoll()
{
#ifdef KUMA_OS_WIN
    if (connect_ex && accept_ex) {
        return createIocpPoll();
    }
    return createSelectPoll();
#elif defined(KUMA_OS_LINUX) || defined(KUMA_OS_OHOS)
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
#ifdef KUMA_OS_WIN
    kev_init(); // in case of calling from global EventLoop variable
#endif
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
#if defined(KUMA_OS_LINUX) || defined(KUMA_OS_OHOS)
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
        case PollType::IORING: {
#if defined(KEV_HAS_IOURING)
            auto *poller = createIOUring();
            if (poller) {
                return poller;
            }
#endif
            return createDefaultIOPoll();
        }
        case PollType::RUNLOOP:
#if defined(KUMA_OS_MAC) && defined(KEV_HAS_RUNLOOP)
            return createRunLoop();
#else
            return createDefaultIOPoll();
#endif
        case PollType::STLCV:
            return createCVPoll();
        default:
            return createDefaultIOPoll();
    }
}

KEV_NS_END
