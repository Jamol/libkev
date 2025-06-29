/* Copyright (c) 2014-2020, Fengping Bao <jamol@live.com>
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

#ifndef __TimerManager_H__
#define __TimerManager_H__

#include "../include/kevdefs.h"
#include "../include/kev.h"

#include <memory>
#include <mutex>
#include <atomic>
#include <thread>

#ifndef TICK_COUNT_TYPE
# define TICK_COUNT_TYPE    uint64_t
#endif

KEV_NS_BEGIN

#define TIMER_VECTOR_BITS   8
#define TIMER_VECTOR_SIZE   (1 << TIMER_VECTOR_BITS)
#define TIMER_VECTOR_MASK   (TIMER_VECTOR_SIZE - 1)
#define TV_COUNT            4

class TimerManager
{
public:
    using Ptr = std::shared_ptr<TimerManager>;
    using TimerCallback = std::function<void(void)>;
    class TimerNode;

    TimerManager(EventLoop::Impl* loop);
    ~TimerManager();

    bool scheduleTimer(TimerNode *timer, uint32_t delay_ms, Timer::Mode mode, TimerCallback cb);
    void cancelTimer(TimerNode *timer);

    int checkExpire(unsigned long* remain_ms = nullptr);

    void setRunningThreadId(std::thread::id tid) { running_tid_ = std::move(tid); }
    bool inSameThread() const { return std::this_thread::get_id() == running_tid_; }

    void clear();
    void shutdown();

public:
    class TimerNode
    {
    public:
        TimerNode() = default;
        TimerNode(const TimerNode&) = delete;
        TimerNode(TimerNode&& other) = delete;
        TimerNode& operator= (const TimerNode &other) = delete;
        TimerNode& operator= (TimerNode &&other) = delete;
        void operator() ()
        {
            if (!cancelled_ && cb_) {
                cb_();
            }
        }
        void resetNode()
        {
            tv_index_ = -1;
            tl_index_ = -1;
            prev_ = nullptr;
            next_ = nullptr;
        }
        auto cancel()
        {
            cancelled_ = true;
            // NOTE: this TimerNode object may be destroyed when cb_ is reset
            return std::exchange(cb_, nullptr);
        }
        
        std::atomic<bool>   cancelled_{ true };
        Timer::Mode         mode_{ Timer::Mode::ONE_SHOT };
        uint32_t            delay_ms_{ 0 };
        TICK_COUNT_TYPE     start_tick_{ 0 };
        // NOTE: timer callback will be reset after timer cancelled or executed, 
        //       or when TimerManager destructed
        // NOTE: the TimerNode may be destroyed when TimerCallback is reset,
        //       for example, the DealyedTaskSlotPtr is stored in TimerCallback
        //       and will be destroyed when TimerCallback is reset
        // NOTE: must not destruct the cb_ under lock of TimerManager::mutex_,
        //       since its destruction may result to another timer to be cancelled
        // NOTE: the callback will be delayed destroyed after its execution if
        //       timer cancel is called from callback
        TimerCallback       cb_;
        
    protected:
        friend class TimerManager;
        int tv_index_{ -1 };
        int tl_index_{ -1 };
        TimerNode* prev_{ nullptr };
        TimerNode* next_{ nullptr };
    };
    
private:
    typedef enum {
        FROM_SCHEDULE,
        FROM_CASCADE,
        FROM_RESCHEDULE
    } FROM;
    bool addTimer(TimerNode* timer_node, FROM from);
    void removeTimer(TimerNode* timer_node);
    int cascadeTimer(int tv_idx, int tl_idx);
    bool isTimerPending(TimerNode* timer_node)
    {
        return timer_node->next_ != nullptr;
    }

    void list_init_head(TimerNode* head);
    void list_add_node(TimerNode* head, TimerNode* timer_node);
    void list_remove_node(TimerNode* timer_node);
    void list_replace(TimerNode* old_head, TimerNode* new_head);
    void list_combine(TimerNode* from_head, TimerNode* to_head);
    bool list_empty(TimerNode* head);
    
    void set_tv0_bitmap(int idx);
    void clear_tv0_bitmap(int idx);
    int find_first_set_in_bitmap(int idx);

    void handleRunningTimerCancellationLocked(TimerNode* timer_node, std::unique_lock<std::mutex>& ul);
    void handlePendingTimerCancellationLocked(TimerNode* timer_node);

private:
    EventLoop::Impl* loop_;
    std::mutex mutex_;
    std::mutex running_mutex_;
    std::thread::id running_tid_;
    TimerNode* running_node_{ nullptr };
    TimerNode* reschedule_node_{ nullptr };
    TimerNode* cancelling_node_{ nullptr };
    bool shutdown_{ false };
    unsigned long last_remain_ms_ = -1;
    TICK_COUNT_TYPE last_tick_{ 0 };
    uint32_t timer_count_{ 0 };
    uint32_t tv0_bitmap_[8]; // 1 -- have timer in this slot
    TimerNode tv_[TV_COUNT][TIMER_VECTOR_SIZE]; // timer vectors
};

class Timer::Impl
{
public:
    using TimerCallback = Timer::TimerCallback;
    
    Impl(TimerManager::Ptr mgr);
    Impl(const Impl &other) = delete;
    Impl(Impl &&other) = delete;
    ~Impl();

    Impl& operator= (const Impl &other) = delete;
    Impl& operator= (Impl &&other) = delete;
    
    template<typename F, std::enable_if_t<!std::is_copy_constructible<F>{}, int> = 0>
    bool schedule(uint32_t delay_ms, Mode mode, F &&f)
    {
        lambda_wrapper<F> wf{std::forward<F>(f)};
        return schedule(delay_ms, mode, TimerCallback(std::move(wf)));
    }
    bool schedule(uint32_t delay_ms, Mode mode, TimerCallback cb);
    void cancel();
    
private:
    friend class TimerManager;
    std::weak_ptr<TimerManager> timer_mgr_;
    TimerManager::TimerNode timer_node_; // intrusive list node
};

KEV_NS_END

#endif
