/* Copyright (c) 2022, Fengping Bao <jamol@live.com>
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

#include "IOPoll.h"
#include "EventNotifier.h"
#include "utils/kmtrace.h"

#include <sys/syscall.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/io_uring.h>
#include <sys/poll.h>
#include <sys/utsname.h>

#include <atomic>

#define io_uring_smp_store_release(p, v)                        \
    std::atomic_store_explicit(                                 \
        (std::atomic<std::decay_t<decltype(*(p))>> *)(p), (v),  \
        std::memory_order_release)

#define io_uring_smp_load_acquire(p)                            \
    std::atomic_load_explicit(                                  \
        (std::atomic<std::decay_t<decltype(*(p))>> const *)(p), \
        std::memory_order_acquire)

KEV_NS_BEGIN

#define MAX_EPOLL_FDS   5000
#define MAX_EVENT_NUM   500
#define QUEUE_DEPTH     256

#define SYS_IORING_OP_NOP           IORING_OP_NOP
#define SYS_IORING_OP_READV         IORING_OP_READV
#define SYS_IORING_OP_WRITEV        IORING_OP_WRITEV
#define SYS_IORING_OP_SENDMSG       IORING_OP_SENDMSG
#define SYS_IORING_OP_RECVMSG       IORING_OP_RECVMSG
#define SYS_IORING_OP_TIMEOUT       IORING_OP_TIMEOUT
#define SYS_IORING_OP_ACCEPT        13
#define SYS_IORING_OP_ASYNC_CANCEL  14
#define SYS_IORING_OP_CONNECT       16
#define SYS_IORING_OP_CLOSE         19
#define SYS_IORING_OP_SEND          26
#define SYS_IORING_OP_RECV          27
#define SYS_IORING_OP_SHUTDOWN      34

class IOUring final : public IOPoll
{
public:
    IOUring();
    ~IOUring();

    bool init() override;
    Result registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb) override;
    Result unregisterFd(SOCKET_FD fd) override;
    Result updateFd(SOCKET_FD fd, KMEvent events) override;
    Result wait(uint32_t wait_time_ms) override;
    void notify() override;
    PollType getType() const override { return PollType::EPOLL; }
    bool isLevelTriggered() const override { return false; }

    Result submitOp(SOCKET_FD fd, const Op &op) override;

private:
    uint32_t get_events(KMEvent kuma_events);
    KMEvent get_kuma_events(uint32_t events);

    void complete_ops();
    void complete_ops(struct io_uring_cqe *cqes, int cnt);
    template<typename SQE_OP>
    Result submit_op(SQE_OP &&sqe_op);

private:
    int             uring_fd_;

    struct app_io_sq_ring {
        unsigned *head;
        unsigned *tail;
        unsigned *ring_mask;
        unsigned *ring_entries;
        unsigned *flags;
        unsigned *dropped;
        unsigned *array;
        unsigned ring_sz;
        unsigned entries;
        void *ring_ptr;
    };

    struct app_io_cq_ring {
        unsigned *head;
        unsigned *tail;
        unsigned *ring_mask;
        unsigned *ring_entries;
        struct io_uring_cqe *cqes;
        unsigned ring_sz;
        unsigned entries;
        void *ring_ptr;
    };

    struct app_io_sq_ring sq_ring_;
    struct io_uring_sqe*  sqes_;
    struct app_io_cq_ring cq_ring_;

    struct timespec to_val_;

    EventNotifier notifier_;
    OpData op_data_;
};


namespace {

inline int __io_uring_setup(unsigned entries, struct io_uring_params *p)
{
    return (int) syscall(__NR_io_uring_setup, entries, p);
}

inline int __io_uring_enter(int fd, unsigned to_submit,
                     unsigned min_complete, unsigned flags,
                     sigset_t *sig, size_t sz)
{
    return (int) syscall(__NR_io_uring_enter, fd, to_submit,
                         min_complete, flags, sig, sz);
}

inline int __io_uring_register(int fd, unsigned opcode,
                        void *arg, unsigned nr_args)
{
    return (int) syscall(__NR_io_uring_register, fd, opcode,
                         arg, nr_args);
}

} // namespace {

static uint8_t to_ioring_opcode(OpCode op);

IOUring::IOUring()
: uring_fd_(INVALID_FD)
{
    op_data_.fd = notifier_.getReadFD();
    op_data_.context = &notifier_;
    op_data_.handler = [] (SOCKET_FD, int res, void* ctx) {
        auto * notifier = static_cast<EventNotifier*>(ctx);
        notifier->onEvent(kEventRead);
    };
}

IOUring::~IOUring()
{
    if(INVALID_FD != uring_fd_) {
        if (cq_ring_.ring_ptr && cq_ring_.ring_ptr != sq_ring_.ring_ptr) {
            munmap(cq_ring_.ring_ptr, cq_ring_.ring_sz);
            cq_ring_.ring_ptr = nullptr;
        }
        if (sqes_) {
            munmap(sqes_, sq_ring_.entries * sizeof(struct io_uring_sqe));
            sqes_ = nullptr;
        }
        if (sq_ring_.ring_ptr) {
            munmap(sq_ring_.ring_ptr, sq_ring_.ring_sz);
            sq_ring_.ring_ptr = nullptr;
        }
        close(uring_fd_);
        uring_fd_ = INVALID_FD;
    }
}

bool IOUring::init()
{
    if(INVALID_FD != uring_fd_) {
        return true;
    }
    struct io_uring_params p;
    memset(&p, 0, sizeof(p));
    uring_fd_ = __io_uring_setup(QUEUE_DEPTH, &p);
    if(uring_fd_ < 0) {
        KM_ERRTRACE("IOUring::init, io_uring_setup failed： " << uring_fd_);
        return false;
    }
    int sqring_sz = p.sq_off.array + p.sq_entries * sizeof(unsigned);
    int cqring_sz = p.cq_off.cqes + p.cq_entries * sizeof(struct io_uring_cqe);
    if (p.features & IORING_FEAT_SINGLE_MMAP) {
        if (cqring_sz > sqring_sz) {
            sqring_sz = cqring_sz;
        }
        cqring_sz = sqring_sz;
    }
    struct io_uring_sqe *sqes;
    struct io_uring_cqe *cqes;
    uint8_t *sq_ptr, *cq_ptr;
    sq_ptr = (uint8_t*)mmap(0, sqring_sz, PROT_READ | PROT_WRITE,
                MAP_SHARED | MAP_POPULATE,
                uring_fd_, IORING_OFF_SQ_RING);
    if (sq_ptr == MAP_FAILED) {
        KM_ERRTRACE("IOUring::init, mmap failed 1");
        return false;
    }
    if (p.features & IORING_FEAT_SINGLE_MMAP) {
        cq_ptr = sq_ptr;
    } else {
        cq_ptr = (uint8_t*)mmap(0, cqring_sz, PROT_READ | PROT_WRITE, 
                MAP_SHARED | MAP_POPULATE,
                uring_fd_, IORING_OFF_CQ_RING);
        if (cq_ptr == MAP_FAILED) {
            KM_ERRTRACE("IOUring::init, mmap failed 2");
            return false;
        }
    }
    sqes_ = (io_uring_sqe*)mmap(0, p.sq_entries * sizeof(struct io_uring_sqe),
                PROT_READ | PROT_WRITE, MAP_SHARED | MAP_POPULATE,
                uring_fd_, IORING_OFF_SQES);
    if (sqes_ == MAP_FAILED) {
        KM_ERRTRACE("IOUring::init, mmap failed 3");
        return false;
    }

    sq_ring_.head = (unsigned*)(sq_ptr + p.sq_off.head);
    sq_ring_.tail = (unsigned*)(sq_ptr + p.sq_off.tail);
    sq_ring_.ring_mask = (unsigned*)(sq_ptr + p.sq_off.ring_mask);
    sq_ring_.ring_entries = (unsigned*)(sq_ptr + p.sq_off.ring_entries);
    sq_ring_.flags = (unsigned*)(sq_ptr + p.sq_off.flags);
    sq_ring_.dropped = (unsigned*)(sq_ptr + p.sq_off.dropped);
    sq_ring_.array = (unsigned*)(sq_ptr + p.sq_off.array);
    sq_ring_.ring_sz = sqring_sz;
    sq_ring_.entries = p.sq_entries;
    sq_ring_.ring_ptr = sq_ptr;

    cq_ring_.head = (unsigned*)(cq_ptr + p.cq_off.head);
    cq_ring_.tail = (unsigned*)(cq_ptr + p.cq_off.tail);
    cq_ring_.ring_mask = (unsigned*)(cq_ptr + p.cq_off.ring_mask);
    cq_ring_.ring_entries = (unsigned*)(cq_ptr + p.cq_off.ring_entries);
    cq_ring_.cqes = (io_uring_cqe*)(cq_ptr + p.cq_off.cqes);
    cq_ring_.ring_sz = cqring_sz;
    cq_ring_.entries = p.cq_entries;
    cq_ring_.ring_ptr = cq_ptr;

    return true;
}

uint32_t IOUring::get_events(KMEvent kuma_events)
{
    uint32_t ev = 0;
    return ev;
}

KMEvent IOUring::get_kuma_events(uint32_t events)
{
    KMEvent ev = 0;
    return ev;
}

Result IOUring::registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb)
{
    if (fd < 0) {
        return Result::INVALID_PARAM;
    }
    resizePollItems(fd);
    return Result::OK;
}

Result IOUring::unregisterFd(SOCKET_FD fd)
{
    int max_fd = int(poll_items_.size() - 1);
    KM_INFOTRACE("IOUring::unregisterFd, fd="<<fd<<", max_fd="<<max_fd);
    if (fd < 0 || fd > max_fd) {
        KM_WARNTRACE("IOUring::unregisterFd, failed, max_fd=" << max_fd);
        return Result::INVALID_PARAM;
    }
    if(fd < max_fd) {
        poll_items_[fd].reset();
    } else if (fd == max_fd) {
        poll_items_.pop_back();
    }
    return Result::OK;
}

Result IOUring::updateFd(SOCKET_FD fd, KMEvent events)
{
    if(fd < 0 || fd >= poll_items_.size() || INVALID_FD == poll_items_[fd].fd) {
        return Result::FAILED;
    }
    poll_items_[fd].events = events;
    return Result::OK;
}

Result IOUring::submitOp(SOCKET_FD fd, const Op &op)
{
    if (op.oc == OpCode::REGISTER || op.oc == OpCode::UNREGISTER) {
        return Result::OK;
    }
    return submit_op([&](io_uring_sqe *sqe) {
        sqe->fd = fd;
        sqe->flags = 0;
        sqe->msg_flags = op.flags;
        sqe->user_data = (__u64)op.data;

        switch (op.oc)
        {
        case OpCode::CONNECT:
            sqe->opcode = SYS_IORING_OP_CONNECT;
            sqe->addr = (__u64)op.addr;
            sqe->off = op.addrlen;
            if (op.data) op.data->fd = fd;
            return Result::OK;
        case OpCode::ACCEPT:
            sqe->opcode = SYS_IORING_OP_ACCEPT;
            sqe->addr = (__u64)op.addr;
            //sqe->addr2 = (__u64)op.addr2;
            if (op.data) op.data->fd = fd;
            return Result::OK;
        case OpCode::READV:
        case OpCode::WRITEV:
            sqe->opcode = to_ioring_opcode(op.oc);
            sqe->addr = (unsigned long)op.iovs;
            sqe->len = op.count;
            if (op.data) op.data->fd = fd;
            return Result::OK;
        case OpCode::SEND:
        case OpCode::RECV:
        case OpCode::SENDMSG:
        case OpCode::RECVMSG:
            sqe->opcode = to_ioring_opcode(op.oc);
            sqe->addr = (unsigned long)op.buf;
            sqe->len = op.buflen;
            if (op.data) op.data->fd = fd;
            return Result::OK;
        case OpCode::CANCEL:
            sqe->opcode = SYS_IORING_OP_ASYNC_CANCEL;
            sqe->addr = (__u64)op.data;
            sqe->user_data = 0;
            if (op.data) op.data->fd = fd;
            return Result::OK;
        
        default:
            return Result::INVALID_OPERATION;
        }
    });
}

void IOUring::complete_ops()
{
    unsigned ring_mask = *cq_ring_.ring_mask;
    unsigned head = *cq_ring_.head;
    unsigned tail;
    do {
        struct io_uring_cqe cqes[32];
        int cnt = 0;
        do {
            // read barrier
            //tail = *cq_ring_.tail;
            //std::atomic_thread_fence(std::memory_order_acquire);
            tail = io_uring_smp_load_acquire(cq_ring_.tail);
            if (head == tail) {
                break;
            }
            unsigned index = head & ring_mask;
            auto *cqe = &cq_ring_.cqes[index];
            cqes[cnt++] = *cqe;
            ++head;
        } while (cnt < ARRAY_SIZE(cqes));
        io_uring_smp_store_release(cq_ring_.head, head);
        complete_ops(cqes, cnt);
        //*cq_ring_.head = head;
        // write barrier
        //std::atomic_thread_fence(std::memory_order_release);
    } while(head != tail);
}

void IOUring::complete_ops(struct io_uring_cqe *cqes, int cnt)
{
    for (int i=0; i < cnt; ++i) {
        auto *data = (OpData*)cqes[i].user_data;
        if (data && data->handler) {
            data->handler(data->fd, cqes[i].res, data->context);
        }
    }
}

template<typename SQE_OP>
Result IOUring::submit_op(SQE_OP &&sqe_op)
{
    unsigned tail, index, ring_mask;
    ring_mask = *sq_ring_.ring_mask;
    tail = *sq_ring_.tail;
    index = tail & ring_mask;
    auto* sqe = &sqes_[index];
    memset(sqe, 0, sizeof(*sqe));
    auto ret = sqe_op(sqe);
    if (ret != Result::OK) {
        return ret;
    }
    sq_ring_.array[index] = index;
    ++tail;
    io_uring_smp_store_release(sq_ring_.tail, tail);
    /*
    //write_barrier();
    std::atomic_thread_fence(std::memory_order_release);
    *sq_ring_.tail = tail;
    //write_barrier();
    std::atomic_thread_fence(std::memory_order_release);*/
    return ret;
}

Result IOUring::wait(uint32_t wait_ms)
{
    struct timespec *pts = nullptr;
    if(wait_ms != -1) {
        to_val_.tv_sec = wait_ms/1000;
        to_val_.tv_nsec = (wait_ms - to_val_.tv_sec*1000)*1000*1000;
        pts = &to_val_;
    }
    submit_op([&](io_uring_sqe *sqe) {
        sqe->opcode = IORING_OP_TIMEOUT;
        sqe->fd = -1;
        sqe->off = 1; // count
        sqe->addr = (unsigned long)pts;
        sqe->len = pts ? 1 : 0;
        return Result::OK;
    });
    submit_op([&](io_uring_sqe *sqe) {
        sqe->opcode = IORING_OP_POLL_ADD;
        sqe->fd = notifier_.getReadFD();
        sqe->user_data = (__u64)&op_data_;
        sqe->poll_events = POLLIN | POLLPRI | POLLERR;
        return Result::OK;
    });
    unsigned flags = IORING_ENTER_GETEVENTS;
    int ret = __io_uring_enter(uring_fd_, 1, 1, flags, nullptr, 0);
    if (ret >= 0) {
        complete_ops();
    }
    return Result::OK;
}

void IOUring::notify()
{
    notifier_.notify();
}

static uint8_t to_ioring_opcode(OpCode op)
{
    switch (op)
    {
    case OpCode::CONNECT:
        return SYS_IORING_OP_CONNECT;
    case OpCode::ACCEPT:
        return SYS_IORING_OP_ACCEPT;
    case OpCode::SEND:
        return SYS_IORING_OP_SEND;
    case OpCode::RECV:
        return SYS_IORING_OP_RECV;
    case OpCode::READV:
        return SYS_IORING_OP_READV;
    case OpCode::WRITEV:
        return SYS_IORING_OP_WRITEV;
    case OpCode::SENDMSG:
        return SYS_IORING_OP_SENDMSG;
    case OpCode::RECVMSG:
        return SYS_IORING_OP_RECVMSG;
    case OpCode::CANCEL:
        return SYS_IORING_OP_ASYNC_CANCEL;
    
    default:
        return SYS_IORING_OP_NOP;
    }
} 

IOPoll* createIOUring() {
    struct utsname utsn;
    if (::uname(&utsn) == 0) {
        KM_INFOTRACE("kernal version: " << utsn.release);
        if (strcmp(utsn.release, "5.10.0") > 0) {
            return new IOUring();
        }
    }
    return nullptr;
}

KEV_NS_END
