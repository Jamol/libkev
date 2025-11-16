/* Copyright (c) 2022-2025, Fengping Bao <jamol@live.com>
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

#if defined(KEV_HAS_IOURING)

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

#define QUEUE_DEPTH     256
#define CQE_BATCH_SIZE  32
#define TIMEOUT_FD_VAL  -9527

#define SYS_IORING_OP_NOP           IORING_OP_NOP
#define SYS_IORING_OP_READV         IORING_OP_READV
#define SYS_IORING_OP_WRITEV        IORING_OP_WRITEV
#define SYS_IORING_OP_SENDMSG       IORING_OP_SENDMSG
#define SYS_IORING_OP_RECVMSG       IORING_OP_RECVMSG
#define SYS_IORING_OP_TIMEOUT       IORING_OP_TIMEOUT
#define SYS_IORING_OP_ACCEPT        13 //IORING_OP_ACCEPT
#define SYS_IORING_OP_ASYNC_CANCEL  14 //IORING_OP_ASYNC_CANCEL
#define SYS_IORING_OP_CONNECT       16 //IORING_OP_CONNECT
#define SYS_IORING_OP_CLOSE         IORING_OP_CLOSE
#define SYS_IORING_OP_SEND          26 //IORING_OP_SEND
#define SYS_IORING_OP_RECV          27 //IORING_OP_RECV
#define SYS_IORING_OP_SHUTDOWN      IORING_OP_SHUTDOWN 
#define SYS_IORING_OP_POLL_ADD      IORING_OP_POLL_ADD
#define SYS_IORING_OP_POLL_REMOVE   IORING_OP_POLL_REMOVE

/*
 * POLL_ADD flags. Note that since sqe->poll_events is the flag space, the
 * command flags for POLL_ADD are stored in sqe->len.
 *
 * IORING_POLL_ADD_MULTI	Multishot poll. Sets IORING_CQE_F_MORE if
 *				the poll handler will continue to report
 *				CQEs on behalf of the same SQE.
 *
 * IORING_POLL_UPDATE		Update existing poll request, matching
 *				sqe->addr as the old user_data field.
 *
 * IORING_POLL_LEVEL		Level triggered poll.
 */
#ifndef IORING_POLL_ADD_MULTI
#define IORING_POLL_ADD_MULTI           (1U << 0)
#endif
#ifndef IORING_POLL_UPDATE_EVENTS
#define IORING_POLL_UPDATE_EVENTS       (1U << 1)
#endif
#ifndef IORING_POLL_UPDATE_USER_DATA
#define IORING_POLL_UPDATE_USER_DATA    (1U << 2)
#endif
#ifndef IORING_POLL_ADD_LEVEL
#define IORING_POLL_ADD_LEVEL           (1U << 3)
#endif

/*
 * ASYNC_CANCEL flags.
 *
 * IORING_ASYNC_CANCEL_ALL	Cancel all requests that match the given key
 * IORING_ASYNC_CANCEL_FD	Key off 'fd' for cancelation rather than the
 *				request 'user_data'
 * IORING_ASYNC_CANCEL_ANY	Match any request
 * IORING_ASYNC_CANCEL_FD_FIXED	'fd' passed in is a fixed descriptor
 * IORING_ASYNC_CANCEL_USERDATA	Match on user_data, default for no other key
 * IORING_ASYNC_CANCEL_OP	Match request based on opcode
 */
#ifndef IORING_ASYNC_CANCEL_ALL
#define IORING_ASYNC_CANCEL_ALL         (1U << 0)
#endif
#ifndef IORING_ASYNC_CANCEL_FD
#define IORING_ASYNC_CANCEL_FD          (1U << 1)
#endif
#ifndef IORING_ASYNC_CANCEL_ANY
#define IORING_ASYNC_CANCEL_ANY         (1U << 2)
#endif
#ifndef IORING_ASYNC_CANCEL_FD_FIXED
#define IORING_ASYNC_CANCEL_FD_FIXED    (1U << 3)
#endif
#ifndef IORING_ASYNC_CANCEL_USERDATA
#define IORING_ASYNC_CANCEL_USERDATA    (1U << 4)
#endif
#ifndef IORING_ASYNC_CANCEL_OP
#define IORING_ASYNC_CANCEL_OP          (1U << 5)
#endif

#ifndef IORING_ENTER_EXT_ARG
#define IORING_ENTER_EXT_ARG            (1U << 3)
#endif

/*
 * Argument for io_uring_enter(2) with IORING_GETEVENTS | IORING_ENTER_EXT_ARG
 */
struct io_uring_getevents_arg {
	__u64	sigmask;
	__u32	sigmask_sz;
	__u32	min_wait_usec;
	__u64	ts;
};

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
    PollType getType() const override { return PollType::IORING; }
    bool isLevelTriggered() const override { return false; }

    Result submitOp(SOCKET_FD fd, const Op &op) override;

private:
    void complete_ops();
    void complete_ops(struct io_uring_cqe *cqes, int cnt);
    template<typename SQE_OP>
    Result submit_op(SQE_OP &&sqe_op);
    int submit_sqes();
    int submit_and_wait(uint32_t wait_ms, uint32_t wait_nr);

    void cleanup();

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
    OpData notifier_op_data_;
    //bool timeout_scheduled_ = false;
    //OpData timer_op_data_;
    unsigned int to_submit_ = 0;
};


namespace {

inline int __io_uring_setup(unsigned entries, struct io_uring_params *p)
{
    return (int) syscall(__NR_io_uring_setup, entries, p);
}

inline int __io_uring_enter(int fd, unsigned to_submit,
                     unsigned min_complete, unsigned flags,
                     const void *sig, size_t sz)
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

} // namespace

static uint8_t to_ioring_opcode(OpCode op);

IOUring::IOUring()
: uring_fd_(INVALID_FD)
{
    notifier_op_data_.context = &notifier_;
    notifier_op_data_.handler = [] (SOCKET_FD fd, int res, void* ctx) {
        //KLOGI("IOUring notifier callback, fd=" << fd);
        auto * notifier = static_cast<EventNotifier*>(ctx);
        notifier->onEvent(kEventRead);
    };
    /*timer_op_data_.fd = TIMEOUT_FD_VAL;
    timer_op_data_.context = this;
    timer_op_data_.handler = [] (SOCKET_FD fd, int res, void* ctx) {
        //KLOGI("IOUring timer callback, fd=" << fd);
        auto * _this = static_cast<IOUring*>(ctx);
        _this->timeout_scheduled_ = false;
    };*/
}

IOUring::~IOUring()
{
    cleanup();
}

void IOUring::cleanup()
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
        KLOGE("IOUring::init, io_uring_setup failed: " << uring_fd_);
        return false;
    }
    
    // Calculate ring sizes
    int sqring_sz = p.sq_off.array + p.sq_entries * sizeof(unsigned);
    int cqring_sz = p.cq_off.cqes + p.cq_entries * sizeof(struct io_uring_cqe);
    if (p.features & IORING_FEAT_SINGLE_MMAP) {
        if (cqring_sz > sqring_sz) {
            sqring_sz = cqring_sz;
        }
        cqring_sz = sqring_sz;
    }
    
    // Map submission queue ring
    uint8_t *sq_ptr = (uint8_t*)mmap(0, sqring_sz, PROT_READ | PROT_WRITE,
                MAP_SHARED | MAP_POPULATE,
                uring_fd_, IORING_OFF_SQ_RING);
    if (sq_ptr == MAP_FAILED) {
        KLOGE("IOUring::init, mmap failed for SQ ring");
        cleanup();
        return false;
    }
    
    // Map completion queue ring
    uint8_t *cq_ptr;
    if (p.features & IORING_FEAT_SINGLE_MMAP) {
        cq_ptr = sq_ptr;
    } else {
        cq_ptr = (uint8_t*)mmap(0, cqring_sz, PROT_READ | PROT_WRITE, 
                MAP_SHARED | MAP_POPULATE,
                uring_fd_, IORING_OFF_CQ_RING);
        if (cq_ptr == MAP_FAILED) {
            KLOGE("IOUring::init, mmap failed for CQ ring");
            munmap(sq_ptr, sqring_sz);
            cleanup();
            return false;
        }
    }
    
    // Map submission queue entries
    sqes_ = (io_uring_sqe*)mmap(0, p.sq_entries * sizeof(struct io_uring_sqe),
                PROT_READ | PROT_WRITE, MAP_SHARED | MAP_POPULATE,
                uring_fd_, IORING_OFF_SQES);
    if (sqes_ == MAP_FAILED) {
        KLOGE("IOUring::init, mmap failed for SQEs");
        if (cq_ptr != sq_ptr) {
            munmap(cq_ptr, cqring_sz);
        }
        munmap(sq_ptr, sqring_sz);
        cleanup();
        return false;
    }

    // Setup submission queue ring pointers
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

    // Setup completion queue ring pointers
    cq_ring_.head = (unsigned*)(cq_ptr + p.cq_off.head);
    cq_ring_.tail = (unsigned*)(cq_ptr + p.cq_off.tail);
    cq_ring_.ring_mask = (unsigned*)(cq_ptr + p.cq_off.ring_mask);
    cq_ring_.ring_entries = (unsigned*)(cq_ptr + p.cq_off.ring_entries);
    cq_ring_.cqes = (io_uring_cqe*)(cq_ptr + p.cq_off.cqes);
    cq_ring_.ring_sz = cqring_sz;
    cq_ring_.entries = p.cq_entries;
    cq_ring_.ring_ptr = cq_ptr;

    // Register notifier for wake-up events
    if (!notifier_.ready()) {
        if(!notifier_.init()) {
            KLOGE("IOUring::init, failed to initialize notifier");
            cleanup();
            return false;
        }
        auto efd = notifier_.getReadFD();
        notifier_op_data_.fd = efd;
        //__io_uring_register(uring_fd_, IORING_REGISTER_EVENTFD, &efd, 1);
        submit_op([&](io_uring_sqe *sqe) {
            sqe->opcode = SYS_IORING_OP_POLL_ADD;
            sqe->fd = efd;
            sqe->user_data = (__u64)&notifier_op_data_;
            sqe->poll_events = POLLIN;
            sqe->len = IORING_POLL_ADD_MULTI;
            return Result::OK;
        });
    }
    //timeout_scheduled_ = false;
    return true;
}

Result IOUring::registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb)
{
    KLOGI("IOUring::registerFd, fd=" << fd << ", events=" << events);
    return Result::OK;
}

Result IOUring::unregisterFd(SOCKET_FD fd)
{
    KLOGI("IOUring::unregisterFd, fd="<<fd);
    return Result::OK;
}

Result IOUring::updateFd(SOCKET_FD fd, KMEvent events)
{
    return Result::OK;
}

Result IOUring::submitOp(SOCKET_FD fd, const Op &op)
{
    //KLOGI("submitOp, fd=" << fd << ", oc=" << int(op.oc) << ", len=" << op.count << ", iovs=" << op.iovs);
    if (op.oc == OpCode::REGISTER || op.oc == OpCode::UNREGISTER) {
        return Result::OK;
    }
    return submit_op([&](io_uring_sqe *sqe) {
        sqe->fd = fd;
        sqe->flags = 0;

        switch (op.oc)
        {
        case OpCode::CONNECT:
            sqe->opcode = SYS_IORING_OP_CONNECT;
            sqe->addr = (__u64)op.addr;
            sqe->off = op.addrlen;
            sqe->user_data = (__u64)op.data;
            if (op.data) op.data->fd = fd;
            return Result::OK;
        case OpCode::ACCEPT:
            sqe->opcode = SYS_IORING_OP_ACCEPT;
            sqe->addr = (__u64)op.addr;
            sqe->off/*addr2*/ = (__u64)op.addr2;
            sqe->msg_flags/*accept_flags*/ = op.flags;
            sqe->user_data = (__u64)op.data;
            if (op.data) op.data->fd = fd;
            return Result::OK;
        case OpCode::READV:
        case OpCode::WRITEV:
            sqe->opcode = to_ioring_opcode(op.oc);
            sqe->addr = (__u64)op.iovs;
            sqe->len = op.count;
            sqe->user_data = (__u64)op.data;
            if (op.data) op.data->fd = fd;
            return Result::OK;
        case OpCode::SEND:
        case OpCode::RECV:
        case OpCode::SENDMSG:
        case OpCode::RECVMSG:
            sqe->opcode = to_ioring_opcode(op.oc);
            sqe->addr = (__u64)op.buf;
            sqe->len = op.buflen;
            sqe->msg_flags = op.flags;
            sqe->user_data = (__u64)op.data;
            if (op.data) op.data->fd = fd;
            return Result::OK;
        case OpCode::POLL_ADD:
            sqe->opcode = SYS_IORING_OP_POLL_ADD;
            sqe->user_data = (__u64)op.data;
            sqe->poll_events = op.flags;
            sqe->len = op.buflen;
            return Result::OK;
        case OpCode::POLL_REMOVE:
            sqe->opcode = SYS_IORING_OP_POLL_REMOVE;
            sqe->addr = (__u64)op.data;
            return Result::OK;
        case OpCode::CANCEL:
            sqe->opcode = SYS_IORING_OP_ASYNC_CANCEL;
            sqe->user_data = 0;
            if (op.buf) {
                sqe->addr = (__u64)op.buf;
            } else {
                sqe->msg_flags/*cancel_flags*/ = IORING_ASYNC_CANCEL_FD | IORING_ASYNC_CANCEL_ALL;
            }
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
        struct io_uring_cqe cqes[CQE_BATCH_SIZE];
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
    unsigned tail, head, index, ring_mask;
    ring_mask = *sq_ring_.ring_mask;
    tail = *sq_ring_.tail;
    head = io_uring_smp_load_acquire(sq_ring_.head);
    if (tail - head >= *sq_ring_.ring_entries) {
        if (submit_sqes() < 0) {
            return Result::POLL_ERROR;
        }
        head = io_uring_smp_load_acquire(sq_ring_.head);
        if (tail - head >= *sq_ring_.ring_entries) {
            //flags |= IORING_ENTER_SQ_WAIT; ??
            return Result::AGAIN;
        }
        /*complete_ops();
        head = io_uring_smp_load_acquire(sq_ring_.head);
        if (tail - head >= *sq_ring_.ring_entries) {
            return Result::AGAIN;
        }*/
    }
    index = tail & ring_mask;
    auto* sqe = &sqes_[index];
    memset(sqe, 0, sizeof(*sqe));
    auto ret = sqe_op(sqe);
    if (ret != Result::OK) {
        return ret;
    }
    sq_ring_.array[index] = index;
    ++tail;
    ++to_submit_;
    io_uring_smp_store_release(sq_ring_.tail, tail);
    /*
    //write_barrier();
    std::atomic_thread_fence(std::memory_order_release);
    *sq_ring_.tail = tail;
    //write_barrier();
    std::atomic_thread_fence(std::memory_order_release);*/
    return ret;
}

int IOUring::submit_and_wait(uint32_t wait_ms, uint32_t wait_nr)
{
    unsigned flags = 0;
    size_t argsz = 0;
    void* parg = nullptr;
    if (wait_nr > 0) {
        flags |= IORING_ENTER_GETEVENTS;
    }
#if 0
    if (!timeout_scheduled_) {
        struct timespec *pts = nullptr;
        if(wait_ms != -1 && (wait_nr || wait_ms)) {
            to_val_.tv_sec = wait_ms/1000;
            to_val_.tv_nsec = (wait_ms - to_val_.tv_sec*1000)*1000*1000;
            pts = &to_val_;
        }
        if (pts) {
            submit_op([&](io_uring_sqe *sqe) {
                sqe->opcode = IORING_OP_TIMEOUT;
                sqe->fd = TIMEOUT_FD_VAL;
                sqe->off = 1; // count
                sqe->addr = (unsigned long)pts;
                sqe->len = pts ? 1 : 0;
                sqe->user_data = (__u64)&timer_op_data_;
                timeout_scheduled_ = true;
                return Result::OK;
            });
        }
    }
#else
    struct timespec tso;
    struct io_uring_getevents_arg arg{0, 0, 0, 0};
    if(wait_ms != -1 && (wait_nr || wait_ms)) {
        tso.tv_sec = wait_ms/1000;
        tso.tv_nsec = (wait_ms - tso.tv_sec*1000)*1000*1000;
        arg.ts = (__u64)&tso;
        argsz = sizeof(arg);
        parg = &arg;
        flags |= IORING_ENTER_EXT_ARG;
    }
#endif
    unsigned int to_submit = to_submit_;
    to_submit_ = 0;
    int ret;
    do {
        ret = __io_uring_enter(uring_fd_, to_submit, wait_nr, flags, parg, argsz);
    } while (ret == EINTR);
    return ret;
}

int IOUring::submit_sqes()
{
    return submit_and_wait(0, 0);
}

Result IOUring::wait(uint32_t wait_ms)
{
    int ret = submit_and_wait(wait_ms, 1);
    if (ret >= 0) {
        complete_ops();
        return Result::OK;
    } else {
        return Result::POLL_ERROR;
    }
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
    case OpCode::POLL_ADD:
        return SYS_IORING_OP_POLL_ADD;
    case OpCode::POLL_REMOVE:
        return SYS_IORING_OP_POLL_REMOVE;
    case OpCode::CANCEL:
        return SYS_IORING_OP_ASYNC_CANCEL;
    
    default:
        return SYS_IORING_OP_NOP;
    }
} 

IOPoll* createIOUring() {
    struct utsname utsn;
    if (::uname(&utsn) == 0) {
        //KLOGI("kernal version: " << utsn.release);
        int major = -1, minor = -1;
        for_each_token(utsn.release, '.', [&](const std::string &s) {
            if (major == -1) {
                major = std::stoi(s);
                return true;
            }
            if (minor == -1) {
                minor = std::stoi(s);
                return false;
            }
            return false;
        });
        if (major > 5 || (major == 5 && minor >= 13)) {
            return new IOUring();
        }
    }
    return nullptr;
}

KEV_NS_END

#endif // KEV_HAS_IOURING
