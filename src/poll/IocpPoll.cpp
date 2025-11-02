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

#include "IOPoll.h"
#include "utils/kmtrace.h"
#include "utils/utils.h"
#include "utils/skutils.h"

#ifdef KUMA_OS_WIN
# include <MSWSock.h>
#endif

KEV_NS_BEGIN

extern LPFN_CONNECTEX connect_ex;
extern LPFN_ACCEPTEX accept_ex;
extern LPFN_CANCELIOEX cancel_io_ex;
extern LPFN_TRANSMITFILE transmit_file;
extern LPFN_WSASENDMSG wsa_sendmsg;
extern LPFN_WSARECVMSG wsa_recvmsg;

class IocpPoll : public IOPoll, public IOPollItem<PollItem>
{
public:
    IocpPoll();
    ~IocpPoll();
    
    bool init() override;
    Result registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb) override;
    Result unregisterFd(SOCKET_FD fd) override;
    Result updateFd(SOCKET_FD fd, KMEvent events) override;
    Result wait(uint32_t wait_ms) override;
    void notify() override;
    PollType getType() const override { return PollType::IOCP; }
    bool isLevelTriggered() const override { return false; }

    Result submitOp(SOCKET_FD fd, const Op &op) override;
    
protected:
    Result submitConnectOp(SOCKET_FD fd, const Op &op);
    Result submitAcceptOp(SOCKET_FD fd, const Op &op);
    Result submitReadvOp(SOCKET_FD fd, const Op &op);
    Result submitWritevOp(SOCKET_FD fd, const Op &op);
    Result submitSendmsgOp(SOCKET_FD fd, const Op &op);
    Result submitRecvmsgOp(SOCKET_FD fd, const Op &op);
    
    // Common error handling for async operations
    Result handleAsyncResult(BOOL result, SOCKET_FD fd, const char* operation);

protected:
    HANDLE hCompPort_ { nullptr };
};

IocpPoll::IocpPoll()
{
    
}

IocpPoll::~IocpPoll()
{
    if (hCompPort_) {
        CloseHandle(hCompPort_);
        hCompPort_ = nullptr;
    }
}

bool IocpPoll::init()
{
    if (hCompPort_) {
        return true;
    }
    hCompPort_ = CreateIoCompletionPort(INVALID_HANDLE_VALUE, NULL, 0, 1);
    if (!hCompPort_) {
        KM_ERRTRACE("IocpPoll::init, CreateIoCompletionPort failed, err=" << GetLastError());
        return false;
    }
    return true;
}

Result IocpPoll::registerFd(SOCKET_FD fd, KMEvent events, IOCallback cb)
{
    KM_INFOTRACE("IocpPoll::registerFd, fd=" << fd << ", events=" << events);
    if (CreateIoCompletionPort((HANDLE)fd, hCompPort_, (ULONG_PTR)fd, 0) == NULL) {
        KM_ERRTRACE("IocpPoll::registerFd, CreateIoCompletionPort failed, err=" << GetLastError());
        return Result::POLL_ERROR;
    }
    auto *poll_item = getPollItem(fd, true);
    if (!poll_item) {
        KM_ERRTRACE("IocpPoll::registerFd no poll item, fd=" << fd << ", sz=" << getPollItemSize());
        return Result::BUFFER_TOO_SMALL;
    }
    poll_item->fd = fd;
    poll_item->cb = std::move(cb);
    return Result::OK;
}

Result IocpPoll::unregisterFd(SOCKET_FD fd)
{
    KM_INFOTRACE("IocpPoll::unregisterFd, fd="<<fd);
    clearPollItem(fd);
    
    return Result::OK;
}

Result IocpPoll::updateFd(SOCKET_FD fd, KMEvent events)
{
    // IOCP doesn't support dynamic event updates like epoll/kqueue
    // Events are managed through individual async operations
    return Result::NOT_SUPPORTED;
}

Result IocpPoll::handleAsyncResult(BOOL result, SOCKET_FD fd, const char* operation)
{
    if (!result) {
        DWORD error = SKUtils::getLastError();
        if (error == WSA_IO_PENDING) {
            return Result::OK; // Operation is pending, which is expected
        }
        KM_ERRTRACE("submitOp, " << operation << " error, fd=" << fd << ", err=" << error);
        return Result::SOCK_ERROR;
    }
    
    // Operation completed immediately, continue to wait for completion notification
    // or set FILE_SKIP_COMPLETION_PORT_ON_SUCCESS
    // SetFileCompletionNotificationModes
    return Result::OK;
}

Result IocpPoll::submitConnectOp(SOCKET_FD fd, const Op &op)
{
    if (!connect_ex) {
        return Result::NOT_SUPPORTED;
    }
    if (!op.data) {
        return Result::INVALID_PARAM;
    }
    
    memset(&op.data->ol, 0, sizeof(op.data->ol));
    auto ret = connect_ex(fd, op.addr, op.addrlen, NULL, 0, NULL, (LPOVERLAPPED)op.data);
    return handleAsyncResult(ret, fd, "connect");
}

Result IocpPoll::submitAcceptOp(SOCKET_FD fd, const Op &op)
{
    if (!accept_ex) {
        return Result::NOT_SUPPORTED;
    }
    if (!op.data) {
        return Result::INVALID_PARAM;
    }
    
    memset(&op.data->ol, 0, sizeof(op.data->ol));
    DWORD bytes_recv = 0;
    auto ret = accept_ex(fd, op.data->fd, op.addr, 0, op.addrlen, op.addrlen, &bytes_recv, (LPOVERLAPPED)op.data);
    return handleAsyncResult(ret, fd, "accept");
}

Result IocpPoll::submitReadvOp(SOCKET_FD fd, const Op &op)
{
    if (!op.data) {
        return Result::INVALID_PARAM;
    }
    
    memset(&op.data->ol, 0, sizeof(op.data->ol));
    DWORD flags = op.flags, bytes_recv = 0;
    auto ret = WSARecv(fd, (LPWSABUF)op.iovs, op.count, &bytes_recv, &flags, (LPOVERLAPPED)op.data, NULL);
    return handleAsyncResult(ret != SOCKET_ERROR, fd, "readv");
}

Result IocpPoll::submitWritevOp(SOCKET_FD fd, const Op &op)
{
    if (!op.data) {
        return Result::INVALID_PARAM;
    }
    
    memset(&op.data->ol, 0, sizeof(op.data->ol));
    DWORD bytes_sent = 0;
    auto ret = WSASend(fd, (LPWSABUF)op.iovs, op.count, &bytes_sent, op.flags, (LPOVERLAPPED)op.data, NULL);
    return handleAsyncResult(ret != SOCKET_ERROR, fd, "writev");
}

Result IocpPoll::submitSendmsgOp(SOCKET_FD fd, const Op &op)
{
    if (!wsa_sendmsg) {
        return Result::NOT_SUPPORTED;
    }
    if (!op.data) {
        return Result::INVALID_PARAM;
    }
    
    memset(&op.data->ol, 0, sizeof(op.data->ol));
    DWORD bytes_sent = 0;
    auto ret = wsa_sendmsg(fd, (LPWSAMSG)op.buf, op.flags, &bytes_sent, (LPOVERLAPPED)op.data, NULL);
    return handleAsyncResult(ret != SOCKET_ERROR, fd, "sendmsg");
}

Result IocpPoll::submitRecvmsgOp(SOCKET_FD fd, const Op &op)
{
    if (!wsa_recvmsg) {
        return Result::NOT_SUPPORTED;
    }
    if (!op.data) {
        return Result::INVALID_PARAM;
    }
    
    memset(&op.data->ol, 0, sizeof(op.data->ol));
    DWORD bytes_recv = 0;
    auto ret = wsa_recvmsg(fd, (LPWSAMSG)op.buf, &bytes_recv, (LPOVERLAPPED)op.data, NULL);
    return handleAsyncResult(ret != SOCKET_ERROR, fd, "recvmsg");
}

Result IocpPoll::submitOp(SOCKET_FD fd, const Op &op)
{
    switch (op.oc)
    {
    case OpCode::REGISTER: {
        if (CreateIoCompletionPort((HANDLE)fd, hCompPort_, (ULONG_PTR)fd, 0) == NULL) {
            return Result::POLL_ERROR;
        }
        return Result::OK;
    }
    case OpCode::UNREGISTER: {
        return Result::OK;
    }
    case OpCode::CONNECT: {
        return submitConnectOp(fd, op);
    }
    case OpCode::ACCEPT: {
        return submitAcceptOp(fd, op);
    }
    case OpCode::READV: {
        return submitReadvOp(fd, op);
    }
    case OpCode::WRITEV: {
        return submitWritevOp(fd, op);
    }
    case OpCode::SENDMSG: {
        return submitSendmsgOp(fd, op);
    }
    case OpCode::RECVMSG: {
        return submitRecvmsgOp(fd, op);
    }
    case OpCode::CANCEL: {
        BOOL ret = cancel_io_ex ?
                   cancel_io_ex(reinterpret_cast<HANDLE>(fd), nullptr) :
                   CancelIo(reinterpret_cast<HANDLE>(fd));
        return ret ? Result::OK : Result::FAILED;
    }
    
    default:
        return Result::INVALID_OPERATION;
    }
}

#if 1
Result IocpPoll::wait(uint32_t wait_ms)
{
    OVERLAPPED_ENTRY entries[128];
    ULONG count = 0;
    auto success = GetQueuedCompletionStatusEx(hCompPort_, entries, ARRAY_SIZE(entries), &count, wait_ms, FALSE);
    if (success) {
        for (ULONG i = 0; i < count; ++i) {
            if (entries[i].lpOverlapped) {
                SOCKET_FD fd = (SOCKET_FD)entries[i].lpCompletionKey;
#if 0
                auto *poll_item = getPollItem(fd);
                if (poll_item) {
                    IOCallback &cb = poll_item->cb;
                    size_t io_size = entries[i].dwNumberOfBytesTransferred;
                    if (cb) cb(fd, 0, entries[i].lpOverlapped, io_size);
                }
#else
                auto *data = (OpData*)entries[i].lpOverlapped;
                if (data->handler) {
                    int io_size = (int)entries[i].dwNumberOfBytesTransferred;
                    data->handler(fd, io_size, data->context);
                }
#endif
            }
        }
    }
    else {
        auto err = ::GetLastError();
        if (err != WAIT_TIMEOUT) {
            KM_ERRTRACE("IocpPoll::wait, err="<<err);
        }
    }
    return Result::OK;
}
#else
Result IocpPoll::wait(uint32_t wait_ms)
{
    DWORD bytes;
    ULONG_PTR key;
    OVERLAPPED *pOverlapped = NULL;
    auto success = GetQueuedCompletionStatus(hCompPort_, &bytes, &key, &pOverlapped, wait_ms);
    if (success) {
        SOCKET_FD fd = (SOCKET_FD)key;
        auto *poll_item = getPollItem(fd);
        if (poll_item) {
            IOCallback &cb = poll_item->cb;
            size_t io_size = bytes;
            if (cb) cb(fd, 0, pOverlapped, io_size);
        }
    }
    else {
        auto err = ::GetLastError();
        if (NULL == pOverlapped) { // GetQueuedCompletionStatus() failed
            if (err != WAIT_TIMEOUT) {
                KM_ERRTRACE("IocpPoll::wait, err="<<err);
            }
        } else { // async IO failed
            if (key) {
                SOCKET_FD fd = (SOCKET_FD)key;
            }
        }
    }
    return Result::OK;
}
#endif

void IocpPoll::notify()
{
    if (hCompPort_ != nullptr) {
        PostQueuedCompletionStatus(hCompPort_, 0, 0, NULL);
    }
}

IOPoll* createIocpPoll() {
    return new IocpPoll();
}

KEV_NS_END
