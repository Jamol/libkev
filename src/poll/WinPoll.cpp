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

#include "IOPoll.h"
#include "utils/kmtrace.h"

KEV_NS_BEGIN

#define WM_SOCKET_NOTIFY		0x0373

#define WM_POLLER_NOTIFY		WM_USER+101

#define KEV_WIN_CLASS_NAME		L"kev_win_class_name"

class WinPoll : public IOPoll, public IOPollItem<PollItem>
{
public:
    WinPoll();
    ~WinPoll();
    
    bool init();
    Result registerFd(SOCKET_FD fd, uint32_t events, IOCallback cb);
    Result unregisterFd(SOCKET_FD fd);
    Result updateFd(SOCKET_FD fd, uint32_t events);
    Result wait(uint32_t wait_ms);
    void notify();
    PollType getType() const { return PollType::WINEV; }
    bool isLevelTriggered() const { return false; }

public:
    void on_socket_notify(SOCKET_FD fd, uint32_t events);

    void on_poller_notify();
    
private:
    uint32_t get_events(uint32_t kuma_events) const;
    uint32_t get_kuma_events(uint32_t events) const;

private:
    HWND            hwnd_;
};

WinPoll::WinPoll()
: hwnd_(NULL)
{
    
}

WinPoll::~WinPoll()
{
    if (hwnd_) {
        if (::IsWindow(hwnd_)) {
            DestroyWindow(hwnd_);
        }
        hwnd_ = NULL;
    }
}

bool WinPoll::init()
{
    hwnd_ = ::CreateWindow(KM_WIN_CLASS_NAME, NULL, WS_OVERLAPPED, 0,
                                   0, 0, 0, NULL, NULL, NULL, 0);
    if (NULL == hwnd_) {
        return false;
    }
    SetWindowLong(hwnd_, 0, (LONG)this);
    return true;
}

uint32_t WinPoll::get_events(uint32_t kuma_events) const
{
    uint32_t ev = 0;
    if(kuma_events & kEventRead) {
        ev |= FD_READ;
    }
    if(kuma_events & kEventWrite) {
        ev |= FD_WRITE;
    }
    if(kuma_events & kEventError) {
        ev |= FD_CLOSE;
    }
    return ev;
}

uint32_t WinPoll::get_kuma_events(uint32_t events) const
{
    uint32_t ev = 0;
    if (events & FD_CONNECT) { // writeable
        ev |= kEventWrite;
    }
    if (events & FD_ACCEPT) { // writeable
        ev |= kEventRead;
    }
    if(events & FD_READ) {
        ev |= kEventRead;
    }
    if(events & FD_WRITE) {
        ev |= kEventWrite;
    }
    if(events & FD_CLOSE) {
        ev |= kEventError;
    }
    return ev;
}

Result WinPoll::registerFd(SOCKET_FD fd, uint32_t events, IOCallback cb)
{
    KLOGI("WinPoll::registerFd, fd=" << fd << ", events=" << events);
    auto *poll_item = getPollItem(fd, true);
    if (!poll_item) {
        KLOGE("WinPoll::registerFd no poll item, fd=" << fd << ", sz=" << getPollItemSize());
        return Result::BUFFER_TOO_SMALL;
    }
    poll_item->fd = fd;
    poll_item->cb = std::move(cb);
    WSAAsyncSelect(fd, hwnd_, WM_SOCKET_NOTIFY, get_events(events) | FD_CONNECT);
    return Result::OK;
}

Result WinPoll::unregisterFd(SOCKET_FD fd)
{
    KLOGI("WinPoll::unregisterFd, fd="<<fd);
    clearPollItem(fd);
    WSAAsyncSelect(fd, hwnd_, 0, 0);
    return Result::OK;
}

Result WinPoll::updateFd(SOCKET_FD fd, uint32_t events)
{
    auto *poll_item = getPollItem(fd);
    if (!poll_item || INVALID_FD == poll_item->fd) {
        return Result::INVALID_PARAM;
    }
    if(poll_item->fd != fd) {
        KLOGW("WinPoll::updateFd, failed, fd="<<fd<<", fd1="<<poll_item->fd);
        return Result::INVALID_PARAM;
    }
    return Result::OK;
}

Result WinPoll::wait(uint32_t wait_ms)
{
    MSG msg;
    if (GetMessage(&msg, NULL, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
    return Result::OK;
}

void WinPoll::notify()
{
    if (hwnd_) {
        ::PostMessage(hwnd_, WM_POLLER_NOTIFY, 0, 0);
    }
}

void WinPoll::on_socket_notify(SOCKET_FD fd, uint32_t events)
{
    int err = WSAGETSELECTERROR(events);
    int evt = WSAGETSELECTEVENT(events);
}

void WinPoll::on_poller_notify()
{

}

IOPoll* createWinPoll() {
    return new WinPoll();
}

//////////////////////////////////////////////////////////////////////////
//
LRESULT CALLBACK km_notify_wnd_proc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    switch(uMsg)
    {
        case WM_SOCKET_NOTIFY:
        {
            WinPoll* poll = (WinPoll*)GetWindowLong(hwnd, 0);
            if (poll)
                poll->on_socket_notify(wParam, lParam);
            return 0L;
        }
        
        case WM_POLLER_NOTIFY:
        {
            WinPoll* poll = (WinPoll*)GetWindowLong(hwnd, 0);
            if (poll)
                poll->on_poller_notify();
            return 0L;
        }
    }
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

static void initWinClass()
{
    WNDCLASS wc = {0};
    wc.style = 0;
    wc.lpfnWndProc = (WNDPROC)km_notify_wnd_proc;
    wc.cbClsExtra = 0;
    wc.cbWndExtra = sizeof(void*);
    wc.hInstance = NULL;
    wc.hIcon = 0;
    wc.hCursor = 0;
    wc.hbrBackground = 0;
    wc.lpszMenuName = NULL;
    wc.lpszClassName = KEV_WIN_CLASS_NAME;
    RegisterClass(&wc);
}

static void uninitWinClass()
{
    UnregisterClass(KEV_WIN_CLASS_NAME, NULL);
}

//WBX_Init_Object g_init_obj(poller_load, poller_unload);

KEV_NS_END
