# libkev
libkev is a cross-platform C++(C++14) event loop that supports epoll/iocp/kqueue/io_uring/poll/select on platform Linux/Windows/mac/iOS/Android.

## build
CMake is the supported build system

#### requirements
- A conforming C++14 compiler
- CMake v3.8 or newer

build command: (replace your_os to one of windows/mac/ios/linux/android)
```
$ python ./bld/your_os/build_your_os.py
```
build with specified msvc on windows, default is vs2017
```
$ python ./bld/windows/build_windows.py --msvc vs2017/vs2019/vs2022
```

## example
```
#include "kev.h"

#include <thread>
#include <memory>

using namespace kev;

void foo()
{
    printf("foo called\n");
}

void foo2()
{
    printf("foo2 called\n");
}

int bar()
{
    printf("bar called\n");
    return 333;
}

int main(int argc, const char * argv[])
{
    EventLoop run_loop;
    EventLoop::Token token = run_loop.createToken();

    run_loop.start();
    std::thread thr([&] {
        run_loop.loop();
    });
    
    Timer timer(&run_loop);
    timer.schedule(3000, Timer::Mode::ONE_SHOT, [&] {
        printf("onTimer\n");

        run_loop.stop();
    });

    std::string test_str;
    auto timer3 = std::make_shared<Timer>(&run_loop);
    timer3->schedule(600, Timer::Mode::REPEATING, [=]() mutable {
        printf("onTimer, timer3 cancelling\n");
        timer3->cancel();
        test_str = "timer3 was cancelled from callback";
        printf("%s\n", test_str.c_str());
    });
    timer3.reset();
    
    auto delayed_token = run_loop.createToken();
    run_loop.postDelayed(1000,[] {
        printf("postDelayed 1000\n");
    }, &delayed_token);
    
    run_loop.postDelayed(1500,[] {
        printf("postDelayed 1500\n");
    });
    
    run_loop.postDelayed(5000,[] {
        printf("postDelayed 5000\n");
    });
    
    delayed_token.reset();
    
    run_loop.async([] { printf ("loop async\n"); }, &token);
    printf("async called\n");
    
    auto ret = run_loop.invoke([] { printf ("loop invoke\n"); return 88; });
    printf("ret=%d\n", ret);
    
    auto int_ptr = std::make_unique<int>(123);
    
    ret = run_loop.invoke([p=std::move(int_ptr)] {
        return *p;
    });
    printf("move-only ret=%d\n", ret);
    
    int_ptr = std::make_unique<int>(456);
    run_loop.sync([p=std::move(int_ptr)] {
        printf("sync: move-only, i=%d\n", *p);
    });
    
    int_ptr = std::make_unique<int>(789);
    run_loop.async([p=std::move(int_ptr)] {
        printf("async: move-only, i=%d\n", *p);
    }, &token);
    
    run_loop.post(foo);
    int ddc = 81;
    run_loop.post([=] {
        printf("ddc=%d\n", ddc);
        foo2();
    }, &token);
    
    //token.reset();
    
    ret = run_loop.invoke(bar);
    printf("ret=%d\n", ret);
    
    run_loop.invoke(foo);

    try {
        if (thr.joinable()) {
            thr.join();
        }
    } catch(...) {}

    return 0;
}

```