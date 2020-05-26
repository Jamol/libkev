# libkev
libkev is a cross-platform C++(C++14) event loop from project [kuma](https://github.com/Jamol/kuma), the supported platforms include windows/mac/iOS/linux/android

## build
CMake is the supported build system

#### requirements
- A conforming C++14 compiler
- CMake v3.5 or newer

build command: (replace your_os to one of windows/mac/ios/linux/android)
```
$ python ./bld/your_os/build_your_os.py
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

    std::thread thr([&] {
        if (run_loop.init()) {
            run_loop.loop();
        }
    });
    
    Timer timer(&run_loop);
    timer.schedule(3000, Timer::Mode::ONE_SHOT, [&] {
        printf("onTimer\n");

        run_loop.stop();
    });
    
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