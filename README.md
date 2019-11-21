# libkev
an event loop from project kuma

## example
```
#include "kev.h"

#include <thread>

using namespace kuma;

void foo()
{
    printf("foo called\n");
}

int bar()
{
    printf("bar called\n");
    return 333;
}

int main(int argc, const char * argv[])
{
    EventLoop run_loop;

    std::thread thr([&] {
        if (run_loop.init()) {
            run_loop.loop();
        }
    });
    
    Timer timer(&run_loop);
    timer.schedule(2000, TimerMode::ONE_SHOT, [&] {
        printf("onTimer\n");

        run_loop.stop();
    });
    
    run_loop.async([] { printf ("loop async\n"); });
    printf("async called\n");
    
    auto ret = run_loop.invoke([] { printf ("loop invoke\n"); return 88; });
    printf("ret=%d\n", ret);
    
    run_loop.post(foo);
    
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