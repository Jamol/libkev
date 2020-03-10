#ifndef __KEV_ImplHelper_h__
#define __KEV_ImplHelper_h__

#include <string>

namespace kev {

template <typename Impl>
struct ImplHelper {
    using ImplPtr = std::shared_ptr<Impl>;
    Impl impl;
    ImplPtr ptr;

    template <typename... Args>
    ImplHelper(Args&... args)
        : impl(args...)
    {
        ptr.reset(&impl, [](Impl* p) {
            auto h = reinterpret_cast<ImplHelper*>(p);
            delete h;
        });
    }

    template <typename... Args>
    ImplHelper(Args&&... args)
        : impl(std::forward<Args...>(args)...)
    {
        ptr.reset(&impl, [](Impl* p) {
            auto h = reinterpret_cast<ImplHelper*>(p);
            delete h;
        });
    }
    
    template <typename... Args>
    static Impl* create(Args&... args)
    {
        auto *ih = new ImplHelper(args...);
        return &ih->impl;
    }
    
    template <typename... Args>
    static Impl* create(Args&&... args)
    {
        auto *ih = new ImplHelper(std::forward<Args...>(args)...);
        return &ih->impl;
    }
    
    static void destroy(Impl *pimpl)
    {
        if (pimpl) {
            auto *ih = reinterpret_cast<ImplHelper*>(pimpl);
            ih->ptr.reset();
        }
    }
    
    static ImplPtr implPtr(Impl *pimpl)
    {
        if (pimpl) {
            auto h = reinterpret_cast<ImplHelper*>(pimpl);
            return h->ptr;
        }
        return ImplPtr();
    }

private:
    ~ImplHelper() = default;
};

} // namespace kev

#endif // __KEV_ImplHelper_h__
