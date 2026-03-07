// 
// wrapper for move-only lambda
// https://stackoverflow.com/questions/25330716/move-only-version-of-stdfunction
//
#ifndef __KEVTYPES_H__
#define __KEVTYPES_H__

#include <type_traits>

namespace kev {

template<typename Fn, typename En = void>
struct lambda_wrapper;

template<typename Fn>
struct lambda_wrapper<Fn, std::enable_if_t< std::is_copy_constructible<Fn>{} >>
{
    Fn fn;

    template<typename... Args>
    auto operator()(Args&&... args) { return fn(std::forward<Args>(args)...); }
};

template<typename Fn>
struct lambda_wrapper<Fn, std::enable_if_t< !std::is_copy_constructible<Fn>{} &&
                                             std::is_move_constructible<Fn>{} >>
{
    Fn fn;

    lambda_wrapper(Fn &&fn) : fn(std::forward<Fn>(fn)) { }

    lambda_wrapper(lambda_wrapper&&) = default;
    lambda_wrapper& operator=(lambda_wrapper&&) = default;

    lambda_wrapper(const lambda_wrapper& rhs) : fn(const_cast<Fn&&>(rhs.fn))
    {
        throw std::logic_error("lambda_wrapper: copy ctor should never be called");
    }
    lambda_wrapper& operator=(const lambda_wrapper&)
    {
        throw std::logic_error("lambda_wrapper: copy assignment should never be called");
    }

    template<typename... Args>
    auto operator()(Args&&... args) { return fn(std::forward<Args>(args)...); }
};

} // namespace kev

#endif // __KEVTYPES_H__
