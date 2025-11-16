/* Copyright (c) 2014-2025, Fengping Bao <jamol@live.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#pragma once

#include <memory>
#include <iterator>
#include <algorithm>
#include <type_traits>

namespace kev {

template <typename T, typename Tag = void>
class inode
{
public:
    using value_type = T;
protected:
    T* value_pointer() { return static_cast<T*>(this); }
    const T* value_pointer() const { return static_cast<const T*>(this); }
    
private:
    inode *next_{nullptr};
    inode *prev_{nullptr};

    template<typename E>
    friend class ilist_impl;
};

template <typename T, typename Tag = void>
class shared_inode
{
public:
    using value_type = T;
protected:
    T* value_pointer() { return static_cast<T*>(this); }
    const T* value_pointer() const { return static_cast<const T*>(this); }
    
private:
    std::shared_ptr<shared_inode> next_;
    std::shared_ptr<shared_inode> prev_;

    template<typename E>
    friend class ilist_impl;
};

template <typename E>
struct is_inode_type : public std::false_type {};

template <typename T, typename Tag>
struct is_inode_type<inode<T, Tag>> : public std::true_type {};

template <typename E>
struct is_shared_inode_type : public std::false_type {};

template <typename T, typename Tag>
struct is_shared_inode_type<std::shared_ptr<shared_inode<T, Tag>>> : public std::true_type {};

template <typename E>
struct is_ilist_node : public std::integral_constant<bool, is_inode_type<E>::value ||
                                                    is_shared_inode_type<E>::value> {};

template <typename T>
struct extract_value_type
{
    using type = std::remove_cv_t<std::remove_reference_t<decltype(*std::declval<T>())>>;
};

template <typename T, typename Tag>
struct extract_value_type<inode<T, Tag>*>
{
    using type = std::remove_cv_t<std::remove_reference_t<T>>;
};

template <typename T, typename Tag>
struct extract_value_type<std::shared_ptr<shared_inode<T, Tag>>>
{
    using type = std::remove_cv_t<std::remove_reference_t<T>>;
};

template <typename E> // pointer like type
class ilist_impl
{
public:
    using element_type = std::remove_pointer_t<E>;
    using value_type = typename extract_value_type<E>::type;
    using reference = value_type&;
    using const_reference = const value_type&;
    using pointer = value_type*;
    using const_pointer = const value_type*;
    
    ilist_impl& operator=(const ilist_impl& other) = delete;
    ilist_impl& operator=(ilist_impl&& other) noexcept
    {
        if (this != &other) {
            clear();
            head_ = other.head_;
            count_ = other.count_;
            other.head_ = nullptr;
            other.count_ = 0;
        }
        return *this;
    }
    void push_front(const E& element)
    {
        push_back(element);
        head_ = element;
    }
    void push_back(const E& element)
    {
        if (!head_) {
            head_ = element;
            head_->next_ = head_;
            head_->prev_ = head_;
        }
        else {
            element->next_ = head_;
            element->prev_ = head_->prev_;
            head_->prev_->next_ = element;
            head_->prev_ = element;
        }
        ++count_;
    }
    void pop_front()
    {
        if (head_) {
            remove(head_);
        }
    }
    void pop_back()
    {
        if (head_) {
            remove(head_->prev_);
        }
    }
    const_reference front() const
    {
        return *begin();
    }
    reference front()
    {
        return *begin();
    }
    const_reference back() const
    {
        return *(--end());
    }
    reference back()
    {
        return *(--end());
    }
    template <typename U = element_type, std::enable_if_t<!is_ilist_node<U>::value, int> = 0>
    E front_element() const noexcept
    {
        return head_;
    }
    template <typename U = element_type, std::enable_if_t<!is_ilist_node<U>::value, int> = 0>
    E back_element() const noexcept
    {
        if (head_) {
            return head_->prev_;
        }
        return nullptr;
    }
    template <typename U = element_type, std::enable_if_t<is_inode_type<U>::value, int> = 0>
    pointer front_element() const noexcept
    {
        return head_->value_pointer();
    }
    template <typename U = element_type, std::enable_if_t<is_inode_type<U>::value, int> = 0>
    pointer back_element() const noexcept
    {
        if (head_) {
            return head_->prev_->value_pointer();
        }
        return nullptr;
    }
    template <typename U = E, std::enable_if_t<is_shared_inode_type<U>::value, int> = 0>
    std::shared_ptr<value_type> front_element() const noexcept
    {
        return std::static_pointer_cast<value_type>(head_);
    }
    template <typename U = E, std::enable_if_t<is_shared_inode_type<U>::value, int> = 0>
    std::shared_ptr<value_type> back_element() const noexcept
    {
        if (head_) {
            return std::static_pointer_cast<value_type>(head_->prev_);
        }
        return nullptr;
    }
    void clear()
    {
        if (head_) {
            auto node = head_;
            do {
                auto next = node->next_;
                node->next_ = nullptr;
                node->prev_ = nullptr;
                node = next;
            } while (node != head_);
            head_ = nullptr;
            count_ = 0;
        }
    }
    void splice(ilist_impl& other) noexcept
    {
        if (other.head_) {
            if (!head_) {
                head_ = other.head_;
            }
            else {
                head_->prev_->next_ = other.head_;
                other.head_->prev_->next_ = head_;
                std::swap(head_->prev_, other.head_->prev_);
            }
            count_ += other.count_;
            other.head_ = nullptr;
            other.count_ = 0;
        }
    }
    bool empty() const noexcept
    {
        return head_ == nullptr;
    }
    size_t size() const noexcept
    {
        return count_;
    }
    void swap(ilist_impl& other) noexcept
    {
        std::swap(head_, other.head_);
        std::swap(count_, other.count_);
    }
    void remove(const E& element)
    {// make sure the element is in this list
        if (!head_ || !element || element->next_ == nullptr || element->prev_ == nullptr) {
            return;
        }
        if (element->next_ == element && element->prev_ == element && head_ == element) {
            element->next_ = nullptr;
            element->prev_ = nullptr;
            head_ = nullptr;
        }
        else {
            element->prev_->next_ = element->next_;
            element->next_->prev_ = element->prev_;
            auto next = element->next_;
            element->next_ = nullptr;
            element->prev_ = nullptr;
            if (head_ == element) {
                head_ = std::move(next);
            }
        }
        --count_;
    }

protected:
    ilist_impl() = default;
    ilist_impl(const ilist_impl& other) = delete;
    ilist_impl(ilist_impl&& other) noexcept
        : head_(other.head_), count_(other.count_)
    {
        other.head_ = nullptr;
        other.count_ = 0;
    }
    ~ilist_impl()
    {
        clear();
    }

public:
    class iterator_impl
    {
    public:
        using iterator_category = std::bidirectional_iterator_tag;
        using difference_type = std::ptrdiff_t;

        iterator_impl(const E current, const E head) noexcept
            : current_(current), head_(head)
        {
        }
        iterator_impl(const iterator_impl& other) noexcept = default;

        iterator_impl& operator++() noexcept
        {
            if (current_) {
                current_ = current_->next_;
                if (current_ == head_) {
                    current_ = nullptr;
                }
            }
            return *this;
        }
        iterator_impl operator++(int) noexcept
        {
            iterator_impl ret(*this);
            ++(*this);
            return ret;
        }
        iterator_impl& operator--() noexcept
        {
            if (!current_ && head_) {
                current_ = head_->prev_;
            } else if (current_) {
                current_ = current_->prev_;
                if (current_ == head_) {
                    current_ = nullptr;
                }
            }
            return *this;
        }
        iterator_impl operator--(int) noexcept
        {
            iterator_impl ret(*this);
            --(*this);
            return ret;
        }

        const_reference operator*() const noexcept
        {
            return get_const_ref();
        }
        reference operator*() noexcept
        {
            return get_ref();
        }
        const_pointer operator->() const noexcept
        {
            return &get_const_ref();
        }
        pointer operator->() noexcept
        {
            return &get_ref();
        }

        friend bool operator==(const iterator_impl& it1, const iterator_impl& it2) noexcept
        {
            return it1.current_ == it2.current_ && it1.head_ == it2.head_;
        }
        friend bool operator!=(const iterator_impl& it1, const iterator_impl& it2) noexcept
        {
            return !(it1 == it2);
        }

    private:
        template <typename U = element_type, std::enable_if_t<is_ilist_node<U>::value, int> = 0>
        reference get_ref() const noexcept
        {
            return *current_->value_pointer();
        }
        template <typename U = element_type, std::enable_if_t<is_ilist_node<U>::value, int> = 0>
        const_reference get_const_ref() const noexcept
        {
            return *current_->value_pointer();
        }
        template <typename U = element_type, std::enable_if_t<!is_ilist_node<U>::value, char> = 0>
        reference get_ref() const noexcept
        {
            return *current_;
        }
        template <typename U = element_type, std::enable_if_t<!is_ilist_node<U>::value, char> = 0>
        const_reference get_const_ref() const noexcept
        {
            return *current_;
        }
    protected:
        E current_{ nullptr };
        E head_{ nullptr };

        friend class ilist_impl<E>;
    };

    using iterator = iterator_impl;
    using const_iterator = const iterator_impl;

    iterator begin() const noexcept
    {
        return iterator_impl(head_, head_);
    }
    iterator end() const noexcept
    {
        return iterator_impl(nullptr, head_);
    }
    iterator erase(iterator it)
    {
        if (!it.current_) {
            return end();
        }
        auto next = it.current_->next_;
        const bool was_last = (next == head_ || next == it.current_);
        remove(it.current_);
        return iterator_impl(was_last ? nullptr : next, head_);
    }
private:
    E head_{ nullptr };
    size_t count_{ 0 };
};

template <typename E>
class ilist : public ilist_impl<E> {};

template <typename T, typename TAG>
class ilist<inode<T, TAG>> : public ilist_impl<inode<T, TAG>*> {};

template <typename T, typename TAG>
class ilist<shared_inode<T, TAG>> : public ilist_impl<std::shared_ptr<shared_inode<T, TAG>>> {};

} // namespace kev
