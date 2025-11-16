/* Copyright (c) 2014-2020, Fengping Bao <jamol@live.com>
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
#include <atomic>

#include "kmilist.h"

namespace kev {

const size_t kPaddingSize = 128;
///
// enqueue on a thread and dequeue from another thread
///
template <class E>
class KMQueue
{
public:
    KMQueue()
    {
        head_ = new TLNode();
        tail_ = head_;
    }
    ~KMQueue()
    {
        TLNode *node = nullptr;
        while(head_) {
            node = head_;
            head_ = head_->next_.load(std::memory_order_relaxed);
            delete node;
        }
    }

    template <class... Args>
    void enqueue(Args&&... args)
    {
        TLNode *node = new TLNode(std::forward<Args>(args)...);
        tail_->next_.store(node, std::memory_order_release);
        tail_ = node;
        ++count_;
    }

    bool dequeue(E &element)
    {
        auto *node = head_->next_.load(std::memory_order_acquire);
        if (node == nullptr) {
            return false;
        }
        --count_;
        element = std::move(node->element_);
        delete head_;
        head_ = node;
        return true;
    }
    
    E& front() {
        auto *node = head_->next_.load(std::memory_order_acquire);
        if (node == nullptr) {
            static E E_empty{};
            return E_empty;
        }
        return node->element_;
    }
    
    void pop_front() {
        auto *node = head_->next_.load(std::memory_order_acquire);
        if (node != nullptr) {
            --count_;
            delete head_;
            head_ = node;
        }
    }
    
    bool empty() const noexcept
    {
        return size() == 0;
    }
    
    size_t size() const noexcept
    {
        return count_.load(std::memory_order_relaxed);
    }
    
protected:
    class TLNode
    {
    public:
        template<class... Args>
        TLNode(Args&&... args) : element_{ std::forward<Args>(args)... } {}

        E element_;
        std::atomic<TLNode*> next_{ nullptr };
    };

    TLNode* head_{ nullptr };
    char __pad0__[kPaddingSize - sizeof(TLNode*)];
    TLNode* tail_{ nullptr };
    char __pad1__[kPaddingSize - sizeof(TLNode*)];
    std::atomic<size_t> count_{ 0 };
};

// double linked list
template <class E>
class DLQueue final
{
public:
    class DLNode : public kev::shared_inode<DLNode>
    {
    public:
        using Ptr = std::shared_ptr<DLNode>;
        
        template<class... Args>
        DLNode(Args&&... args) : element_{ std::forward<Args>(args)... } {}
        E& element() { return element_; }
        
    private:
        friend class DLQueue;
        E element_;
    };
    using NodePtr = typename DLNode::Ptr;
    
public:
    ~DLQueue()
    {
        list_.clear();
    }
    
    template <class... Args>
    NodePtr enqueue(Args&&... args)
    {
        auto node = std::make_shared<DLNode>(std::forward<Args>(args)...);
        list_.push_back(node);
        return node;
    }
    
    bool dequeue(E &element)
    {
        if (list_.empty()) {
            return false;
        }
        auto node = list_.front_element();
        list_.pop_front();
        element = std::move(node->element_);
        return true;
    }
    
    E& front()
    {
        if(empty()) {
            static E E_empty{};
            return E_empty;
        }
        return list_.front().element_;
    }
    
    NodePtr& front_node()
    {
        return list_.front_element();
    }
    
    void pop_front()
    {
        list_.pop_front();
    }
    
    bool remove(const NodePtr &node)
    {// make sure the node is in this queue
        if (!node) {
            return false;
        }
        list_.remove(node);
        return true;
    }
    
    bool empty() const noexcept
    {
        return list_.empty();
    }

    size_t size() const noexcept
    {
        return list_.size();
    }
    
    void swap(DLQueue &other)
    {
        list_.swap(other.list_);
    }
    
protected:
    ilist<kev::shared_inode<DLNode>> list_;
};

} // namespace kev
