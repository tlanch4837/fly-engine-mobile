// A small portable thread pool for world generation (terrain chunks, city tiles): jobs run in priority order (lowest first),
// results come back through each user's own locked queue and become GPU meshes on the main thread. No OS headers.
#pragma once
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace fly {

class Workers {
public:
    using Fn = std::function<void()>;
    explicit Workers(int threads = 0) {
        if (threads <= 0) threads = std::max(2, (int)std::thread::hardware_concurrency() - 2);
        for (int i = 0; i < threads; ++i) threads_.emplace_back([this] { Run(); });
    }
    ~Workers() {
        { std::lock_guard<std::mutex> l(m_); quit_ = true; }
        cv_.notify_all();
        for (auto& t : threads_) t.join();
    }
    Workers(const Workers&) = delete;
    Workers& operator=(const Workers&) = delete;

    void Submit(Fn fn, double priority) {
        { std::lock_guard<std::mutex> l(m_); queue_.push_back({priority, std::move(fn)}); std::push_heap(queue_.begin(), queue_.end(), Later); }
        cv_.notify_one();
    }
    // Drops every queued (not yet started) job.
    void Clear() { std::lock_guard<std::mutex> l(m_); queue_.clear(); }
    // Queued plus running.
    size_t Busy() const { std::lock_guard<std::mutex> l(m_); return queue_.size() + (size_t)running_; }
    int Threads() const { return (int)threads_.size(); }

private:
    struct Job { double priority; Fn fn; };
    static bool Later(const Job& a, const Job& b) { return a.priority > b.priority; }
    void Run() {
        for (;;) {
            Job j;
            {
                std::unique_lock<std::mutex> l(m_);
                cv_.wait(l, [&] { return quit_ || !queue_.empty(); });
                if (quit_) return;
                std::pop_heap(queue_.begin(), queue_.end(), Later);
                j = std::move(queue_.back());
                queue_.pop_back();
                ++running_;
            }
            j.fn();
            { std::lock_guard<std::mutex> l(m_); --running_; }
        }
    }
    mutable std::mutex m_;
    std::condition_variable cv_;
    std::vector<Job> queue_;
    int running_ = 0;
    bool quit_ = false;
    std::vector<std::thread> threads_;
};

} // namespace fly
