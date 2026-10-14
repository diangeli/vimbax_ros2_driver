#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace camera_recorder
{

template<typename T>
class BoundedQueue
{
public:
  explicit BoundedQueue(std::size_t capacity)
  : capacity_(capacity) {}

  bool try_push(T item)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_ || items_.size() >= capacity_) {
        return false;
      }
      items_.push_back(std::move(item));
    }
    ready_.notify_one();
    return true;
  }

  // Keeps returning queued items after close() and ends with nullopt once empty.
  std::optional<T> pop()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ready_.wait(lock, [this] {return closed_ || !items_.empty();});
    if (items_.empty()) {
      return std::nullopt;
    }
    T item = std::move(items_.front());
    items_.pop_front();
    return item;
  }

  std::size_t size()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return items_.size();
  }

  void close()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    ready_.notify_all();
  }

private:
  const std::size_t capacity_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<T> items_;
  bool closed_ = false;
};

}  // namespace camera_recorder
