#pragma once
#include "exception.h"

#include <condition_variable>
#include <memory>
#include <mutex>

namespace threading {

template <typename T> class FastIO {
private:
  std::mutex mutex;
  std::condition_variable cond;
  std::shared_ptr<const T> ptr = nullptr;
  bool open = true;

  void assign(std::shared_ptr<const T> &&ptr) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!open)
      throw EOS();
    this->ptr = ptr;
    cond.notify_all();
  }

  void assign(std::shared_ptr<const T> &ptr) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!open)
      throw EOS();
    this->ptr = ptr;
    cond.notify_all();
  }

public:
  ~FastIO() { close(); }

  std::shared_ptr<const T> read() {
    std::lock_guard<std::mutex> lock(mutex);
    if (!open)
      throw EOS();
    return ptr;
  }

  bool next(std::shared_ptr<const T> &dst, bool wait = false) {
    std::unique_lock<std::mutex> lock(mutex);
    while (wait && open && (ptr == dst || ptr == nullptr))
      cond.wait(lock);
    if (!open)
      throw EOS();
    if (dst == ptr)
      return false;
    dst = ptr;
    return dst != nullptr;
  }

  void write(std::shared_ptr<const T> &data) { assign(data); }
  void write(const T &data) { assign(std::make_shared<const T>(data)); }
  void write(const T &&data) {
    assign(std::make_shared<const T>(std::move(data)));
  }
  void write(const T *data) { assign(std::shared_ptr<const T>(data)); }

  void close() {
    std::lock_guard<std::mutex> lock(mutex);
    open = false;
    ptr = nullptr;
    cond.notify_all();
  }
};

} // namespace threading
