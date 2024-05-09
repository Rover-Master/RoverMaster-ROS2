#pragma once

#include <condition_variable>
#include <mutex>

namespace threading {

template <typename T> class Sync {
private:
  T data;
  bool valid = false;
  std::mutex mutex;
  std::condition_variable cond_r, cond_w;

public:
  Sync() {}

  bool has_data() { return valid; }

  void write(T data, bool overwrite = true) {
    std::unique_lock<std::mutex> lock(mutex);
    // Only blocking when overwriting is prohibited
    while (!overwrite && valid)
      cond_r.wait(lock);
    this->data = data;
    cond_w.notify_one();
  }

  T read() {
    std::unique_lock<std::mutex> lock(mutex);
    while (!valid)
      cond_w.wait(lock);
    T data = this->data;
    valid = false;
    cond_r.notify_one();
    return data;
  }
};

} // namespace threading
