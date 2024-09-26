#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
using Node = rclcpp::Node;

template <class T> class Message : public T {
  template <typename V> V filter_out(std::string name, V vector) {
    V result;
    for (auto &item : vector)
      if (item->get_topic_name() != name)
        result.push_back(item);
    return result;
  }

public:
  std::chrono::system_clock::time_point timestamp =
      std::chrono::system_clock::now();
  bool updated = false;
  // Reset internal data content to default
  void clear() {
    *static_cast<T *>(this) = T();
    updated = false;
  }

  std::vector<typename rclcpp::Subscription<T>::SharedPtr> subscribers;
  std::vector<typename rclcpp::Publisher<T>::SharedPtr> publishers;

  void subscribe(
      Node *node, std::string topic, int qos = 10,
      std::function<void(Message<T> &)> callback = [](auto) {}) {
    subscribers.push_back(node->create_subscription<T>(
        topic, qos, [this, callback](T::SharedPtr msg) {
          *static_cast<T *>(this) = *msg;
          updated = true;
          timestamp = std::chrono::system_clock::now();
          callback(*this);
        }));
  };

  void unsubscribe() { subscribers.clear(); }
  void unsubscribe(std::string topic) {
    subscribers = filter_out(topic, subscribers);
  }

  void publish(Node *node, std::string topic, int qos = 10) {
    publishers.push_back(node->create_publisher<T>(topic, qos));
  };
  void unpublish() { publishers.clear(); }
  void unpublish(std::string topic) {
    publishers = filter_out(topic, publishers);
  }

  void operator()() {
    for (auto publisher : publishers)
      publisher->publish(*static_cast<T *>(this));
  }
};
