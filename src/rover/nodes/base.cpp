#include "BaseDriver.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BaseDriver>();
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->communicate();
  }
  rclcpp::shutdown();
  return 0;
}
