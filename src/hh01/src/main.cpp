#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

typedef sensor_msgs::msg::Image ImageMsg;

class HH01 : public rclcpp::Node {
private:
  rclcpp::Subscription<ImageMsg>::SharedPtr image_in;
  rclcpp::Publisher<ImageMsg>::SharedPtr image_out;
  cv::CascadeClassifier face_cascade;

public:
  HH01() : Node("hh01_main") {
    RCLCPP_INFO(this->get_logger(), "Launching from directory: %s",
                std::getenv("PWD"));
    // Initialize face cascade classifier
    RCLCPP_INFO(this->get_logger(), "Initializing face cascade detector");
    this->face_cascade.load("./res/haarcascade_frontalface_default.xml");
    // Initialize I/O
    RCLCPP_INFO(this->get_logger(), "Initializing I/O instances");
    image_out = this->create_publisher<ImageMsg>("img/out", 10);
    image_in = this->create_subscription<ImageMsg>(
        "img/in", 10, [this](const ImageMsg::SharedPtr msg) {
          cv_bridge::CvImagePtr img_ptr;
          try {
            img_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            auto &img = img_ptr->image;
            cv::Mat gray;
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
            std::vector<cv::Rect> faces;
            face_cascade.detectMultiScale(gray, faces);
            for (auto &face : faces) {
              cv::rectangle(img, face, cv::Scalar(0, 255, 0), 3);
            }
            auto img_out =
                cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img);
            image_out->publish(*img_out.toImageMsg());
          } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "callback error: %s", e.what());
            return;
          }
        });
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HH01>());
  rclcpp::shutdown();
  return 0;
}
