#include <cmath>
#include <errno.h>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <thread>

#include "MultiWii/protocol.h"
#include "serial/serial.h"
#include "util/assert.h"
#include "util/clamp.h"

using namespace std::chrono_literals;
typedef geometry_msgs::msg::Twist Twist;
static const double V[4][3] = {
    // Motor Layout (UP is forward)
    // 4 2
    // 3 1
    {1, -1, -1},
    {1, 1, -1},
    {1, 1, 1},
    {1, -1, 1}};

#define DSHOT_NEUTRAL 1500
#define BACKUP_COMP 2
#define MAX_VELOCITY 100
static const double max_velocity = MAX_VELOCITY;
// Single wheel can top to 2 times of the maximum velocity
// This ensures the base can do same translation speed in 360 degrees
#define COMB_LIMIT 2
static const double comb_limit = COMB_LIMIT;

class BaseDriver : public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<Twist>::SharedPtr vel_set;
  rclcpp::Publisher<Twist>::SharedPtr vel_get;
  Twist::SharedPtr msg_vel_set = nullptr;
  struct {
    rclcpp::Publisher<Twist>::SharedPtr publisher;
    Twist message;
    void publish() { publisher->publish(message); }
  } imu_acc, imu_att;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_get;
  // Configurations
  std::string port;
  int baud;
  // Serial port
  int serial_fd;
  // MSP inbound receiver
  MultiWii::Receiver receiver;
  // Debounced device bound messages
  MSP_SET_MOTOR_t msp_set_motor;

  // Velocity command
  void set_motors(double &vx, double &vy, double &vr) {
    auto &motor = msp_set_motor.motor;
    clamp(vx, -1.0, 1.0);
    clamp(vy, -1.0, 1.0);
    clamp(vr, -1.0, 1.0);
    double v[4];
    double amplitude = 0;
    for (int i = 0; i < 4; i++) {
      v[i] = vx * V[i][0] + vy * V[i][1] + vr * V[i][2];
      amplitude = std::max(amplitude, abs(v[i]));
    }
    // Throttling to comform to the speed limit
    amplitude /= comb_limit;
    if (amplitude > 1) {
      for (int i = 0; i < 4; i++)
        v[i] /= amplitude;
    }
    // Map to DSHOT values centered at 1500
    for (int i = 0; i < 4; i++) {
      double cmd_vel = std::round(DSHOT_NEUTRAL + max_velocity * v[i]);
      if (cmd_vel < DSHOT_NEUTRAL)
        cmd_vel -= BACKUP_COMP;
      clamp(cmd_vel, DSHOT_NEUTRAL - max_velocity * comb_limit - BACKUP_COMP,
            DSHOT_NEUTRAL + max_velocity * comb_limit);
      motor[i] = round(cmd_vel);
    }
    MultiWii::send<MSP_CMD_SET_MOTOR>(serial_fd, msp_set_motor);
  }

  void serial_init() {
    // Initialize serial connection
    declare_parameter<std::string>("pid", "");
    declare_parameter<std::string>("vid", "");
    declare_parameter<int>("baud", 115200);
    baud = get_parameter("baud").as_int();
    const auto vid = get_parameter("vid").as_string();
    const auto pid = get_parameter("pid").as_string();
    const std::string name =
        (vid.empty() ? "----" : vid) + ":" + (pid.empty() ? "----" : pid);
    const auto ports = serial::locate(vid, pid);
    if (ports.empty()) {
      throw std::runtime_error("No serial port found for device " + name);
    } else if (ports.size() > 1) {
      RCLCPP_WARN(get_logger(), "Multiple serial ports found for device %s",
                  name.c_str());
    }
    port = ports[0];
    RCLCPP_INFO(get_logger(), "Opening serial port %s (%s), baudrate %d",
                port.c_str(), name.c_str(), baud);
    serial_fd = serial::open(port.c_str(), baud);
    if (serial_fd < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port %s", port.c_str());
      RCLCPP_ERROR(get_logger(), "Error: %s", strerror(errno));
      throw std::runtime_error("Failed to open serial port " + port + ": " +
                               strerror(errno));
    }
    std::this_thread::sleep_for(2s);
    serial::flush(serial_fd);
    // Halt all motors (NEUTRAL)
    std::this_thread::sleep_for(1s);
    memset(&msp_set_motor, 0, sizeof(msp_set_motor));
    double v = 0;
    set_motors(v, v, v);
    std::this_thread::sleep_for(1s);
  }

  struct {
    sensor_msgs::msg::Imu data;
    bool acc_init = false;
    bool att_init = false;
  } imu_status;

  void publish_imu() {
    if (!imu_status.acc_init || !imu_status.att_init)
      return;
    imu_get->publish(imu_status.data);
    imu_status.acc_init = false;
    imu_status.att_init = false;
  }

  // Process serial inbound data
  void serial_in() {
    uint8_t c;
    while (read(serial_fd, &c, 1) > 0) {
      if (!receiver.recv(c))
        continue;
      else if (receiver.match<MSP_QRY_RAW_IMU>())
        update(receiver.data<MSP_QRY_RAW_IMU>());
      else if (receiver.match<MSP_QRY_ATTITUDE>())
        update(receiver.data<MSP_QRY_ATTITUDE>());
    }
  }

  // Data handlers
  void update(MSP_RAW_IMU_t data) {
    imu_acc.message.linear.x = data.accX;
    imu_acc.message.linear.y = data.accY;
    imu_acc.message.linear.z = data.accZ;
    imu_acc.message.angular.x = data.gyrX;
    imu_acc.message.angular.y = data.gyrY;
    imu_acc.message.angular.z = data.gyrZ;
    imu_acc.publish();
    static const double G = 9.80;
    imu_status.data.linear_acceleration.x = data.accX * G / 2050.0;
    imu_status.data.linear_acceleration.y = data.accY * G / 2050.0;
    imu_status.data.linear_acceleration.z = data.accZ * G / 2050.0;
    imu_status.data.angular_velocity.x = data.gyrX;
    imu_status.data.angular_velocity.y = data.gyrY;
    imu_status.data.angular_velocity.z = data.gyrZ;
    imu_status.acc_init = true;
    publish_imu();
  }

  void update(MSP_ATTITUDE_t data) {
    imu_att.message.angular.x = data.angx;
    imu_att.message.angular.y = data.angy;
    imu_att.message.angular.z = data.heading;
    imu_att.publish();
    imu_status.data.orientation.x = data.angx;
    imu_status.data.orientation.y = data.angy;
    imu_status.data.orientation.z = data.heading;
    imu_status.att_init = true;
    publish_imu();
  }

  void serial_out() {
    if (msg_vel_set != nullptr) {
      // Set motors
      set_motors(msg_vel_set->linear.x, msg_vel_set->linear.y,
                 msg_vel_set->angular.z);
      // TODO: calculate and publish odometry
      vel_get->publish(*msg_vel_set);
      msg_vel_set = nullptr;
    }
    // Send query for IMU data
    MultiWii::send<MSP_QRY_RAW_IMU>(serial_fd);
    MultiWii::send<MSP_QRY_ATTITUDE>(serial_fd);
  }

public:
  BaseDriver() : Node("Rover_BaseDriver") {
    serial_init();
    // Initialize topics
    RCLCPP_INFO(get_logger(), "Initializing node topics");
    vel_get = create_publisher<Twist>("base/velocity/get", 10);
    vel_set = create_subscription<Twist>(
        "base/velocity/set", 10, [this](const Twist::SharedPtr msg) {
          try {
            msg->linear.z = 0;
            msg->angular.x = 0;
            msg->angular.y = 0;
            msg_vel_set = msg;
          } catch (std::exception &e) {
            RCLCPP_ERROR(get_logger(), "callback error: %s", e.what());
          }
        });
    imu_acc.publisher = create_publisher<Twist>("base/imu/acc", 10);
    imu_att.publisher = create_publisher<Twist>("base/imu/att", 10);
    imu_get = create_publisher<sensor_msgs::msg::Imu>("base/imu/get", 10);
    // Timer loop for serial I/O
    timer = create_timer(10ms, [this]() {
      serial_in();
      serial_out();
    });
  }

  ~BaseDriver() {
    RCLCPP_INFO(get_logger(), "Stopping all motors");
    double v = 0;
    set_motors(v, v, v);
    std::this_thread::sleep_for(1s);
    RCLCPP_INFO(get_logger(), "Closing serial port %s", port.c_str());
    close(serial_fd);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseDriver>());
  rclcpp::shutdown();
  return 0;
}
