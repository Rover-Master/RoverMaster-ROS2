#include <chrono>
#include <cmath>

#include "BaseDriver.h"

#include "MultiWii/protocol.h"
#include "serial/serial.h"
#include "util/clamp.h"

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

void BaseDriver::halt() {
  MSP::SET_MOTOR command;
  for (size_t i = 0; i < (sizeof(command.motor) / sizeof(*command.motor)); i++)
    command.motor[i] = DSHOT_NEUTRAL;
  device->send(command);
}

// Velocity command
void BaseDriver::motion(double vx, double vy, double vr) {
  if (halted)
    return;
  MSP::SET_MOTOR command;
  auto &motor = command.motor;
  clamp(vx, -1.0, 1.0);
  clamp(vy, -1.0, 1.0);
  clamp(vr, -1.0, 1.0);
  double v[4];
  double amplitude = 0;
  for (int i = 0; i < 4; i++) {
    v[i] = vx * V[i][0] + vy * V[i][1] + vr * V[i][2];
    amplitude = std::max(amplitude, abs(v[i]));
  }
  // Throttle to conform to the speed limit
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
  device->send(command);
}

// https://github.com/ros2/launch/issues/797
std::string hex_string(std::string s) {
  if (s.starts_with("0x") || s.starts_with("0X"))
    return s.substr(2);
  if (s.starts_with("x") || s.starts_with("X"))
    return s.substr(1);
  return s;
}

// Data handlers
void BaseDriver::update(MSP::RAW_IMU data,
                        std::chrono::system_clock::time_point timestamp) {
  data.accX *= 1;
  data.accY *= -1;
  data.accZ *= 1;
  data.gyrX *= 1;
  data.gyrY *= -1;
  data.gyrZ *= 1;
  auto &msg = imu_out;
  static const double g = 9.80 / 510.0;
  msg.linear_acceleration.x = data.accX * g;
  msg.linear_acceleration.y = data.accY * g;
  msg.linear_acceleration.z = data.accZ * g;
  static const double r = 3.1415926 / 180.0;
  msg.angular_velocity.x = data.gyrX * r;
  msg.angular_velocity.y = data.gyrY * r;
  msg.angular_velocity.z = data.gyrZ * r;
  msg.header.stamp.sec =
      std::chrono::time_point_cast<std::chrono::seconds>(timestamp)
          .time_since_epoch()
          .count();
  msg.header.stamp.nanosec =
      std::chrono::time_point_cast<std::chrono::nanoseconds>(timestamp)
          .time_since_epoch()
          .count() %
      1000000000UL;
  imu_status.acc_updated = true;
  if (imu_status.updated()) {
    imu_status.reset();
    msg();
  }
}

#define deg2rad(deg) (static_cast<double>(deg) * 3.1415926 / 180.0)
// Data handlers
void BaseDriver::update(MSP::ATTITUDE data,
                        std::chrono::system_clock::time_point timestamp) {
  const double x = deg2rad(data.angx / 10.0);   // roll
  const double y = deg2rad(-data.angy / 10.0);  // pitch
  const double z = deg2rad(data.heading / 1.0); // yaw
  // Convert to quaternion
  const double cx = cos(x / 2), sx = sin(x / 2);
  const double cy = cos(y / 2), sy = sin(y / 2);
  const double cz = cos(z / 2), sz = sin(z / 2);
  auto &msg = imu_out;
  msg.orientation.w = cx * cy * cz + sx * sy * sz;
  msg.orientation.x = sx * cy * cz - cx * sy * sz;
  msg.orientation.y = cx * sy * cz + sx * cy * sz;
  msg.orientation.z = cx * cy * sz - sx * sy * cz;
  imu_status.att_updated = true;
  msg.header.stamp.sec =
      std::chrono::time_point_cast<std::chrono::seconds>(timestamp)
          .time_since_epoch()
          .count();
  msg.header.stamp.nanosec =
      std::chrono::time_point_cast<std::chrono::nanoseconds>(timestamp)
          .time_since_epoch()
          .count() %
      1000000000UL;
  if (imu_status.updated()) {
    imu_status.reset();
    msg();
  }
}

BaseDriver::BaseDriver() : Node("Rover_BaseDriver") {
  // Initialize serial connection
  declare_parameter<std::string>("pid", "");
  declare_parameter<std::string>("vid", "");
  declare_parameter<int>("baud", 115200);
  const int baud = get_parameter("baud").as_int();
  const auto vid = hex_string(get_parameter("vid").as_string());
  const auto pid = hex_string(get_parameter("pid").as_string());
  const std::string name =
      (vid.empty() ? "----" : vid) + ":" + (pid.empty() ? "----" : pid);
  const auto ports = serial::locate(vid, pid);
  if (ports.empty()) {
    throw std::runtime_error("No serial port found for device " + name);
  } else if (ports.size() > 1) {
    RCLCPP_WARN(get_logger(), "Multiple serial ports found for device %s",
                name.c_str());
  }
  const auto port = ports[0];
  RCLCPP_INFO(get_logger(), "Opening serial port %s (%s:%s), baudrate %d",
              port.path.c_str(), port.vid.c_str(), port.pid.c_str(), baud);
  device = std::make_unique<MultiWii::Device>(port.path, baud);
  // Halt all motors at startup (NEUTRAL)
  halt();
  // Initialize topics
  RCLCPP_INFO(get_logger(), "Initializing ROS2 topics");
  halt_signal.subscribe(this, "base/halt", 10, [this](auto msg) {
    halted = msg.data;
    if (halted)
      halt();
  });
  velocity_io.subscribe(this, "base/velocity/set");
  velocity_io.publish(this, "base/velocity/get");
  imu_out.publish(this, "base/imu");
  // Timer loop for serial I/O
  timers.push_back(create_timer(50ms, [this]() {
    if (imu_status.query())
      device->query<MSP::RAW_IMU>();
    else
      device->query<MSP::ATTITUDE>();
  }));
  timers.push_back(
      create_timer(2000ms, [this]() { device->query<MSP::ANALOG>(); }));
  // Debounced velocity command
  timers.push_back(create_timer(10ms, [this]() {
    if (velocity_io.updated) {
      motion(velocity_io.linear.x, velocity_io.linear.y, velocity_io.angular.z);
      velocity_io();
      velocity_io.updated = false;
    } else {
      using clock = std::chrono::system_clock;
      const auto time_since_last_cmd = clock::now() - velocity_io.timestamp;
      if (time_since_last_cmd > vel_timeout) {
        halt();
      }
    }
  }));
}

BaseDriver::~BaseDriver() {
  RCLCPP_INFO(get_logger(), "Stopping all motors");
  halt();
  RCLCPP_INFO(get_logger(), "Resetting device");
  device->query<MSP::REBOOT>();
  RCLCPP_INFO(get_logger(), "Disconnecting ...");
  std::this_thread::sleep_for(100ms);
  device = nullptr;
}

void BaseDriver::communicate() {
  const auto raw_packet = device->read();
  if (raw_packet == nullptr)
    return;
  try {
    if (raw_packet->is<MSP::RAW_IMU>())
      return update(raw_packet->as<MSP::RAW_IMU>(), raw_packet->timestamp);
    if (raw_packet->is<MSP::ATTITUDE>())
      return update(raw_packet->as<MSP::ATTITUDE>(), raw_packet->timestamp);
    if (raw_packet->is<MSP::ANALOG>()) {
      const auto &msg = raw_packet->as<MSP::ANALOG>();
      double volt = static_cast<double>(msg.vbat) * 0.01;
      RCLCPP_INFO(get_logger(), "Battery voltage: %.2f V", volt);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "MSP Error: %s", e.what());
  }
}
