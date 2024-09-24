#include "Message.h"
#include "Types.h"

#include "MultiWii/protocol.h"
#include <chrono>

class BaseDriver : public Node {
private:
  std::vector<Timer::SharedPtr> timers;
  Message<Bool> halt_signal;
  Message<Twist> velocity_io;
  Message<Imu> imu_out;
  // MSP inbound receiver
  MultiWii::Device::Ptr device = nullptr;
  // Timeout since last velocity command to halt motors
  const std::chrono::milliseconds vel_timeout = 1000ms;
  // Halt flag
  bool halted = false;
  void halt();
  void motion(double vx, double vy, double vr);
  struct {
    sensor_msgs::msg::Imu data;
    bool acc_init = false;
    bool att_init = false;
  } imu_status;
  void update(MSP::RAW_IMU data);

public:
  BaseDriver();
  ~BaseDriver();
  // Handle packets sent from serial device
  void communicate();
};
