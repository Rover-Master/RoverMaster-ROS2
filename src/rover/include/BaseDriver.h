#include "Message.h"
#include "Types.h"

#include "MultiWii/protocol.h"

class BaseDriver : public Node {
private:
  std::vector<Timer::SharedPtr> timers;
  Message<Bool> halt_signal;
  Message<Twist> velocity_io;
  Message<Odometry> odom_out;
  Message<Imu> imu_out;
  // MSP inbound receiver
  MultiWii::Device::Ptr device = nullptr;
  // Timeout since last velocity command to halt motors
  const std::chrono::milliseconds vel_timeout = 2000ms;
  // Halt flag
  bool halted = false;
  void halt();
  void motion(double vx, double vy, double vr);
  struct {
    uint8_t counter = 0;
    bool acc_updated = false;
    bool att_updated = false;
    inline bool query() {
      counter = (counter + 1) % 2;
      return counter == 0;
    }
    inline bool updated() { return acc_updated && att_updated; }
    inline void reset() { acc_updated = att_updated = false; }
  } imu_status;
  void update(MSP::RAW_IMU data, time_point timestamp);
  void update(MSP::ATTITUDE data, time_point timestamp);

  // Linear velocity normalization factor (m/s)
  double odom_k_linear;
  double odom_initial_heading = NAN;
  double odom_last_heading = NAN;
  void integrate_odometry();
  void integrate_odometry(double current_heading);

public:
  BaseDriver();
  ~BaseDriver();
  // Handle packets sent from serial device
  void communicate();
};
