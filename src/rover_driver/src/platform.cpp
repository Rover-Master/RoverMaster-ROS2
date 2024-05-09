#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>
#include <vector>

#include "serial/serial.h"
#include "util/clamp.h"

using namespace std::chrono_literals;

typedef geometry_msgs::msg::Twist Twist;
typedef std_msgs::msg::Float64 Float64;

typedef struct {
  double lower, upper;
} Limit;

class Arg {
public:
  const std::string raw;
  std::string key, value;
  Arg(std::string str) : raw(str) {
    // optional '=' separator
    size_t pos = raw.find('=');
    if (pos == std::string::npos) {
      key = raw;
    } else {
      key = raw.substr(0, pos);
      value = raw.substr(pos + 1);
    }
  };
};

class Command {
public:
  bool valid = false;
  std::string cmd;
  std::vector<Arg> args;
  Command(std::string line) {
    if (line.empty())
      return;
    size_t pos = line.find(' ');
    if (pos == std::string::npos) {
      cmd = line;
    } else {
      cmd = line.substr(0, pos);
      line = line.substr(pos + 1);
      while (!line.empty()) {
        pos = line.find(' ');
        if (pos == std::string::npos) {
          args.push_back(Arg(line));
          break;
        } else if (pos > 0) {
          args.push_back(Arg(line.substr(0, pos)));
        }
        line = line.substr(pos + 1);
      }
    }
    valid = true;
  }
};

class PlaformDriver : public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<Float64>::SharedPtr spd_set = nullptr;
  rclcpp::Subscription<Twist>::SharedPtr att_set = nullptr;
  rclcpp::Publisher<Twist>::SharedPtr att_get = nullptr;
  Twist att;
  // Configurations
  std::string port;
  int baud;
  // Serial port
  int serial_fd;
  // Input line buffer
  char line[1024], *line_cursor = line;
  char *const line_end = line + sizeof(line) - 1;
  /**
   * @brief Wait for a complete line from serial device.
   *        Data stored in `(char *) line`.
   * @param blocking Wait for data if true, return immediately otherwise.
   * @return int
   *         `0`: line not ready (pending);
   *         positive value: data valid, value = line length;
   *         `-1`: buffer overflow.
   *         other negative value: unknown error.
   */
  int wait_line(bool blocking = true) {
    while (1) {
      int ret = serial::readline(serial_fd, &line_cursor, line_end, '\n');
      if (ret == -1)
        RCLCPP_WARN(get_logger(), "serial line buffer overflow");
      else if (ret < 0)
        RCLCPP_WARN(get_logger(), "serial readline error %d", ret);
      if (ret > 0)
        ret = line_cursor - line;
      if (ret != 0)
        line_cursor = line;
      // Respond to shutdown signal
      if (!rclcpp::ok())
        return -1;
      if (ret || !blocking)
        return ret;
    }
  }

  enum { ATT, SPD } latest = ATT;
  Twist::SharedPtr msg_att_set = nullptr;
  Float64::SharedPtr msg_spd_set = nullptr;
  void init_subscription(Limit j1_limit) {
    att_set = create_subscription<Twist>("platform/attitude/set", 10,
                                         [this](const Twist::SharedPtr msg) {
                                           msg_att_set = msg;
                                           latest = ATT;
                                         });
    RCLCPP_INFO(get_logger(), "Subscribed to %s", att_set->get_topic_name());
    spd_set = create_subscription<Float64>(
        "platform/speed/set", 10, [this](const Float64::SharedPtr msg) {
          msg_spd_set = msg;
          latest = SPD;
        });
    RCLCPP_INFO(get_logger(), "Subscribed to %s", spd_set->get_topic_name());
    create_wall_timer(50ms, [this, j1_limit]() {
      char cmd[256];
      // First In First Out
      if (msg_att_set != nullptr && (latest == SPD || msg_spd_set == nullptr)) {
        try {
          auto &pan = msg_att_set->angular.z, &tilt = msg_att_set->angular.y;
          clamp(pan, j1_limit.lower, j1_limit.upper);
          clamp(tilt, -90.0, 30.0);
          snprintf(cmd, sizeof(cmd), "MOVE J1=%f J2=%f\n", pan, tilt);
          serial::write(serial_fd, cmd);
          RCLCPP_INFO(get_logger(), "Command = %s", cmd);
          msg_att_set = nullptr;
        } catch (std::exception &e) {
          RCLCPP_ERROR(get_logger(), "callback error: %s", e.what());
        }
      } else if (msg_spd_set != nullptr) {
        try {
          double speed = msg_spd_set->data;
          clamp(speed, 1.0, 500.0);
          snprintf(cmd, sizeof(cmd), "SPEED %f\n", speed);
          serial::write(serial_fd, cmd);
          RCLCPP_INFO(get_logger(), "Command = %s", cmd);
          msg_spd_set = nullptr;
        } catch (std::exception &e) {
          RCLCPP_ERROR(get_logger(), "callback error: %s", e.what());
        }
      } else {
      }
    });
  }

  void process_command() {
    Command cmd(line);
    if (!cmd.valid) {
      RCLCPP_WARN(get_logger(), "Invalid command: %s", line);
      return;
    }
    if (cmd.cmd == "INFO") {
      std::stringstream ss;
      ss << "Controller:";
      for (auto arg : cmd.args)
        ss << ' ' << arg.raw;
      RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
    } else if (cmd.cmd == "RANGE") {
      bool flag_valid = false;
      Limit j1_limit;
      for (auto arg : cmd.args) {
        if (arg.key == "J1") {
          // Comma separated range values
          auto npos = arg.value.find(',');
          if (npos != std::string::npos) {
            j1_limit.lower = std::stof(arg.value.substr(0, npos));
            j1_limit.upper = std::stof(arg.value.substr(npos + 1));
            flag_valid = true;
          }
        }
      }
      if (flag_valid) {
        RCLCPP_INFO(get_logger(), "Homming sequence completed.");
        init_subscription(j1_limit);
      }
    } else if (cmd.cmd == "SYNC") {
      if (att_set == nullptr)
        return;
      for (auto arg : cmd.args) {
        if (arg.key == "J1") {
          att.angular.z = std::stof(arg.value);
        } else if (arg.key == "J2") {
          att.angular.y = std::stof(arg.value);
        }
      }
      att_get->publish(att);
    } else {
      RCLCPP_WARN(get_logger(), "Unknown command : %s", line);
    }
  }

public:
  PlaformDriver() : Node("Rover_PlaformDriver") {
    // Initialize serial connection
    declare_parameter("port", "/dev/ttyUSB0");
    declare_parameter("baud", 115200);
    port = get_parameter("port").as_string();
    baud = get_parameter("baud").as_int();
    RCLCPP_INFO(get_logger(), "Opening serial port %s, baudrate %d",
                port.c_str(), baud);
    serial_fd = serial::open(port.c_str(), baud);
    std::this_thread::sleep_for(2s);
    serial::flush(serial_fd);
    std::this_thread::sleep_for(1s);
    serial::write(serial_fd, "\n");
    serial::write(serial_fd, "ENABLE\n");
    // Initiate auto homing sequence
    RCLCPP_INFO(get_logger(), "Initializing Homing Sequence");
    serial::write(serial_fd, "HOME J1\n");
    // Initialize topics
    RCLCPP_INFO(get_logger(), "Initializing node topics");
    att_get = create_publisher<Twist>("platform/attitude/get", 10);
    att.linear.x = att.linear.y = att.linear.z = 0;
    att.angular.x = att.angular.y = att.angular.z = 0;
    timer = create_wall_timer(10ms, [this]() {
      while (wait_line(false) > 1)
        process_command();
      if (msg_att_set != nullptr)
        msg_att_set = nullptr;
    });
  }

  ~PlaformDriver() {
    RCLCPP_INFO(get_logger(), "Disabling platform driver");
    serial::write(serial_fd, "\nDISABLE\n");
    RCLCPP_INFO(get_logger(), "Closing serial port %s", port.c_str());
    close(serial_fd);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaformDriver>());
  rclcpp::shutdown();
  return 0;
}
