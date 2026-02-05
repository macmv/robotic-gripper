#include <memory>
#include <string>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/set_bool.hpp"

class GripperNode : public rclcpp::Node {
public:
  GripperNode() : Node("robot_gripper") {
    tty_path_ = declare_parameter<std::string>("tty", "");
    if (tty_path_.empty()) {
      RCLCPP_FATAL(
          get_logger(),
          "Missing required parameter 'tty' (e.g., tty:=/dev/ttyUSB0)");
      throw std::runtime_error("missing required parameter: tty");
    }

    serial_fd_ = ::open(tty_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", tty_path_.c_str(),
                   std::strerror(errno));
    } else {
      if (!configure_serial_(serial_fd_)) {
        RCLCPP_ERROR(get_logger(), "Failed to configure %s", tty_path_.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Opened %s at 500000 baud",
                    tty_path_.c_str());
      }
    }

    position_publisher_ =
        create_publisher<std_msgs::msg::Float32>("gripper/position", 10);
    service_ = create_service<std_srvs::srv::SetBool>(
        "gripper/set_open",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          if (request->data) {
            open_position_ = 1.0f;
            if (serial_fd_ >= 0) {
              const char cmd = 'o';
              if (::write(serial_fd_, &cmd, 1) != 1) {
                RCLCPP_ERROR(get_logger(), "Failed to write open command: %s",
                             std::strerror(errno));
              }
            }
            response->message = "opened";
          } else {
            open_position_ = 0.0f;
            if (serial_fd_ >= 0) {
              const char cmd = 'c';
              if (::write(serial_fd_, &cmd, 1) != 1) {
                RCLCPP_ERROR(get_logger(), "Failed to write open command: %s",
                             std::strerror(errno));
              }
            }
            response->message = "closed";
          }
          std_msgs::msg::Float32 pos_msg;
          pos_msg.data = open_position_;
          position_publisher_->publish(pos_msg);
          response->success = true;
        });
  }

  ~GripperNode() override {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
    }
  }

private:
  bool configure_serial_(int fd) {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    if (cfsetispeed(&tty, B500000) != 0 || cfsetospeed(&tty, B500000) != 0) {
      RCLCPP_ERROR(get_logger(), "cfsetispeed/cfsetospeed failed: %s",
                   std::strerror(errno));
      return false;
    }

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      return false;
    }

    return true;
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  float open_position_ = 0.0f;
  int serial_fd_ = -1;
  std::string tty_path_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperNode>());
  rclcpp::shutdown();
  return 0;
}
