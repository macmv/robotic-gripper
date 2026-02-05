#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class GripperNode : public rclcpp::Node {
public:
  GripperNode() : Node("robot_gripper") {
    position_publisher_ =
        create_publisher<std_msgs::msg::Float32>("gripper/position", 10);
    service_ = create_service<std_srvs::srv::SetBool>(
        "gripper/set_open",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          if (request->data) {
            // TODO: open gripper hardware here.
            open_position_ = 1.0f;
            response->message = "opened";
          } else {
            // TODO: close gripper hardware here.
            open_position_ = 0.0f;
            response->message = "closed";
          }
          std_msgs::msg::Float32 pos_msg;
          pos_msg.data = open_position_;
          position_publisher_->publish(pos_msg);
          response->success = true;
        });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  float open_position_ = 0.0f;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperNode>());
  rclcpp::shutdown();
  return 0;
}
