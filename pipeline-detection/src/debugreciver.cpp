#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_array.hpp"

class DebugReceiver : public rclcpp::Node
{
public:
  DebugReceiver() : Node("debug_receiver")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "linefitting_topic", 10, std::bind(&DebugReceiver::topic_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Started!");
  }

private:
  void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses", msg->poses.size());
    for (const auto& pose : msg->poses)
    {
      RCLCPP_INFO(this->get_logger(), "Pose: position(%f, %f, %f), orientation(%f, %f, %f, %f)",
                  pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DebugReceiver>());
  rclcpp::shutdown();
  return 0;
}