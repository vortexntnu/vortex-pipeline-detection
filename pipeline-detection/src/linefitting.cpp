
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <ros_linefitting/linedetectorPipe.h>

using namespace std::chrono_literals;

class PoseArrayPublisher : public rclcpp::Node
{
  LinedetectorPipe pipeline;
public:
  PoseArrayPublisher() : Node("linefitting")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("linefitting_topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PoseArrayPublisher::publish_pose_array, this));

    LinedetectorPipe pipeline = LinedetectorPipe();
  }

private:
  void publish_pose_array()
  {

    cv::Mat img = cv::imread("/home/felicia/workspaces/linefittingCpp/test.png", cv::IMREAD_GRAYSCALE);
    vector<Line> lines = pipeline(img, 2);
    auto message = geometry_msgs::msg::PoseArray();
    message.header.stamp = this->now();
    message.header.frame_id = "base_link";

    // Example: Add a single pose to the PoseArray
    for (int i = 0; i < lines.size(); i+=1){
        geometry_msgs::msg::Pose pose;
        pose.position.x = lines[i].start.x;
        pose.position.y = lines[i].start.y;
        pose.position.z = 0.0;
        pose.orientation.x = lines[i].end.x;
        pose.orientation.y = lines[i].end.y;
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;
        message.poses.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(), "Publishing Lines");

    publisher_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{ 
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseArrayPublisher>());
  rclcpp::shutdown();
  return 0;
}