
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/msg/pose_array.hpp"
#include <ros_linefitting/linedetectorPipe.h>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class PoseArrayPublisher : public rclcpp::Node
{
  LinedetectorPipe pipeline;
  cv::Mat img;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;

public:
  PoseArrayPublisher() : Node("linefitting")
  {
    //Replace with empty and check if empty in execution
    img = cv::imread("/home/felicia/workspaces/linefittingCpp/test.png", cv::IMREAD_GRAYSCALE);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);


    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("linefitting_topic", qos_sensor_data);
    publisher_d = this->create_publisher<sensor_msgs::msg::Image>("lines_image", qos_sensor_data);
    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("filtered_image", qos_sensor_data, 
      std::bind(&PoseArrayPublisher::updateImage, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      3ms, std::bind(&PoseArrayPublisher::publish_pose_array, this));

    LinedetectorPipe pipeline = LinedetectorPipe();
  }

private:
  void updateImage(const sensor_msgs::msg::Image msg){
    try
    {
      // Convert ROS Image message to OpenCV image
      img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }

  void publish_pose_array()
  {

    vector<Line> lines = pipeline(img, 2);
    auto message = geometry_msgs::msg::PoseArray();
    message.header.stamp = this->now();
    message.header.frame_id = "base_link";

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

    //bridge and publish the scribled image
  auto image_message = sensor_msgs::msg::Image();
  cv_bridge::CvImage(image_message.header, "bgr8", pipeline.drawResults(img, lines)*10).toImageMsg(image_message);
  publisher_d->publish(image_message);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_d;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{ 
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseArrayPublisher>());
  rclcpp::shutdown();
  return 0;
}