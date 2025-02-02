#ifndef PIPELINE_LINE_FITTING_ROS_HPP
#define PIPELINE_LINE_FITTING_ROS_HPP

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <opencv2/opencv.hpp>
#include <pipeline_line_fitting/linedetectorPipe.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

class PipelineLineFittingNode : public rclcpp::Node {

public:
  PipelineLineFittingNode(const rclcpp::NodeOptions &options);

  ~PipelineLineFittingNode() {};

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      image_visualization_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

  LinedetectorPipe pipeline;
  bool publish_visualization_;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  cv::Mat draw_lines(cv::Mat &image, const vector<Line> &lines);
};

#endif // PIPELINE_LINE_FITTING_ROS_HPP
