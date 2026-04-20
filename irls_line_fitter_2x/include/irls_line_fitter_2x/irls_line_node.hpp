#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "vortex_msgs/msg/line_segment2_d_array.hpp"
#include "vortex_msgs/msg/line_segment2_d.hpp"
#include "vortex_msgs/msg/point2_d.hpp"

#include "irls_line_fitter_2x/irls_utils.hpp"

#include <string>
#include <vector>

class IRLSLineNode : public rclcpp::Node {
public:
  IRLSLineNode();

private:
  LineModel fitIRLS(const std::vector<cv::Point2f>& pts);

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  void publishImage(const cv::Mat& bgr, const std_msgs::msg::Header& header);

  static vortex_msgs::msg::LineSegment2D makeSeg(const cv::Point& a, const cv::Point& b);

  void publishLines(const std_msgs::msg::Header& header,
                    const cv::Point& p1a, const cv::Point& p1b,
                    bool have_first,
                    bool have_second,
                    const cv::Point& p2a, const cv::Point& p2b);

  // Topics
  std::string input_topic_;
  std::string input_topic_info_;
  std::string output_topic_img_;
  std::string output_topic_lines_;

  // Preprocessing
  int binary_threshold_;
  int min_pixels_;

  // IRLS
  int max_iters_;
  double eps_change_;
  std::string loss_type_str_;
  double huber_delta_;
  double tukey_c_;
  bool scale_mad_;
  RobustLoss loss_;

  // Drawing (first line)
  int draw_thickness_;
  int draw_b_, draw_g_, draw_r_;
  bool publish_original_if_fail_;

  // Clipping
  bool clip_to_object_;
  double clip_max_dist_px_;

  // Second-pass controls
  bool find_second_line_;
  double removal_band_px_;
  int min_pixels_second_;
  int draw2_b_, draw2_g_, draw2_r_;
  bool draw_intersection_;
  int intersection_radius_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;
  rclcpp::Publisher<vortex_msgs::msg::LineSegment2DArray>::SharedPtr pub_lines_;
};
