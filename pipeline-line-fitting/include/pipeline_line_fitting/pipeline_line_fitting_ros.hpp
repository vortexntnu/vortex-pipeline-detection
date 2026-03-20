#ifndef PIPELINE_LINE_FITTING__PIPELINE_LINE_FITTING_ROS_HPP_
#define PIPELINE_LINE_FITTING__PIPELINE_LINE_FITTING_ROS_HPP_

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <opencv2/opencv.hpp>
#include <pipeline_line_fitting/linedetectorPipe.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

class PipelineLineFittingNode : public rclcpp::Node {
   public:
    explicit PipelineLineFittingNode(const rclcpp::NodeOptions& options);

    ~PipelineLineFittingNode() {}

   private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imageVisualizationPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poseArrayPub_;

    LinedetectorPipe pipeline_;
    bool publishVisualization_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    RandsacParams fetchParams();

    cv::Mat drawLines(cv::Mat& image, const std::vector<Line>& lines);
};

#endif  // PIPELINE_LINE_FITTING__PIPELINE_LINE_FITTING_ROS_HPP_
