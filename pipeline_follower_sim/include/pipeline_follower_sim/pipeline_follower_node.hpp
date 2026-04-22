#pragma once

#include <rclcpp/rclcpp.hpp>
#include "vortex_msgs/msg/line_segment2_d_array.hpp"
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/srv/send_waypoints.hpp>
#include <vortex_msgs/msg/dvl_altitude.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <opencv2/opencv.hpp>
#include <deque>
#include <string>

class PipelineFollowerNode : public rclcpp::Node
{
public:
  PipelineFollowerNode();

private:
  // --- Callbacks ---
  void linesCb(const vortex_msgs::msg::LineSegment2DArray::SharedPtr msg);
  void poseCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void altitudeCb(const vortex_msgs::msg::DVLAltitude::SharedPtr msg);
  void infoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void timerTick();

  // --- Waypoint sending ---
  void sendOrDebugWaypoint(double x, double y, double z, double yaw,
                           bool overwrite_prior, bool take_priority,
                           uint mode, double switching_threshold);

  struct WaypointRequest {
    vortex_msgs::msg::Waypoint wp;
    bool overwrite_prior;
    bool take_priority;
    double switching_threshold;
  };

  void enqueueWaypoint(const vortex_msgs::msg::Waypoint & wp,
                       bool overwrite_prior, bool take_priority,
                       double switching_threshold);
  void trySendNextRequest();

  // --- Line handling ---
  void handleSingleLine(const vortex_msgs::msg::LineSegment2D & line);
  void handleTwoLines(const vortex_msgs::msg::LineSegment2D & l1,
                      const vortex_msgs::msg::LineSegment2D & l2);

  // --- Geometry helpers ---
  bool isNewCorner(double x, double y) const;
  static float pointSegmentDistance(const cv::Point2f & p,
                                    const cv::Point2f & a,
                                    const cv::Point2f & b);
  bool findLineIntersection(const cv::Point2f & p1, const cv::Point2f & p2,
                            const cv::Point2f & q1, const cv::Point2f & q2,
                            cv::Point2f & out);

  // --- Members ---
  std::deque<WaypointRequest> request_queue_;
  bool request_in_flight_{false};

  std::string debug_waypoint_topic_;
  std::string debug_service_off_topic_;

  rclcpp::Publisher<vortex_msgs::msg::Waypoint>::SharedPtr debug_wp_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr service_off_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info;

  double robot_x_{0.0}, robot_y_{0.0}, robot_z_{0.0}, robot_yaw_{0.0};
  std::string input_topic_lines_;
  std::string input_topic_pose_;
  std::string input_topic_info_;
  std::string input_topic_altitude_;

  double camera_height_;
  double send_rate_hz_;
  double camera_placment_x_;
  double camera_placment_y_;
  double camera_placment_z_;
  cv::Matx33d K_;
  double robot_a_{0.0};
  double image_width;
  double image_height;
  double target_height_;

  bool have_last_corner_{false};
  double last_corner_x_{0.0};
  double last_corner_y_{0.0};
  double corner_min_separation_{0.4};

  bool have_pose_{false};
  nav_msgs::msg::Odometry latest_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose;

  bool have_lines_{false};
  vortex_msgs::msg::LineSegment2DArray latest_lines_;

  std::deque<cv::Point2f> cross_history_;

  bool have_prev_wp_ = false;
  double prev_x_ = 0.0;
  double prev_y_ = 0.0;
  double prev_z_ = 0.0;
  bool goal_blocked_ = false;

  int min_dist_skip_count_ = 0;
  static constexpr int MAX_MIN_DIST_SKIPS = 20;

  rclcpp::TimerBase::SharedPtr request_delay_timer_;
  rclcpp::Subscription<vortex_msgs::msg::LineSegment2DArray>::SharedPtr sub_line;
  rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedPtr client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr goal_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<vortex_msgs::msg::DVLAltitude>::SharedPtr sub_altitude;
};
