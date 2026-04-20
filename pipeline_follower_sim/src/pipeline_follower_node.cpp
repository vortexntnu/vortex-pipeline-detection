#include "pipeline_follower_sim/pipeline_follower_node.hpp"
#include "pipeline_follower_sim/pipeline_follower_utils.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

// --- Constructor ---

PipelineFollowerNode::PipelineFollowerNode()
: Node("pipeline_follower_node")
{
  input_topic_lines_    = this->declare_parameter<std::string>("input_topic_lines", "irls_line/lines");
  input_topic_info_     = this->declare_parameter<std::string>("input_topic_info", "/pipeline/camera/camera_info");
  input_topic_pose_     = this->declare_parameter<std::string>("input_topic_pose", "/orca/odom");
  input_topic_altitude_ = this->declare_parameter<std::string>("input_topic_altitude", "/orca/dvl/altitude");
  camera_height_        = this->declare_parameter<double>("camera_height", 0.5);
  send_rate_hz_         = this->declare_parameter<double>("send_rate_hz", 0.90);
  camera_placment_x_    = this->declare_parameter<double>("camera_placment_x", 0.4);

  debug_waypoint_topic_     = this->declare_parameter<std::string>("debug_waypoint_topic", "/debug/waypoint");
  debug_service_off_topic_  = this->declare_parameter<std::string>("debug_service_off_topic", "/debug/send_waypoints_service_off");

  debug_wp_pub_   = this->create_publisher<vortex_msgs::msg::Waypoint>(debug_waypoint_topic_, 10);
  service_off_pub_ = this->create_publisher<std_msgs::msg::Bool>(debug_service_off_topic_, 10);

  sub_line = this->create_subscription<vortex_msgs::msg::LineSegment2DArray>(
    input_topic_lines_,
    rclcpp::SensorDataQoS(),
    std::bind(&PipelineFollowerNode::linesCb, this, std::placeholders::_1)
  );

  sub_pose = this->create_subscription<nav_msgs::msg::Odometry>(
    input_topic_pose_,
    rclcpp::SensorDataQoS(),
    std::bind(&PipelineFollowerNode::poseCb, this, std::placeholders::_1)
  );

  sub_altitude = this->create_subscription<vortex_msgs::msg::DVLAltitude>(
    input_topic_altitude_,
    rclcpp::SensorDataQoS(),
    std::bind(&PipelineFollowerNode::altitudeCb, this, std::placeholders::_1)
  );

  sub_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    input_topic_info_,
    rclcpp::SensorDataQoS(),
    std::bind(&PipelineFollowerNode::infoCb, this, std::placeholders::_1)
  );

  client_ = this->create_client<vortex_msgs::srv::SendWaypoints>("/orca/waypoint_addition");

  goal_service_ = this->create_service<std_srvs::srv::Trigger>(
    "pipline/goal_service",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr,
           std_srvs::srv::Trigger::Response::SharedPtr response) {
      goal_blocked_ = !goal_blocked_;
      if (goal_blocked_) {
        RCLCPP_INFO(this->get_logger(), "Goal reached — blocking execution");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal cleared — resuming execution");
      }
      response->success = goal_blocked_;
      response->message = goal_blocked_ ? "blocked" : "cleared";
    });

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / send_rate_hz_),
    std::bind(&PipelineFollowerNode::timerTick, this)
  );
}

// --- Callbacks ---

void PipelineFollowerNode::linesCb(const vortex_msgs::msg::LineSegment2DArray::SharedPtr msg)
{
  latest_lines_ = *msg;
  have_lines_   = true;
}

void PipelineFollowerNode::poseCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_z_ = msg->pose.pose.position.z;

  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  robot_yaw_ = yaw;
  RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000, "yaw = %f", robot_yaw_);
  have_pose_ = true;
}

void PipelineFollowerNode::altitudeCb(const vortex_msgs::msg::DVLAltitude::SharedPtr msg)
{
  robot_a_ = msg->altitude;
}

void PipelineFollowerNode::infoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  K_ = cv::Matx33d(
    msg->k[0], msg->k[1], msg->k[2],
    msg->k[3], msg->k[4], msg->k[5],
    msg->k[6], msg->k[7], msg->k[8]
  );
  image_width  = msg->width;
  image_height = msg->height;
}

void PipelineFollowerNode::timerTick()
{
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
    "tick: have_lines=%d have_pose=%d", have_lines_, have_pose_);

  if (!have_lines_ || !have_pose_) { return; }

  if (goal_blocked_) { return; }

  const bool service_ready = client_->service_is_ready();
  std_msgs::msg::Bool off_msg;
  off_msg.data = !service_ready;
  service_off_pub_->publish(off_msg);

  const auto & lines = latest_lines_.lines;

  if (lines.size() == 1) {
    auto sorted_lines = lines;
    sortLines(sorted_lines);
    handleSingleLine(sorted_lines[0]);
  } else if (lines.size() == 2) {
    auto sorted_lines = lines;
    sortLines(sorted_lines);
    handleTwoLines(sorted_lines[0], sorted_lines[1]);
  }
}

// --- Waypoint sending ---

void PipelineFollowerNode::sendOrDebugWaypoint(
  double x, double y, double z, double yaw,
  bool overwrite_prior, bool take_priority,
  uint mode, double switching_threshold)
{
  constexpr double MIN_WP_DIST = 0.15;

  if (have_prev_wp_) {
    const double dx   = x - prev_x_;
    const double dy   = y - prev_y_;
    const double dz   = z - prev_z_;
    const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (dist < MIN_WP_DIST) {
      min_dist_skip_count_++;
      if (min_dist_skip_count_ < MAX_MIN_DIST_SKIPS) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waypoint skipped (distance %.3f m < %.2f m) (%d/%d)",
          dist, MIN_WP_DIST, min_dist_skip_count_, MAX_MIN_DIST_SKIPS);
        return;
      }
      RCLCPP_WARN(get_logger(),
        "Waypoint forced after %d consecutive min-dist skips (%.3f m)",
        min_dist_skip_count_, dist);
      min_dist_skip_count_ = 0;
    } else {
      min_dist_skip_count_ = 0;
    }
  }

  vortex_msgs::msg::Waypoint wp;
  wp.pose.position.x  = x;
  wp.pose.position.y  = y;
  wp.pose.position.z  = z;
  wp.pose.orientation = quatFromYaw(yaw);
  wp.mode             = mode;

  if (!client_->service_is_ready()) {
    debug_wp_pub_->publish(wp);
    prev_x_ = x; prev_y_ = y; prev_z_ = z;
    have_prev_wp_ = true;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "tick: sent debug waypoint");
    return;
  }

  auto req = std::make_shared<vortex_msgs::srv::SendWaypoints::Request>();
  req->waypoints                = {wp};
  req->switching_threshold      = switching_threshold;
  req->overwrite_prior_waypoints = overwrite_prior;
  req->take_priority            = take_priority;

  client_->async_send_request(
    req,
    [this, wp](rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedFuture future) {
      try {
        auto resp = future.get();
        RCLCPP_WARN(this->get_logger(), "Sent waypoint: success=%d", resp->success);
        prev_x_ = wp.pose.position.x;
        prev_y_ = wp.pose.position.y;
        prev_z_ = wp.pose.position.z;
        have_prev_wp_ = true;
        debug_wp_pub_->publish(wp);
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Waypoint send failed (exception). Publishing debug waypoint.");
      }
    });
}

void PipelineFollowerNode::enqueueWaypoint(
  const vortex_msgs::msg::Waypoint & wp,
  bool overwrite_prior, bool take_priority, double switching_threshold)
{
  debug_wp_pub_->publish(wp);
  request_queue_.push_back({wp, overwrite_prior, take_priority, switching_threshold});
  trySendNextRequest();
}

void PipelineFollowerNode::trySendNextRequest()
{
  if (request_in_flight_ || request_queue_.empty()) { return; }

  if (!client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Service not ready.");
    return;
  }

  request_in_flight_ = true;
  auto item = request_queue_.front();
  request_queue_.pop_front();

  auto req = std::make_shared<vortex_msgs::srv::SendWaypoints::Request>();
  req->waypoints                = {item.wp};
  req->overwrite_prior_waypoints = item.overwrite_prior;
  req->take_priority            = item.take_priority;
  req->switching_threshold      = item.switching_threshold;

  client_->async_send_request(
    req,
    [this](rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedFuture future) {
      request_in_flight_ = false;
      try {
        auto resp = future.get();
        RCLCPP_INFO(this->get_logger(), "Waypoint sent. success=%d", resp->success);
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Waypoint send failed.");
      }
      request_delay_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        [this]() {
          request_delay_timer_->cancel();
          trySendNextRequest();
        });
    });
}

// --- Line handling ---

void PipelineFollowerNode::handleSingleLine(const vortex_msgs::msg::LineSegment2D & line)
{
  RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000, "starting single line handeling");
  cv::Point2f p1(line.p0.x, line.p0.y);
  cv::Point2f p2(line.p1.x, line.p1.y);

  cv::Point2f end = (p1.y < p2.y) ? p1 : p2;

  auto point_opt = groundDistanceFromPixel(end, K_, robot_a_);
  if (!point_opt) { return; }

  const auto & ground = *point_opt;
  double right   = ground.x;
  double forward = ground.y;

  auto [dx, dy] = rotateXY(forward, right, robot_yaw_);
  auto [xc, yc] = rotateXY(camera_placment_x_, 0, robot_yaw_);
  auto new_robot_z = hight_regulator(robot_a_, robot_z_);

  double x_in_meters = robot_x_ + dx + xc;
  double y_in_meters = robot_y_ + dy + yc;

  if (abs(p1.x - p2.x) >= image_width / 7) {
    double newYaw = angleBetweenLinesRad(robot_yaw_, p1, p2, cv::Point2f(0, 0), cv::Point2f(1, 1), true);
    if (abs(abs(robot_yaw_) - abs(newYaw)) > 1.35) {
      sendOrDebugWaypoint(x_in_meters, y_in_meters, new_robot_z, robot_yaw_,
        true, false, vortex_msgs::msg::Waypoint::FORWARD_HEADING, 0.3);
      return;
    }
    sendOrDebugWaypoint(robot_x_, robot_y_, new_robot_z, newYaw,
      true, false, vortex_msgs::msg::Waypoint::ONLY_ORIENTATION, 0.3);
  } else {
    sendOrDebugWaypoint(x_in_meters, y_in_meters, new_robot_z, robot_yaw_,
      true, false, vortex_msgs::msg::Waypoint::FORWARD_HEADING, 0.3);
  }
}

void PipelineFollowerNode::handleTwoLines(
  const vortex_msgs::msg::LineSegment2D & l1,
  const vortex_msgs::msg::LineSegment2D & l2)
{
  RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000, "starting two lines handeling");
  cv::Point2f p1(l1.p0.x, l1.p0.y), p2(l1.p1.x, l1.p1.y);
  cv::Point2f q1(l2.p0.x, l2.p0.y), q2(l2.p1.x, l2.p1.y);

  cv::Point2f cross;
  if (!findLineIntersection(p1, p2, q1, q2, cross)) {
    RCLCPP_WARN(this->get_logger(), "didnt find intersection");
    return;
  }

  cross_history_.push_back(cross);
  if (cross_history_.size() > 3) { cross_history_.pop_front(); }

  if (cross_history_.size() == 3) {
    double avg_dist = 0.0;
    for (size_t i = 0; i < cross_history_.size() - 1; ++i)
      avg_dist += cv::norm(cross_history_[i] - cross_history_[i + 1]);
    avg_dist /= 2.0;

    if (avg_dist < 80.0 & cross.y < 300) {
      auto point_opt = groundDistanceFromPixel(cv::Point2d(cross.x, cross.y), K_, robot_a_);
      auto ray = K_.inv() * cv::Vec3d(cross.x, cross.y, 1.0);
      if (!point_opt) {
        RCLCPP_WARN(this->get_logger(), "did not hav point_opt ray.y = %.6f, pixel y = %.1f", ray[1], cross.y);
        return;
      }

      const auto & ground = *point_opt;
      double right   = ground.x;
      double forward = ground.y;

      auto [dx, dy] = rotateXY(forward, right, robot_yaw_);
      auto [xc, yc] = rotateXY(camera_placment_x_, 0, robot_yaw_);

      double x_in_meters = robot_x_ + dx + xc;
      double y_in_meters = robot_y_ + dy + yc;

      if (!isNewCorner(x_in_meters, y_in_meters)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
          "Corner suppressed (too close to previous)");
        return;
      }

      last_corner_x_  = x_in_meters;
      last_corner_y_  = y_in_meters;
      have_last_corner_ = true;
      auto new_robot_z = hight_regulator(robot_a_, robot_z_);

      double angle_rad = angleBetweenLinesRad(robot_yaw_, p1, p2, q1, q2);
      double yaw_rad   = angle_rad;

      vortex_msgs::msg::Waypoint wp1;
      wp1.pose.position.x  = x_in_meters;
      wp1.pose.position.y  = y_in_meters;
      wp1.pose.position.z  = new_robot_z;
      wp1.pose.orientation = quatFromYaw(robot_yaw_);
      wp1.mode             = vortex_msgs::msg::Waypoint::FULL_POSE;

      vortex_msgs::msg::Waypoint wp2;
      wp2.pose.position.x  = x_in_meters;
      wp2.pose.position.y  = y_in_meters;
      wp2.pose.position.z  = new_robot_z;
      wp2.pose.orientation = quatFromYaw(yaw_rad);
      wp2.mode             = vortex_msgs::msg::Waypoint::FULL_POSE;

      double xf = x_in_meters + 0.5 * std::cos(yaw_rad);
      double yf = y_in_meters + 0.5 * std::sin(yaw_rad);

      vortex_msgs::msg::Waypoint wp3;
      wp3.pose.position.x  = xf;
      wp3.pose.position.y  = yf;
      wp3.pose.position.z  = new_robot_z;
      wp3.pose.orientation = quatFromYaw(yaw_rad);
      wp3.mode             = vortex_msgs::msg::Waypoint::FULL_POSE;

      enqueueWaypoint(wp1, true,  true, 0.1);
      enqueueWaypoint(wp2, false, true, 0.1);
      enqueueWaypoint(wp3, false, true, 0.1);
      return;
    }
    cross_history_.clear();
  }

  RCLCPP_WARN(this->get_logger(), "did not have a consistent corner, dooing single line");
  handleSingleLine(l1);
}

// --- Geometry helpers ---

bool PipelineFollowerNode::isNewCorner(double x, double y) const
{
  if (!have_last_corner_) { return true; }
  double dx   = x - last_corner_x_;
  double dy   = y - last_corner_y_;
  return std::hypot(dx, dy) > corner_min_separation_;
}

float PipelineFollowerNode::pointSegmentDistance(
  const cv::Point2f & p, const cv::Point2f & a, const cv::Point2f & b)
{
  cv::Point2f ab    = b - a;
  float       abLen2 = ab.dot(ab);
  if (abLen2 < 1e-6f) { return cv::norm(p - a); }
  float t = (p - a).dot(ab) / abLen2;
  t = std::max(0.f, std::min(1.f, t));
  return cv::norm(p - (a + t * ab));
}

bool PipelineFollowerNode::findLineIntersection(
  const cv::Point2f & p1, const cv::Point2f & p2,
  const cv::Point2f & q1, const cv::Point2f & q2,
  cv::Point2f & out)
{
  RCLCPP_WARN(this->get_logger(), "looking for intersection");

  cv::Point2f r = p2 - p1;
  cv::Point2f s = q2 - q1;

  float denom = r.x * s.y - r.y * s.x;
  if (std::fabs(denom) < 1e-6f) {
    RCLCPP_WARN(this->get_logger(), "parallel or degenerate");
    return false;
  }

  cv::Point2f qp = q1 - p1;
  float t = (qp.x * s.y - qp.y * s.x) / denom;

  cv::Point2f intersection = p1 + t * r;
  RCLCPP_WARN(this->get_logger(), "intersection = (%.1f, %.1f)", intersection.x, intersection.y);

  out = intersection;
  return true;
}

// --- main ---

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PipelineFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
