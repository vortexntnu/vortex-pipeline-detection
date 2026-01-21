#include <rclcpp/rclcpp.hpp>
#include "vortex_msgs/msg/line_segment2_d_array.hpp"
#include "vortex_msgs/msg/reference_filter.hpp"
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/srv/send_waypoints.hpp>

#include <opencv2/opencv.hpp>
#include <optional>
#include <deque>
#include <std_msgs/msg/bool.hpp>
#include <utility> 
#include <cmath> 

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>

inline std::pair<double, double> rotateXY(double x, double y, double yaw)
{
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    const double xr = c * x - s * y;
    const double yr = s * x + c * y;

    return {xr, yr};
}


static geometry_msgs::msg::Quaternion quatFromYaw(double yaw_rad)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_rad);
  return tf2::toMsg(q);
}

static double angleBetweenLinesDeg(const cv::Point2f& p1, const cv::Point2f& p2,
                                   const cv::Point2f& q1, const cv::Point2f& q2)
{
    cv::Point2f v1 = p2 - p1;
    cv::Point2f v2 = q2 - q1;

    double n1 = std::hypot(v1.x, v1.y);
    double n2 = std::hypot(v2.x, v2.y);

    // Degenerate line check
    if (n1 == 0.0 || n2 == 0.0) return 0.0;

    double dot = v1.x * v2.x + v1.y * v2.y;
    double cosTheta = dot / (n1 * n2);

    // Clamp for numeric safety (avoid NaNs from acos)
    cosTheta = std::max(-1.0, std::min(1.0, cosTheta));

    double rad = std::acos(cosTheta);
    return rad * 180.0 / CV_PI;
}

// Returns (X, Z) on the ground in meters:
//   X = left/right (camera x axis), Z = forward (camera z axis)
// Assumes camera is level and camera Y axis points down, ground plane is y = H.
std::optional<cv::Point2d> groundDistanceFromPixel(
    const cv::Point2d& pixel_uv,
    const cv::Matx33d& K,
    double H,
    double* groundDistance_out = nullptr // optional: sqrt(X^2 + Z^2)
) {
    // Invert intrinsics to get a ray direction in camera coordinates
    const cv::Matx33d Kinv = K.inv();
    const cv::Vec3d pix_h(pixel_uv.x, pixel_uv.y, 1.0);
    const cv::Vec3d ray = Kinv * pix_h; // (x_n, y_n, 1) up to scale

    // If ray.y <= 0, ray goes to sky / parallel to ground -> no ground hit
    const double ry = ray[1];
    if (ry <= 1e-12) {
        return std::nullopt;
    }

    // Intersect p(t)=t*ray with ground plane y = H  (Y points down)
    const double t = H / ry;

    const double X = t * ray[0];
    const double Z = t * ray[2];

    if (groundDistance_out) {
        *groundDistance_out = std::sqrt(X * X + Z * Z);
    }

    return cv::Point2d(X, Z);
}


class PipelineFollowerNode : public rclcpp::Node
{
public:
  PipelineFollowerNode()
  : Node("pipeline_follower_node")
  {
    // Parameters
    input_topic_lines_ = this->declare_parameter<std::string>("input_topic_lines", "irls_line/lines");
    input_topic_pose_ = this->declare_parameter<std::string>("input_topic_pose", "/orca/odom");
    camera_height_ = this->declare_parameter<double>("camera_height", 0.5);
    send_rate_hz_ = this->declare_parameter<double>("send_rate_hz", 5.0);

    debug_waypoint_topic_ = this->declare_parameter<std::string>("debug_waypoint_topic", "/debug/waypoint");
    debug_service_off_topic_ = this->declare_parameter<std::string>("debug_service_off_topic", "/debug/send_waypoints_service_off");

    debug_wp_pub_ = this->create_publisher<vortex_msgs::msg::Waypoint>(debug_waypoint_topic_, 10);
    service_off_pub_ = this->create_publisher<std_msgs::msg::Bool>(debug_service_off_topic_, 10);


    // Example camera intrinsics (update for your camera)
    double fx = 900, fy = 900, cx = 720, cy = 540;
    K_ = cv::Matx33d(fx, 0, cx,
                     0, fy, cy,
                     0, 0, 1);

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

    client_ = this->create_client<vortex_msgs::srv::SendWaypoints>("/send_waypoints");

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / send_rate_hz_),
      std::bind(&PipelineFollowerNode::timerTick, this)
    );
  }

private:
  // --- Core logic ---

  void linesCb(const vortex_msgs::msg::LineSegment2DArray::SharedPtr msg)
  {
    latest_lines_ = *msg;
    have_lines_ = true;
  }

  void poseCb(const nav_msgs::msg::Odometry::SharedPtr msg)
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


  void timerTick()
  {

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                       "tick: have_lines=%d have_pose=%d", have_lines_, have_pose_);

    if (!have_lines_ || !have_pose_) return;

    const bool service_ready = client_->service_is_ready();
    std_msgs::msg::Bool off_msg;
    off_msg.data = !service_ready;
    service_off_pub_->publish(off_msg);

    if (!service_ready) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "/send_waypoints service not ready (server off). Publishing debug waypoints instead.");
    }


    const auto &lines = latest_lines_.lines; // adjust if array field name differs

    if (lines.size() == 1)
    {
      handleSingleLine(lines[0]);
    }
    else if (lines.size() == 2)
    {
      handleTwoLines(lines[0], lines[1]);
    }
  }

  void sendOrDebugWaypoint(double x, double y, double yaw,
                           bool overwrite_prior, bool take_priority)
  {
    // Build the waypoint once (used for both service + debug publish)
    vortex_msgs::msg::Waypoint wp;
    wp.pose.position.x = x;
    wp.pose.position.y = y;
    wp.pose.position.z = robot_z_;
    wp.pose.orientation = quatFromYaw(yaw);
    wp.mode = vortex_msgs::msg::Waypoint::FULL_POSE;

    // If service is off, publish debug and return
    if (!client_->service_is_ready())
    {
      debug_wp_pub_->publish(wp);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                       "tick: sendt debug waypoint");
      return;
    }

    // Otherwise send to service
    auto req = std::make_shared<vortex_msgs::srv::SendWaypoints::Request>();
    req->waypoints = {wp};
    req->switching_threshold = 1.0;
    req->overwrite_prior_waypoints = overwrite_prior;
    req->take_priority = take_priority;

    client_->async_send_request(
      req,
      [this, wp](rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedFuture future) {
        try {
          auto resp = future.get();
          RCLCPP_INFO(this->get_logger(), "Sent waypoint: success=%d", resp->success);

          // If it failed logically, also publish it for debugging
          if (!resp->success) {
            debug_wp_pub_->publish(wp);
          }
        } catch (...) {
          RCLCPP_WARN(this->get_logger(), "Waypoint send failed (exception). Publishing debug waypoint.");
          debug_wp_pub_->publish(wp);
        }
      }
    );
  }


  // --- 1 line case ---
  void handleSingleLine(const vortex_msgs::msg::LineSegment2D &line)
  {
    cv::Point2f p1(line.p0.x, line.p0.y);
    cv::Point2f p2(line.p1.x, line.p1.y);

    // Take the farther endpoint as the "end"
    cv::Point2f end = (cv::norm(p2) > cv::norm(p1)) ? p2 : p1;

    cv::Point2d groundXZ;
    double dist = 0.5;
    auto point_opt = groundDistanceFromPixel(end, K_, camera_height_, &dist);
    if (!point_opt) return;

    const auto &XZ = *point_opt;   // XZ.x = X (left/right), XZ.y = Z (forward)

    // Example mapping: world x += forward, world y += left/right
    auto [xr, xy] =rotateXY(XZ.y, XZ.x, robot_yaw_);

    double x_in_meters = robot_x_ + xr;
    double y_in_meters = robot_y_ + xy;

    sendOrDebugWaypoint(x_in_meters, y_in_meters, robot_yaw_, true, false);


  }

  // --- 2 line case ---
  void handleTwoLines(const vortex_msgs::msg::LineSegment2D &l1,
                    const vortex_msgs::msg::LineSegment2D &l2)
  {
    cv::Point2f p1(l1.p0.x, l1.p0.y), p2(l1.p1.x, l1.p1.y);
    cv::Point2f q1(l2.p0.x, l2.p0.y), q2(l2.p1.x, l2.p1.y);

    cv::Point2f cross;
    if (!findLineIntersection(p1, p2, q1, q2, cross))
      return;

    // Keep last 3 intersections to check stability
    cross_history_.push_back(cross);
    if (cross_history_.size() > 3)
      cross_history_.pop_front();

    if (cross_history_.size() == 3)
    {
      double avg_dist = 0.0;
      for (size_t i = 0; i < cross_history_.size() - 1; ++i)
        avg_dist += cv::norm(cross_history_[i] - cross_history_[i + 1]);
      avg_dist /= 2.0;

      if (avg_dist < 10.0)  // pixels threshold for "close proximity"
      {
        double dist;
        auto point_opt = groundDistanceFromPixel(cv::Point2d(cross.x, cross.y), K_, camera_height_, &dist);
        if (!point_opt) return;

        const auto &XZ = *point_opt;   // XZ.x = X (left/right), XZ.y = Z (forward)

        // Example mapping: world x += forward, world y += left/right
        auto [xr, xy] =rotateXY(XZ.y, XZ.x, robot_yaw_);

        double x_in_meters = robot_x_ + xr;
        double y_in_meters = robot_y_ + xy;

        sendOrDebugWaypoint(x_in_meters, y_in_meters, robot_yaw_, true, false);

        // Angle between lines
        double angle_deg = angleBetweenLinesDeg(p1, p2, q1, q2);
        double yaw_rad = robot_yaw_ + angle_deg * CV_PI / 180.0;
        sendOrDebugWaypoint(x_in_meters, y_in_meters, yaw_rad, false, true);
        return;
      }
    }

    // Fallback: just use first line’s end
    handleSingleLine(l1);
  }

  // --- Geometry helper ---
  bool findLineIntersection(const cv::Point2f &p1, const cv::Point2f &p2,
                            const cv::Point2f &q1, const cv::Point2f &q2,
                            cv::Point2f &out)
  {
    cv::Point2f r = p2 - p1;
    cv::Point2f s = q2 - q1;
    float denom = r.x * s.y - r.y * s.x;
    if (std::fabs(denom) < 1e-6)
      return false; // parallel

    float t = ((q1 - p1).x * s.y - (q1 - p1).y * s.x) / denom;
    out = p1 + t * r;
    return true;
  }
  

  // --- Service sender ---
  void sendWaypoint(double x, double y, double yaw,
                    bool overwrite_prior, bool take_priority)
  {
    auto req = std::make_shared<vortex_msgs::srv::SendWaypoints::Request>();
    vortex_msgs::msg::Waypoint wp;

    wp.pose.position.x = x;
    wp.pose.position.y = y;
    wp.pose.position.z = robot_z_;
    wp.pose.orientation = quatFromYaw(yaw);
    wp.mode = vortex_msgs::msg::Waypoint::FULL_POSE;

    req->waypoints = {wp};
    req->switching_threshold = 1.0;
    req->overwrite_prior_waypoints = overwrite_prior;
    req->take_priority = take_priority;

    client_->async_send_request(
    req,
    [this](rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedFuture future) {
      try {
        auto resp = future.get();  // resp is SendWaypoints::Response::SharedPtr
        RCLCPP_INFO(this->get_logger(), "Sent waypoint: success=%d", resp->success);
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Waypoint send failed");
      }
    }
    );

  }

  // --- Members ---
  std::string debug_waypoint_topic_;
  std::string debug_service_off_topic_;

  rclcpp::Publisher<vortex_msgs::msg::Waypoint>::SharedPtr debug_wp_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr service_off_pub_;


  double robot_x_{0.0}, robot_y_{0.0}, robot_z_{0.0}, robot_yaw_{0.0};
  std::string input_topic_lines_;
  std::string input_topic_pose_;
  double camera_height_;
  double send_rate_hz_;
  cv::Matx33d K_;

  bool have_pose_{false};
  nav_msgs::msg::Odometry latest_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose;


  bool have_lines_{false};
  vortex_msgs::msg::LineSegment2DArray latest_lines_;
  
  std::deque<cv::Point2f> cross_history_;

  rclcpp::Subscription<vortex_msgs::msg::LineSegment2DArray>::SharedPtr sub_line;
  rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// --- main ---
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PipelineFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
