#include "vortex_pipeline_3d/visualization/marker_publisher.hpp"
#include <cmath>

namespace vortex_pipeline_3d {

MarkerPublisher::MarkerPublisher(
    rclcpp::Node* node,
    const std::string& marker_topic,
    const std::string& frame_id)
    : node_(node),
      frame_id_(frame_id),
      marker_id_counter_(0) {

  pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic, 10);

  RCLCPP_INFO(node_->get_logger(),
      "Marker publisher initialized: topic=%s, frame=%s",
      marker_topic.c_str(), frame_id_.c_str());
}

void MarkerPublisher::publishEndpoints(
    const std::vector<cv::Point3d>& endpoints_3d,
    const cv::Point3d& selected_3d,
    double dvl_altitude) {

  if (endpoints_3d.size() != 2) {
    RCLCPP_WARN(node_->get_logger(),
        "MarkerPublisher: Expected 2 endpoints, got %zu", endpoints_3d.size());
    return;
  }

  marker_id_counter_ = 0;
  visualization_msgs::msg::MarkerArray markers;

  // Add all visualization elements
  addEndpointMarkers(markers, endpoints_3d, selected_3d);
  addGroundPlaneMarker(markers, dvl_altitude);
  addCameraAxesMarker(markers);
  addCameraFrustumMarker(markers, dvl_altitude);
  addRayMarkers(markers, endpoints_3d);

  pub_->publish(markers);
}

visualization_msgs::msg::Marker MarkerPublisher::createBaseMarker(int id) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = node_->now();
  marker.ns = "pipeline_localization";
  marker.id = id;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.lifetime = rclcpp::Duration::from_seconds(0); // Never expire
  return marker;
}

void MarkerPublisher::addEndpointMarkers(
    visualization_msgs::msg::MarkerArray& markers,
    const std::vector<cv::Point3d>& endpoints_3d,
    const cv::Point3d& selected_3d) {

  // Draw both endpoints as blue spheres
  for (size_t i = 0; i < endpoints_3d.size(); ++i) {
    auto marker = createBaseMarker(marker_id_counter_++);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.pose.position.x = endpoints_3d[i].x;
    marker.pose.position.y = endpoints_3d[i].y;
    marker.pose.position.z = endpoints_3d[i].z;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 0.8;
    markers.markers.push_back(marker);
  }

  // Highlight selected endpoint with larger green sphere
  auto selected_marker = createBaseMarker(marker_id_counter_++);
  selected_marker.type = visualization_msgs::msg::Marker::SPHERE;
  selected_marker.pose.position.x = selected_3d.x;
  selected_marker.pose.position.y = selected_3d.y;
  selected_marker.pose.position.z = selected_3d.z;
  selected_marker.scale.x = 0.15;
  selected_marker.scale.y = 0.15;
  selected_marker.scale.z = 0.15;
  selected_marker.color.r = 0.0;
  selected_marker.color.g = 1.0;
  selected_marker.color.b = 0.0;
  selected_marker.color.a = 0.6;
  markers.markers.push_back(selected_marker);
}

void MarkerPublisher::addGroundPlaneMarker(
    visualization_msgs::msg::MarkerArray& markers,
    double altitude) {

  auto marker = createBaseMarker(marker_id_counter_++);
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.scale.x = 0.01; // Line width

  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.5;

  // Create a grid at Y = altitude
  // Grid size: 5m x 5m, centered around camera projection
  const double grid_size = 5.0;
  const double grid_spacing = 0.5;
  const int num_lines = static_cast<int>(grid_size / grid_spacing);

  // Lines parallel to X-axis (varying Z)
  for (int i = -num_lines / 2; i <= num_lines / 2; ++i) {
    geometry_msgs::msg::Point p1, p2;
    p1.x = -grid_size / 2;
    p1.y = altitude;
    p1.z = i * grid_spacing;

    p2.x = grid_size / 2;
    p2.y = altitude;
    p2.z = i * grid_spacing;

    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  // Lines parallel to Z-axis (varying X)
  for (int i = -num_lines / 2; i <= num_lines / 2; ++i) {
    geometry_msgs::msg::Point p1, p2;
    p1.x = i * grid_spacing;
    p1.y = altitude;
    p1.z = -grid_size / 2;

    p2.x = i * grid_spacing;
    p2.y = altitude;
    p2.z = grid_size / 2;

    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  markers.markers.push_back(marker);
}

void MarkerPublisher::addCameraAxesMarker(
    visualization_msgs::msg::MarkerArray& markers) {

  // X-axis (right) - Red arrow
  {
    auto marker = createBaseMarker(marker_id_counter_++);
    marker.type = visualization_msgs::msg::Marker::ARROW;

    geometry_msgs::msg::Point start, end;
    start.x = 0.0; start.y = 0.0; start.z = 0.0;
    end.x = 1.0; end.y = 0.0; end.z = 0.0;

    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.05; // Shaft diameter
    marker.scale.y = 0.1;  // Head diameter
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    markers.markers.push_back(marker);
  }

  // Y-axis (down) - Green arrow
  {
    auto marker = createBaseMarker(marker_id_counter_++);
    marker.type = visualization_msgs::msg::Marker::ARROW;

    geometry_msgs::msg::Point start, end;
    start.x = 0.0; start.y = 0.0; start.z = 0.0;
    end.x = 0.0; end.y = 1.0; end.z = 0.0;

    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.05;
    marker.scale.y = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    markers.markers.push_back(marker);
  }

  // Z-axis (forward) - Blue arrow (longer since it's primary viewing direction)
  {
    auto marker = createBaseMarker(marker_id_counter_++);
    marker.type = visualization_msgs::msg::Marker::ARROW;

    geometry_msgs::msg::Point start, end;
    start.x = 0.0; start.y = 0.0; start.z = 0.0;
    end.x = 0.0; end.y = 0.0; end.z = 2.0;

    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.05;
    marker.scale.y = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;

    markers.markers.push_back(marker);
  }
}

void MarkerPublisher::addCameraFrustumMarker(
    visualization_msgs::msg::MarkerArray& markers,
    double altitude) {

  auto marker = createBaseMarker(marker_id_counter_++);
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.scale.x = 0.02; // Line width

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0; // Cyan
  marker.color.a = 0.6;

  // Simplified camera frustum
  // Assume typical camera FOV: ~60 degrees horizontal, ~45 degrees vertical
  // Draw frustum extending to Z = 3m
  const double frustum_depth = 3.0;
  const double fov_h = 60.0 * M_PI / 180.0; // radians
  const double fov_v = 45.0 * M_PI / 180.0;

  double half_width = frustum_depth * std::tan(fov_h / 2.0);
  double half_height = frustum_depth * std::tan(fov_v / 2.0);

  // Four corners at Z = frustum_depth
  std::vector<cv::Point3d> corners = {
    cv::Point3d(-half_width, -half_height, frustum_depth),  // Top-left
    cv::Point3d(half_width, -half_height, frustum_depth),   // Top-right
    cv::Point3d(half_width, half_height, frustum_depth),    // Bottom-right
    cv::Point3d(-half_width, half_height, frustum_depth)    // Bottom-left
  };

  cv::Point3d origin(0, 0, 0);

  // Draw lines from origin to each corner
  for (const auto& corner : corners) {
    geometry_msgs::msg::Point p1, p2;
    p1.x = origin.x; p1.y = origin.y; p1.z = origin.z;
    p2.x = corner.x; p2.y = corner.y; p2.z = corner.z;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  // Draw rectangle at frustum depth
  for (size_t i = 0; i < corners.size(); ++i) {
    size_t next = (i + 1) % corners.size();
    geometry_msgs::msg::Point p1, p2;
    p1.x = corners[i].x; p1.y = corners[i].y; p1.z = corners[i].z;
    p2.x = corners[next].x; p2.y = corners[next].y; p2.z = corners[next].z;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  markers.markers.push_back(marker);
}

void MarkerPublisher::addRayMarkers(
    visualization_msgs::msg::MarkerArray& markers,
    const std::vector<cv::Point3d>& endpoints_3d) {

  // Draw rays from camera origin to each endpoint
  for (size_t i = 0; i < endpoints_3d.size(); ++i) {
    auto marker = createBaseMarker(marker_id_counter_++);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale.x = 0.01; // Line width

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0; // Yellow
    marker.color.a = 0.5;

    geometry_msgs::msg::Point origin, endpoint;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;

    endpoint.x = endpoints_3d[i].x;
    endpoint.y = endpoints_3d[i].y;
    endpoint.z = endpoints_3d[i].z;

    marker.points.push_back(origin);
    marker.points.push_back(endpoint);

    markers.markers.push_back(marker);
  }
}

} // namespace vortex_pipeline_3d
