#pragma once

#include <cmath>
#include <utility>
#include <optional>
#include <fstream>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "vortex_msgs/msg/line_segment2_d_array.hpp"

inline std::pair<double, double> rotateXY(double x, double y, double yaw)
{
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {c * x - s * y, s * x + c * y};
}

inline void sortByClosestX(vortex_msgs::msg::LineSegment2D & line1,
                            vortex_msgs::msg::LineSegment2D & line2)
{
    double targetX = line1.p0.x;
    double d0 = std::abs(line2.p0.x - targetX);
    double d1 = std::abs(line2.p1.x - targetX);
    if (d1 < d0) {
        std::swap(line2.p0, line2.p1);
    }
}

inline geometry_msgs::msg::Quaternion quatFromYaw(double yaw_rad)
{
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    return tf2::toMsg(q);
}

constexpr double PI = 3.14159265358979323846;

inline double normalizeAngle(double angle)
{
    while (angle > PI)  angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

inline double lineOrientation(const cv::Point2f & p1, const cv::Point2f & p2)
{
    double dx = p2.x - p1.x;
    double dy = p1.y - p2.y;
    return std::atan2(-dy, dx);
}

inline double angleBetweenLinesRad(
    double yaw,
    const cv::Point2f & p1, const cv::Point2f & p2,
    const cv::Point2f & q1, const cv::Point2f & q2,
    double single = false)
{
    double oldYaw = yaw;
    double theta1 = lineOrientation(p1, p2);
    double theta2 = lineOrientation(q1, q2);
    double angle  = normalizeAngle(theta2 - theta1);

    if (single) {
        double dx = p2.x - p1.x;
        double dy = p1.y - p2.y;
        angle = std::atan2(dx, dy);
    }

    double newYaw = yaw + angle;

    static std::ofstream logFile("angle_log.txt", std::ios::app);
    if (logFile.is_open() && !single) {
        logFile << "Line1: (" << p1.x << "," << p1.y << ") -> ("
                << p2.x << "," << p2.y << ") | ";
        logFile << "Line2: (" << q1.x << "," << q1.y << ") -> ("
                << q2.x << "," << q2.y << ") | ";
        logFile << "Angle: " << angle
                << " | OldYaw: " << oldYaw
                << " | NewYaw: " << newYaw
                << std::endl;
    }

    return normalizeAngle(yaw + angle);
}

inline double hight_regulator(double z_hight, double dvl_hight, double target_hight)
{
    return z_hight - (target_hight - dvl_hight);
}

inline float maxY(const vortex_msgs::msg::LineSegment2D & line)
{
    return std::max(line.p0.y, line.p1.y);
}

inline void ensureHighestYFirst(vortex_msgs::msg::LineSegment2D & line)
{
    if (line.p1.y > line.p0.y) {
        std::swap(line.p0, line.p1);
    }
}

inline void sortLines(std::vector<vortex_msgs::msg::LineSegment2D> & lines)
{
    if (lines.size() != 2) {
        ensureHighestYFirst(lines[0]);
        return;
    }
    ensureHighestYFirst(lines[0]);
    ensureHighestYFirst(lines[1]);
    sortByClosestX(lines[0], lines[1]);
}

inline std::optional<cv::Point2d> groundDistanceFromPixel(
    const cv::Point2d & pixel_uv,
    const cv::Matx33d K,
    double cameraHeightMeters)
{
    double fx = K(0, 0);
    double fy = K(1, 1);
    double cx = K(0, 2);
    double cy = K(1, 2);

    double X =  ((pixel_uv.x - cx) / fx) * cameraHeightMeters;
    double Y = -((pixel_uv.y - cy) / fy) * cameraHeightMeters;

    return cv::Point2d(X, Y);
}
