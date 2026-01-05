#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <random>
#include <optional>

struct LineABC {
  float a, b, c; // ax + by + c = 0 (normalized so sqrt(a^2+b^2)=1)
};

static std::optional<LineABC> lineFromTwoPoints(const cv::Point2f& p1, const cv::Point2f& p2) {
  if (cv::norm(p1 - p2) < 1e-6f) return std::nullopt;
  float x1 = p1.x, y1 = p1.y;
  float x2 = p2.x, y2 = p2.y;
  float a = (y2 - y1);
  float b = -(x2 - x1);
  float c = (x2 - x1) * y1 - (y2 - y1) * x1;
  float n = std::hypot(a, b);
  if (n < 1e-9f) return std::nullopt;
  return LineABC{a / n, b / n, c / n};
}

class RansacLineNode : public rclcpp::Node {
public:
  RansacLineNode() : Node("ransac_line_node") {
    // Parameters
    input_topic_  = declare_parameter<std::string>("input_topic", "/segmented_image");
    output_topic_ = declare_parameter<std::string>("output_topic", "/ransac_line/image");
    binary_threshold_       = declare_parameter<int>("binary_threshold", 200);
    min_pixels_             = declare_parameter<int>("min_pixels", 200);
    iterations_             = declare_parameter<int>("iterations", 800);
    distance_threshold_px_  = declare_parameter<double>("distance_threshold_px", 2.5);
    min_inliers_            = declare_parameter<int>("min_inliers", 150);
    draw_thickness_         = declare_parameter<int>("draw_thickness", 3);
    draw_b_                 = declare_parameter<int>("draw_b", 0);
    draw_g_                 = declare_parameter<int>("draw_g", 0);
    draw_r_                 = declare_parameter<int>("draw_r", 255);
    publish_original_if_fail_ = declare_parameter<bool>("publish_original_if_fail", true);

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RansacLineNode::imageCb, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

    RCLCPP_INFO(get_logger(), "RANSAC line node ready. Subscribing: %s -> Publishing: %s",
                input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat cv_img;
    try {
      cv_img = cv_bridge::toCvShare(msg, msg->encoding)->image.clone();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
      return;
    }

    cv::Mat gray, color_for_draw;
    if (cv_img.channels() == 3) {
      cv::cvtColor(cv_img, gray, cv::COLOR_BGR2GRAY);
      color_for_draw = cv_img.clone();
    } else {
      gray = cv_img;
      cv::cvtColor(gray, color_for_draw, cv::COLOR_GRAY2BGR);
    }

    const int h = gray.rows, w = gray.cols;

    cv::Mat binary;
    cv::threshold(gray, binary, binary_threshold_, 255, cv::THRESH_BINARY);

    std::vector<cv::Point> nz;
    cv::findNonZero(binary, nz);

    if (static_cast<int>(nz.size()) < min_pixels_) {
      if (publish_original_if_fail_) publish(color_for_draw, msg->header);
      RCLCPP_WARN(get_logger(), "Not enough white pixels for RANSAC: %zu", nz.size());
      return;
    }

    std::vector<cv::Point2f> points;
    points.reserve(nz.size());
    for (const auto& p : nz) points.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));

    if (points.size() < 2) {
      publish(color_for_draw, msg->header);
      return;
    }

    // --- RANSAC ---
    std::mt19937 rng{123456u};
    std::uniform_int_distribution<size_t> dist(0, points.size() - 1);

    int best_count = -1;
    std::vector<int> best_inliers_idx;

    for (int it = 0; it < iterations_; ++it) {
      size_t i1 = dist(rng), i2 = dist(rng);
      if (i1 == i2) continue;
      auto l = lineFromTwoPoints(points[i1], points[i2]);
      if (!l) continue;

      std::vector<int> inliers_idx;
      inliers_idx.reserve(points.size());

      for (size_t i = 0; i < points.size(); ++i) {
        float x = points[i].x, y = points[i].y;
        float d = std::fabs(l->a * x + l->b * y + l->c); // perpendicular distance (since normalized)
        if (d < distance_threshold_px_) inliers_idx.push_back(static_cast<int>(i));
      }

      if (static_cast<int>(inliers_idx.size()) > best_count) {
        best_count = static_cast<int>(inliers_idx.size());
        best_inliers_idx.swap(inliers_idx);
      }
    }

    if (best_inliers_idx.empty() || best_count < min_inliers_) {
      if (publish_original_if_fail_) publish(color_for_draw, msg->header);
      RCLCPP_WARN(get_logger(), "RANSAC failed or too few inliers: %d", best_count);
      return;
    }

    // Collect inliers
    std::vector<cv::Point2f> inliers;
    inliers.reserve(best_inliers_idx.size());
    for (int idx : best_inliers_idx) inliers.push_back(points[idx]);

    // Refine line with OpenCV fitLine (robust to verticals)
    cv::Vec4f line;
    cv::fitLine(inliers, line, cv::DIST_L2, 0, 0.01, 0.01);
    float vx = line[0], vy = line[1], x0 = line[2], y0 = line[3];

    // Intersections with image borders to get endpoints
    std::vector<cv::Point> candidates;

    auto add_if_inside = [&](float x, float y) {
      if (x >= 0 && x < w && y >= 0 && y < h) candidates.emplace_back(cv::Point(cvRound(x), cvRound(y)));
    };

    if (std::fabs(vx) > 1e-6f) {
      float t = (0.f - x0) / vx;           add_if_inside(0.f, y0 + t * vy);
      t = (float(w - 1) - x0) / vx;        add_if_inside(float(w - 1), y0 + t * vy);
    }
    if (std::fabs(vy) > 1e-6f) {
      float t = (0.f - y0) / vy;           add_if_inside(x0 + t * vx, 0.f);
      t = (float(h - 1) - y0) / vy;        add_if_inside(x0 + t * vx, float(h - 1));
    }

    cv::Point p1, p2;
    if (candidates.size() < 2) {
      // Fallback: long segment through (x0,y0)
      p1 = cv::Point(cvRound(x0 - vx * 1000.f), cvRound(y0 - vy * 1000.f));
      p2 = cv::Point(cvRound(x0 + vx * 1000.f), cvRound(y0 + vy * 1000.f));
    } else {
      // Choose farthest pair
      int maxd = -1;
      for (size_t i = 0; i < candidates.size(); ++i) {
        for (size_t j = i + 1; j < candidates.size(); ++j) {
          int dx = candidates[i].x - candidates[j].x;
          int dy = candidates[i].y - candidates[j].y;
          int d2 = dx * dx + dy * dy;
          if (d2 > maxd) {
            maxd = d2;
            p1 = candidates[i];
            p2 = candidates[j];
          }
        }
      }
    }

    cv::line(color_for_draw, p1, p2, cv::Scalar(draw_b_, draw_g_, draw_r_), draw_thickness_, cv::LINE_AA);
    publish(color_for_draw, msg->header);
  }

  void publish(const cv::Mat& bgr, const std_msgs::msg::Header& header) {
    auto msg_out = cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg();
    pub_->publish(*msg_out);
  }

  // Params / state
  std::string input_topic_, output_topic_;
  int binary_threshold_, min_pixels_, iterations_, min_inliers_, draw_thickness_;
  int draw_b_, draw_g_, draw_r_;
  double distance_threshold_px_;
  bool publish_original_if_fail_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RansacLineNode>());
  rclcpp::shutdown();
  return 0;
}
