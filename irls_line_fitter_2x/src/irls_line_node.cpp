#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <numeric>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>
#include <std_msgs/msg/bool.hpp>

// ----------------------------- Robust Loss ----------------------------------
enum class RobustLoss { HUBER, TUKEY };

// ----------------------------- Utilities ------------------------------------
struct LineModel {
  // Unit direction (vx, vy) and a point (x0, y0) on the line
  float vx{1.f}, vy{0.f}, x0{0.f}, y0{0.f};
};

static inline float perpendicularDistance(const LineModel& L, const cv::Point2f& p) {
  // |(p - p0) x v| for unit v, in 2D this is |(dx * vy - dy * vx)|
  float dx = p.x - L.x0;
  float dy = p.y - L.y0;
  return std::fabs(dx * L.vy - dy * L.vx);
}

static inline void normalize(LineModel& L) {
  float n = std::hypot(L.vx, L.vy);
  if (n > 1e-8f) { L.vx /= n; L.vy /= n; }
  else { L.vx = 1.f; L.vy = 0.f; }
}

static inline LineModel pcaFit(const std::vector<cv::Point2f>& pts) {
  CV_Assert(!pts.empty());
  cv::Mat data((int)pts.size(), 2, CV_32F);
  for (size_t i = 0; i < pts.size(); ++i) {
    data.at<float>((int)i, 0) = pts[i].x;
    data.at<float>((int)i, 1) = pts[i].y;
  }
  cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
  cv::Vec2f mean = pca.mean;
  cv::Vec2f vec = pca.eigenvectors.row(0);
  LineModel L;
  L.x0 = mean[0]; L.y0 = mean[1];
  L.vx = vec[0];  L.vy = vec[1];
  normalize(L);
  return L;
}

static inline LineModel weightedPCALine(
    const std::vector<cv::Point2f>& pts,
    const std::vector<float>& wts) 
{
  // Weighted centroid
  double sw = 0.0, mx = 0.0, my = 0.0;
  for (size_t i = 0; i < pts.size(); ++i) { sw += wts[i]; mx += wts[i] * pts[i].x; my += wts[i] * pts[i].y; }
  if (sw <= 1e-12) {
    // Fallback to unweighted
    return pcaFit(pts);
  }
  mx /= sw; my /= sw;

  // Weighted covariance
  double cxx = 0.0, cxy = 0.0, cyy = 0.0;
  for (size_t i = 0; i < pts.size(); ++i) {
    double dx = pts[i].x - mx;
    double dy = pts[i].y - my;
    double w = wts[i];
    cxx += w * dx * dx;
    cxy += w * dx * dy;
    cyy += w * dy * dy;
  }
  cxx /= sw; cxy /= sw; cyy /= sw;

  // Principal eigenvector of 2x2 covariance
  double tr = cxx + cyy;
  double det = cxx * cyy - cxy * cxy;
  double disc = std::max(0.0, tr * tr - 4.0 * det);
  double l1 = 0.5 * (tr + std::sqrt(disc)); // larger eigenvalue

  cv::Point2f v;
  if (std::fabs(cxy) > 1e-12) {
    v = cv::Point2f((float)(l1 - cyy), (float)cxy);
  } else if (cxx >= cyy) {
    v = cv::Point2f(1.f, 0.f);
  } else {
    v = cv::Point2f(0.f, 1.f);
  }
  float n = std::hypot(v.x, v.y);
  if (n > 1e-12f) { v.x /= n; v.y /= n; } else { v.x = 1.f; v.y = 0.f; }

  LineModel L;
  L.x0 = (float)mx; L.y0 = (float)my;
  L.vx = v.x; L.vy = v.y;
  return L;
}

static inline double mad(const std::vector<float>& r) {
  if (r.empty()) return 0.0;
  std::vector<float> a = r;
  for (float& v : a) v = std::fabs(v);
  std::nth_element(a.begin(), a.begin() + (int)a.size()/2, a.end());
  double med = a[(int)a.size()/2];
  // Convert MAD to sigma under normality
  return 1.4826 * med;
}

// Intersect an infinite line with an image rectangle; return up to 2 points.
static inline std::vector<cv::Point2f> lineRectIntersections(const LineModel& L, int w, int h) {
  std::vector<cv::Point2f> out;
  auto add_if_inside = [&](float x, float y){
    if (x >= 0 && x < w && y >= 0 && y < h) out.emplace_back(x, y);
  };
  if (std::fabs(L.vx) > 1e-6f) {
    float t = (0.f - L.x0) / L.vx;           add_if_inside(0.f, L.y0 + t * L.vy);
    t = (float(w - 1) - L.x0) / L.vx;        add_if_inside((float)(w - 1), L.y0 + t * L.vy);
  }
  if (std::fabs(L.vy) > 1e-6f) {
    float t = (0.f - L.y0) / L.vy;           add_if_inside(L.x0 + t * L.vx, 0.f);
    t = (float(h - 1) - L.y0) / L.vy;        add_if_inside(L.x0 + t * L.vx, (float)(h - 1));
  }
  return out;
}

static inline bool clippedSegmentFromPts(const LineModel& L,
                                         const std::vector<cv::Point2f>& pts,
                                         int w, int h,
                                         bool clip_to_object,
                                         double clip_max_dist_px,
                                         cv::Point& p1, cv::Point& p2)
{
  if (clip_to_object) {
    float min_t = std::numeric_limits<float>::max();
    float max_t = -std::numeric_limits<float>::max();
    int count_inliers = 0;

    for (const auto& p : pts) {
      float d = perpendicularDistance(L, p);
      if (d <= (float)clip_max_dist_px) {
        float t = (p.x - L.x0) * L.vx + (p.y - L.y0) * L.vy; // projection on unit dir
        if (t < min_t) min_t = t;
        if (t > max_t) max_t = t;
        ++count_inliers;
      }
    }

    if (count_inliers >= 2 && (max_t - min_t) > 1.f) {
      cv::Point2f a(L.x0 + min_t * L.vx, L.y0 + min_t * L.vy);
      cv::Point2f b(L.x0 + max_t * L.vx, L.y0 + max_t * L.vy);
      p1 = cv::Point(cvRound(a.x), cvRound(a.y));
      p2 = cv::Point(cvRound(b.x), cvRound(b.y));
      p1.x = std::clamp(p1.x, 0, w - 1);
      p1.y = std::clamp(p1.y, 0, h - 1);
      p2.x = std::clamp(p2.x, 0, w - 1);
      p2.y = std::clamp(p2.y, 0, h - 1);
      return true;
    }
  }

  // Fallback: extend line to image borders
  auto cand = lineRectIntersections(L, w, h);
  if (cand.size() < 2) {
    p1 = cv::Point(cvRound(L.x0 - L.vx * 1000.f), cvRound(L.y0 - L.vy * 1000.f));
    p2 = cv::Point(cvRound(L.x0 + L.vx * 1000.f), cvRound(L.y0 + L.vy * 1000.f));
    return true;
  } else {
    int best_i = 0, best_j = 1;
    double best_d2 = 0.0;
    for (int i = 0; i < (int)cand.size(); ++i) {
      for (int j = i + 1; j < (int)cand.size(); ++j) {
        double dx = cand[i].x - cand[j].x;
        double dy = cand[i].y - cand[j].y;
        double d2 = dx * dx + dy * dy;
        if (d2 > best_d2) { best_d2 = d2; best_i = i; best_j = j; }
      }
    }
    p1 = cv::Point(cvRound(cand[best_i].x), cvRound(cand[best_i].y));
    p2 = cv::Point(cvRound(cand[best_j].x), cvRound(cand[best_j].y));
    return true;
  }
}

// Solve intersection point of two infinite lines; returns false if parallel.
static inline bool intersectLines(const LineModel& A, const LineModel& B, cv::Point2f& out) {
  // A: p = a0 + t * av,  B: q = b0 + s * bv
  float det = A.vx * B.vy - A.vy * B.vx;
  if (std::fabs(det) < 1e-8f) return false; // parallel (or almost)
  // Solve for t in [A] using cross/determinant
  float dx = B.x0 - A.x0;
  float dy = B.y0 - A.y0;
  float t = (dx * B.vy - dy * B.vx) / det;
  out = cv::Point2f(A.x0 + t * A.vx, A.y0 + t * A.vy);
  return true;
}

// ----------------------------- Node -----------------------------------------
class IRLSLineNode : public rclcpp::Node {
public:
  IRLSLineNode() : Node("irls_line_node") {
    // Topics
    input_topic_  = declare_parameter<std::string>("input_topic", "/filtered_image");
    output_topic_ = declare_parameter<std::string>("output_topic", "/irls_line/image");

    // Preprocessing
    binary_threshold_ = declare_parameter<int>("binary_threshold", 200);
    min_pixels_       = declare_parameter<int>("min_pixels", 200);

    // IRLS params
    max_iters_     = declare_parameter<int>("max_irls_iters", 30);
    eps_change_    = declare_parameter<double>("eps_change", 1e-4);
    loss_type_str_ = declare_parameter<std::string>("loss", "tukey"); // "huber" or "tukey"
    huber_delta_   = declare_parameter<double>("huber_delta", 1.5);
    tukey_c_       = declare_parameter<double>("tukey_c", 3.485);
    scale_mad_     = declare_parameter<bool>("scale_with_mad", true);

    // Drawing
    draw_thickness_   = declare_parameter<int>("draw_thickness", 3);
    draw_b_           = declare_parameter<int>("draw_b", 0);
    draw_g_           = declare_parameter<int>("draw_g", 0);
    draw_r_           = declare_parameter<int>("draw_r", 255);
    publish_original_if_fail_ = declare_parameter<bool>("publish_original_if_fail", true);

    // Clipping
    clip_to_object_   = declare_parameter<bool>("clip_to_object", true);
    clip_max_dist_px_ = declare_parameter<double>("clip_max_dist_px", 6.0);

    // Second-pass (intersection) search
    find_second_line_   = declare_parameter<bool>("find_second_line", true);
    removal_band_px_    = declare_parameter<double>("removal_band_px", 8.0);
    min_pixels_second_  = declare_parameter<int>("min_pixels_second", 150);
    draw2_b_            = declare_parameter<int>("draw2_b", 0);
    draw2_g_            = declare_parameter<int>("draw2_g", 255);
    draw2_r_            = declare_parameter<int>("draw2_r", 0);
    draw_intersection_  = declare_parameter<bool>("draw_intersection", true);
    intersection_radius_= declare_parameter<int>("intersection_radius", 5);

    // Loss enum
    if (loss_type_str_ == "tukey") loss_ = RobustLoss::TUKEY; else loss_ = RobustLoss::HUBER;

    auto qos_no_buffer =
      rclcpp::QoS(rclcpp::KeepLast(1))
        .best_effort()          // typical for image streams
        .durability_volatile(); // don't persist samples

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&IRLSLineNode::imageCb, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
    second_line_pub_ = create_publisher<std_msgs::msg::Bool>("/irls_line/second_drawn", 10);
    

  }

private:
  // Fit a line by IRLS given points
  LineModel fitIRLS(const std::vector<cv::Point2f>& pts) {
    LineModel L = pcaFit(pts);
    std::vector<float> resid(pts.size(), 0.f);
    std::vector<float> wts(pts.size(), 1.f);
    for (int iter = 0; iter < max_iters_; ++iter) {
      float vx_old = L.vx, vy_old = L.vy, x0_old = L.x0, y0_old = L.y0;
      for (size_t i = 0; i < pts.size(); ++i) resid[i] = perpendicularDistance(L, pts[i]);
      double scale = 1.0;
      if (scale_mad_) {
        scale = mad(resid);
        if (scale < 1e-6) scale = 1.0;
      }
      if (loss_ == RobustLoss::HUBER) {
        const double d = huber_delta_;
        for (size_t i = 0; i < pts.size(); ++i) {
          double r = resid[i] / scale;
          wts[i] = (float)((std::fabs(r) <= d) ? 1.0 : (d / std::fabs(r)));
        }
      } else { // Tukey
        const double c = tukey_c_;
        for (size_t i = 0; i < pts.size(); ++i) {
          double u = resid[i] / scale / c;
          if (std::fabs(u) < 1.0) {
            double w = std::pow(1.0 - u * u, 2.0);
            wts[i] = (float)w;
          } else {
            wts[i] = 0.f;
          }
        }
      }
      L = weightedPCALine(pts, wts);
      float dv = std::hypot(L.vx - vx_old, L.vy - vy_old);
      float dp = std::hypot(L.x0 - x0_old, L.y0 - y0_old);
      if (dv < (float)eps_change_ && dp < (float)eps_change_) break;
    }
    return L;
  }

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    bool second_line_drawn = false;
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray;
    if (cv_ptr->image.channels() == 3) {
      cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = cv_ptr->image.clone();
    }

    cv::Mat mask;
    cv::threshold(gray, mask, binary_threshold_, 255, cv::THRESH_BINARY);

    std::vector<cv::Point> nz;
    cv::findNonZero(mask, nz);
    if ((int)nz.size() < min_pixels_) {
      if (publish_original_if_fail_) publish(cv_ptr->image, msg->header);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Too few mask pixels: %zu", nz.size());
      return;
    }

    // Convert to float point list
    std::vector<cv::Point2f> pts; pts.reserve(nz.size());
    for (const auto& p : nz) pts.emplace_back((float)p.x, (float)p.y);

    // ----- First fit -----
    LineModel L1 = fitIRLS(pts);

    // Prepare color image for drawing
    cv::Mat color;
    if (cv_ptr->image.channels() == 3) color = cv_ptr->image.clone();
    else cv::cvtColor(cv_ptr->image, color, cv::COLOR_GRAY2BGR);
    const int w = color.cols, h = color.rows;

    // Segment for first line
    cv::Point p1a, p1b;
    clippedSegmentFromPts(L1, pts, w, h, clip_to_object_, clip_max_dist_px_, p1a, p1b);
    cv::line(color, p1a, p1b, cv::Scalar(draw_b_, draw_g_, draw_r_), draw_thickness_, cv::LINE_AA);

    // ----- Second-pass search for a crossing line -----
  LineModel L2;
  cv::Point p2a, p2b;
  if (find_second_line_) {
    std::vector<cv::Point2f> pts2; pts2.reserve(pts.size());
    for (const auto& q : pts) {
      if (perpendicularDistance(L1, q) > (float)removal_band_px_) pts2.push_back(q);
    }

    if ((int)pts2.size() >= min_pixels_second_) {
      L2 = fitIRLS(pts2);

      // Draw second line segment using its own point set for clipping
      if (clippedSegmentFromPts(L2, pts2, w, h, clip_to_object_, clip_max_dist_px_, p2a, p2b)) {
        cv::line(color, p2a, p2b, cv::Scalar(draw2_b_, draw2_g_, draw2_r_), draw_thickness_, cv::LINE_AA);
        second_line_drawn = true;  // NEW: mark drawn
      }

      // Optional: draw intersection
      if (draw_intersection_) {
        cv::Point2f ip;
        if (intersectLines(L1, L2, ip)) {
          int ix = std::clamp((int)std::lround(ip.x), 0, w - 1);
          int iy = std::clamp((int)std::lround(ip.y), 0, h - 1);
          cv::circle(color, cv::Point(ix, iy), intersection_radius_, cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_AA);
        }
      }
    }
  }

  // NEW: report whether the second line was drawn
  if (find_second_line_) {
    if (second_line_drawn) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Second line drawn.");
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Second line NOT drawn (insufficient points or clipping failed).");
    }
  }
   if (second_line_pub_) {
    std_msgs::msg::Bool m;
    m.data = second_line_drawn;
    second_line_pub_->publish(m);
    }

    publish(color, msg->header);
  }

  void publish(const cv::Mat& bgr, const std_msgs::msg::Header& header) {
    auto out = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, bgr).toImageMsg();
    pub_->publish(*out);
  }

  // ----------------------------- Params & ROS --------------------------------
  // Topics
  std::string input_topic_{"/filtered_image"}, output_topic_{"/irls_line/image"};

  // Preprocessing
  int binary_threshold_{200}, min_pixels_{200};

  // IRLS
  int max_iters_{30};
  double eps_change_{1e-4};
  std::string loss_type_str_{"tukey"};
  double huber_delta_{1.5}, tukey_c_{3.485};
  bool scale_mad_{true};
  RobustLoss loss_{RobustLoss::TUKEY};

  // Drawing (first line)
  int draw_thickness_{3}, draw_b_{0}, draw_g_{0}, draw_r_{255};
  bool publish_original_if_fail_{true};

  // Clipping
  bool clip_to_object_{true};
  double clip_max_dist_px_{6.0};

  // Second-pass controls
  bool find_second_line_{true};
  double removal_band_px_{8.0};
  int min_pixels_second_{150};
  int draw2_b_{0}, draw2_g_{255}, draw2_r_{0};
  bool draw_intersection_{true};
  int intersection_radius_{5};

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr second_line_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IRLSLineNode>());
  rclcpp::shutdown();
  return 0;
}