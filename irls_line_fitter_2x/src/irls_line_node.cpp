#include "irls_line_fitter_2x/irls_line_node.hpp"

#include <numeric>

// ----------------------------- Constructor ----------------------------------
IRLSLineNode::IRLSLineNode() : Node("irls_line_node")
{
  // Topics
  input_topic_        = declare_parameter<std::string>("input_topic",        "/pipeline/camera/segmentation_mask");
  input_topic_info_   = declare_parameter<std::string>("input_topic_info",   "/pipeline/camera/camera_info");
  output_topic_img_   = declare_parameter<std::string>("output_topic_img",   "/irls_line/image");
  output_topic_lines_ = declare_parameter<std::string>("output_topic_lines", "/irls_line/lines");

  // Preprocessing
  binary_threshold_ = declare_parameter<int>("binary_threshold", 200);
  min_pixels_       = declare_parameter<int>("min_pixels", 200);

  // IRLS
  max_iters_     = declare_parameter<int>("max_irls_iters", 30);
  eps_change_    = declare_parameter<double>("eps_change", 1e-4);
  loss_type_str_ = declare_parameter<std::string>("loss", "tukey");
  huber_delta_   = declare_parameter<double>("huber_delta", 1.5);
  tukey_c_       = declare_parameter<double>("tukey_c", 3.485);
  scale_mad_     = declare_parameter<bool>("scale_with_mad", true);

  // Drawing
  draw_thickness_           = declare_parameter<int>("draw_thickness", 3);
  draw_b_                   = declare_parameter<int>("draw_b", 0);
  draw_g_                   = declare_parameter<int>("draw_g", 0);
  draw_r_                   = declare_parameter<int>("draw_r", 255);
  publish_original_if_fail_ = declare_parameter<bool>("publish_original_if_fail", true);

  // Clipping
  clip_to_object_   = declare_parameter<bool>("clip_to_object", true);
  clip_max_dist_px_ = declare_parameter<double>("clip_max_dist_px", 6.0);

  // Second-pass
  find_second_line_    = declare_parameter<bool>("find_second_line", true);
  removal_band_px_     = declare_parameter<double>("removal_band_px", 8.0);
  min_pixels_second_   = declare_parameter<int>("min_pixels_second", 250);
  draw2_b_             = declare_parameter<int>("draw2_b", 0);
  draw2_g_             = declare_parameter<int>("draw2_g", 255);
  draw2_r_             = declare_parameter<int>("draw2_r", 0);
  draw_intersection_   = declare_parameter<bool>("draw_intersection", true);
  intersection_radius_ = declare_parameter<int>("intersection_radius", 5);

  loss_ = (loss_type_str_ == "tukey") ? RobustLoss::TUKEY : RobustLoss::HUBER;

  sub_ = create_subscription<sensor_msgs::msg::Image>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&IRLSLineNode::imageCb, this, std::placeholders::_1));

  pub_img_   = create_publisher<sensor_msgs::msg::Image>(output_topic_img_, 10);
  pub_lines_ = create_publisher<vortex_msgs::msg::LineSegment2DArray>(output_topic_lines_, 10);
}

// ----------------------------- IRLS fitting ---------------------------------
LineModel IRLSLineNode::fitIRLS(const std::vector<cv::Point2f>& pts)
{
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
    } else {
      const double c = tukey_c_;
      for (size_t i = 0; i < pts.size(); ++i) {
        double u = resid[i] / scale / c;
        wts[i] = (std::fabs(u) < 1.0) ? (float)std::pow(1.0 - u * u, 2.0) : 0.f;
      }
    }

    L = weightedPCALine(pts, wts);

    float dv = std::hypot(L.vx - vx_old, L.vy - vy_old);
    float dp = std::hypot(L.x0 - x0_old, L.y0 - y0_old);
    if (dv < (float)eps_change_ && dp < (float)eps_change_) break;
  }
  return L;
}

// ----------------------------- Image callback -------------------------------
void IRLSLineNode::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat gray;
  if (cv_ptr->image.channels() == 3) cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
  else gray = cv_ptr->image;

  cv::Mat mask;
  cv::threshold(gray, mask, binary_threshold_, 255, cv::THRESH_BINARY);

  std::vector<cv::Point> nz;
  cv::findNonZero(mask, nz);
  if ((int)nz.size() < min_pixels_) {
    if (publish_original_if_fail_) publishImage(cv_ptr->image, msg->header);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Too few mask pixels: %zu", nz.size());
    return;
  }

  std::vector<cv::Point2f> pts;
  pts.reserve(nz.size());
  for (const auto& p : nz) pts.emplace_back((float)p.x, (float)p.y);

  // ----- First fit -----
  LineModel L1 = fitIRLS(pts);

  cv::Mat color;
  if (cv_ptr->image.channels() == 3) color = cv_ptr->image.clone();
  else cv::cvtColor(cv_ptr->image, color, cv::COLOR_GRAY2BGR);
  const int w = color.cols, h = color.rows;

  cv::Point p1a(0, 0), p1b(0, 0);
  bool first_ok = false;

  if (clippedSegmentFromPts(L1, pts, w, h, clip_to_object_, clip_max_dist_px_, p1a, p1b)) {
    const cv::Vec4i seg1(p1a.x, p1a.y, p1b.x, p1b.y);
    first_ok = hasEnoughWhiteUnderLine(mask, seg1, 10, 0.60);
    if (first_ok) {
      cv::line(color, p1a, p1b, cv::Scalar(draw_b_, draw_g_, draw_r_),
               draw_thickness_, cv::LINE_AA);
    }
  }

  // ----- Second fit -----
  LineModel L2;
  cv::Point p2a(0, 0), p2b(0, 0);
  bool second_ok = false;

  if (find_second_line_) {
    std::vector<cv::Point2f> pts2;
    pts2.reserve(pts.size());
    for (const auto& q : pts) {
      if (perpendicularDistance(L1, q) > (float)removal_band_px_) pts2.push_back(q);
    }

    if ((int)pts2.size() >= min_pixels_second_) {
      L2 = fitIRLS(pts2);

      if (clippedSegmentFromPts(L2, pts2, w, h, clip_to_object_, clip_max_dist_px_, p2a, p2b)) {
        const cv::Vec4i seg2(p2a.x, p2a.y, p2b.x, p2b.y);
        second_ok = hasEnoughWhiteUnderLine(mask, seg2, 10, 0.60);
        if (second_ok) {
          cv::line(color, p2a, p2b, cv::Scalar(draw2_b_, draw2_g_, draw2_r_),
                   draw_thickness_, cv::LINE_AA);
        }
      }

      if (second_ok && draw_intersection_) {
        cv::Point2f ip;
        if (intersectLines(L1, L2, ip)) {
          int ix = std::clamp((int)std::lround(ip.x), 0, w - 1);
          int iy = std::clamp((int)std::lround(ip.y), 0, h - 1);
          cv::circle(color, cv::Point(ix, iy), intersection_radius_,
                     cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_AA);
        }
      }
    }
  }

  publishImage(color, msg->header);
  publishLines(msg->header, p1a, p1b, first_ok, second_ok, p2a, p2b);
}

// ----------------------------- Publishers -----------------------------------
void IRLSLineNode::publishImage(const cv::Mat& bgr, const std_msgs::msg::Header& header)
{
  auto out = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, bgr).toImageMsg();
  pub_img_->publish(*out);
}

vortex_msgs::msg::LineSegment2D IRLSLineNode::makeSeg(const cv::Point& a, const cv::Point& b)
{
  vortex_msgs::msg::LineSegment2D seg;
  seg.p0.x = a.x; seg.p0.y = a.y;
  seg.p1.x = b.x; seg.p1.y = b.y;
  return seg;
}

void IRLSLineNode::publishLines(const std_msgs::msg::Header& header,
                                const cv::Point& p1a, const cv::Point& p1b,
                                bool have_first,
                                bool have_second,
                                const cv::Point& p2a, const cv::Point& p2b)
{
  vortex_msgs::msg::LineSegment2DArray arr;
  arr.header = header;
  if (have_first  && p1a != p1b) arr.lines.push_back(makeSeg(p1a, p1b));
  if (have_second && p2a != p2b) arr.lines.push_back(makeSeg(p2a, p2b));
  pub_lines_->publish(arr);
}

// ----------------------------- main -----------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IRLSLineNode>());
  rclcpp::shutdown();
  return 0;
}
