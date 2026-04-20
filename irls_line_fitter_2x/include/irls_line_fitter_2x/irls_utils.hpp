#pragma once

#include <opencv2/opencv.hpp>
#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>

// ----------------------------- Robust Loss ----------------------------------
enum class RobustLoss { HUBER, TUKEY };

// ----------------------------- Data types -----------------------------------
struct LineModel {
  float vx{1.f}, vy{0.f}, x0{0.f}, y0{0.f};
};

// ----------------------------- Helpers --------------------------------------
static inline int clampInt(int v, int lo, int hi)
{
  return std::max(lo, std::min(v, hi));
}

static inline float perpendicularDistance(const LineModel& L, const cv::Point2f& p)
{
  float dx = p.x - L.x0;
  float dy = p.y - L.y0;
  return std::fabs(dx * L.vy - dy * L.vx);
}

static inline void normalize(LineModel& L)
{
  float n = std::hypot(L.vx, L.vy);
  if (n > 1e-8f) { L.vx /= n; L.vy /= n; }
  else { L.vx = 1.f; L.vy = 0.f; }
}

// ----------------------------- Fitting --------------------------------------
static inline LineModel pcaFit(const std::vector<cv::Point2f>& pts)
{
  CV_Assert(!pts.empty());
  cv::Mat data((int)pts.size(), 2, CV_32F);
  for (size_t i = 0; i < pts.size(); ++i) {
    data.at<float>((int)i, 0) = pts[i].x;
    data.at<float>((int)i, 1) = pts[i].y;
  }
  cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
  cv::Vec2f mean = pca.mean;
  cv::Vec2f vec  = pca.eigenvectors.row(0);
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
  double sw = 0.0, mx = 0.0, my = 0.0;
  for (size_t i = 0; i < pts.size(); ++i) {
    sw += wts[i];
    mx += wts[i] * pts[i].x;
    my += wts[i] * pts[i].y;
  }
  if (sw <= 1e-12) return pcaFit(pts);
  mx /= sw; my /= sw;

  double cxx = 0.0, cxy = 0.0, cyy = 0.0;
  for (size_t i = 0; i < pts.size(); ++i) {
    double dx = pts[i].x - mx;
    double dy = pts[i].y - my;
    double w  = wts[i];
    cxx += w * dx * dx;
    cxy += w * dx * dy;
    cyy += w * dy * dy;
  }
  cxx /= sw; cxy /= sw; cyy /= sw;

  double tr   = cxx + cyy;
  double det  = cxx * cyy - cxy * cxy;
  double disc = std::max(0.0, tr * tr - 4.0 * det);
  double l1   = 0.5 * (tr + std::sqrt(disc));

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

// Median absolute deviation (converted to sigma under normality).
static inline double mad(const std::vector<float>& r)
{
  if (r.empty()) return 0.0;
  std::vector<float> a = r;
  for (float& v : a) v = std::fabs(v);
  std::nth_element(a.begin(), a.begin() + (int)a.size() / 2, a.end());
  return 1.4826 * a[(int)a.size() / 2];
}

// ----------------------------- Geometry -------------------------------------

// Intersect an infinite line with the image rectangle; returns up to 2 points.
static inline std::vector<cv::Point2f> lineRectIntersections(const LineModel& L, int w, int h)
{
  std::vector<cv::Point2f> out;
  auto add_if_inside = [&](float x, float y) {
    if (x >= 0 && x < w && y >= 0 && y < h) out.emplace_back(x, y);
  };
  if (std::fabs(L.vx) > 1e-6f) {
    float t = (0.f - L.x0) / L.vx;              add_if_inside(0.f,           L.y0 + t * L.vy);
    t = (float(w - 1) - L.x0) / L.vx;           add_if_inside((float)(w-1),  L.y0 + t * L.vy);
  }
  if (std::fabs(L.vy) > 1e-6f) {
    float t = (0.f - L.y0) / L.vy;              add_if_inside(L.x0 + t * L.vx, 0.f);
    t = (float(h - 1) - L.y0) / L.vy;           add_if_inside(L.x0 + t * L.vx, (float)(h-1));
  }
  return out;
}

// Compute a segment from the fitted line, optionally clipped to the inlier span.
static inline bool clippedSegmentFromPts(
    const LineModel& L,
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
      if (perpendicularDistance(L, p) <= (float)clip_max_dist_px) {
        float t = (p.x - L.x0) * L.vx + (p.y - L.y0) * L.vy;
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
      p1.x = std::clamp(p1.x, 0, w - 1); p1.y = std::clamp(p1.y, 0, h - 1);
      p2.x = std::clamp(p2.x, 0, w - 1); p2.y = std::clamp(p2.y, 0, h - 1);
      return true;
    }
  }

  // Fallback: extend to image borders.
  auto cand = lineRectIntersections(L, w, h);
  if (cand.size() < 2) {
    p1 = cv::Point(cvRound(L.x0 - L.vx * 1000.f), cvRound(L.y0 - L.vy * 1000.f));
    p2 = cv::Point(cvRound(L.x0 + L.vx * 1000.f), cvRound(L.y0 + L.vy * 1000.f));
    return true;
  }
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

// Solve intersection of two infinite lines. Returns false if parallel.
static inline bool intersectLines(const LineModel& A, const LineModel& B, cv::Point2f& out)
{
  float det = A.vx * B.vy - A.vy * B.vx;
  if (std::fabs(det) < 1e-8f) return false;
  float dx = B.x0 - A.x0;
  float dy = B.y0 - A.y0;
  float t  = (dx * B.vy - dy * B.vx) / det;
  out = cv::Point2f(A.x0 + t * A.vx, A.y0 + t * A.vy);
  return true;
}

// Check that enough white pixels exist in the band directly beneath a segment.
static inline bool hasEnoughWhiteUnderLine(
    const cv::Mat& img,
    const cv::Vec4i& line,
    int bandHeight = 10,
    double minWhiteRatio = 0.60)
{
  CV_Assert(!img.empty());
  CV_Assert(bandHeight > 0);
  CV_Assert(minWhiteRatio >= 0.0 && minWhiteRatio <= 1.0);

  cv::Mat gray, bin;
  if (img.channels() == 3) cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  else gray = img;
  cv::threshold(gray, bin, 127, 255, cv::THRESH_BINARY);

  cv::Point2f p1((float)line[0], (float)line[1]);
  cv::Point2f p2((float)line[2], (float)line[3]);
  cv::Point2f d = p2 - p1;
  float len = std::sqrt(d.x * d.x + d.y * d.y);
  if (len < 1e-6f) return false;

  cv::Point2f n(-d.y / len, d.x / len);
  if (n.y < 0) n = -n;

  cv::Point2f p1b = p1 + n * (float)bandHeight;
  cv::Point2f p2b = p2 + n * (float)bandHeight;

  auto toClampedPoint = [&](const cv::Point2f& pf) {
    int x = clampInt((int)std::lround(pf.x), 0, bin.cols - 1);
    int y = clampInt((int)std::lround(pf.y), 0, bin.rows - 1);
    return cv::Point(x, y);
  };

  std::vector<cv::Point> poly = {
    toClampedPoint(p1), toClampedPoint(p2),
    toClampedPoint(p2b), toClampedPoint(p1b)
  };

  cv::Mat mask(bin.size(), CV_8UC1, cv::Scalar(0));
  cv::fillConvexPoly(mask, poly, cv::Scalar(255));

  int total = cv::countNonZero(mask);
  if (total == 0) return false;

  cv::Mat masked;
  cv::bitwise_and(bin, mask, masked);
  int white = cv::countNonZero(masked);

  return (double)white / (double)total >= minWhiteRatio;
}
