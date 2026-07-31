#include "mtf01_px4_bridge/flow.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace mtf01_px4_bridge {
namespace {

constexpr int kMinTrackedFeatures = 6;

float median(std::vector<float> values) {
  if (values.empty()) {
    throw std::invalid_argument("median requires at least one value");
  }
  std::sort(values.begin(), values.end());
  const size_t middle = values.size() / 2;
  if (values.size() % 2 != 0) {
    return values[middle];
  }
  return 0.5F * (values[middle - 1] + values[middle]);
}

FlowEstimate unavailable(int tracked_features = 0) {
  return {0.0F, 0.0F, 0, tracked_features};
}

}  // namespace

FlowEstimate estimate_flow(
    const cv::Mat & previous,
    const cv::Mat & current,
    double horizontal_fov,
    int max_features) {
  if (previous.type() != CV_8UC1 || current.type() != CV_8UC1) {
    throw std::invalid_argument("flow images must be mono8");
  }
  if (previous.size() != current.size()) {
    throw std::invalid_argument("flow images must have identical dimensions");
  }
  if (!(horizontal_fov > 0.0 && horizontal_fov < CV_PI)) {
    throw std::invalid_argument("horizontal_fov must be between 0 and pi");
  }
  if (max_features < 8) {
    throw std::invalid_argument("max_features must be at least 8");
  }

  std::vector<cv::Point2f> features;
  cv::goodFeaturesToTrack(
      previous, features, max_features, 0.01, 4.0, cv::noArray(), 5, false, 0.04);
  if (features.size() < kMinTrackedFeatures) {
    return unavailable();
  }

  std::vector<cv::Point2f> tracked;
  std::vector<unsigned char> status;
  std::vector<float> errors;
  cv::calcOpticalFlowPyrLK(
      previous,
      current,
      features,
      tracked,
      status,
      errors,
      cv::Size(15, 15),
      2,
      cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 20, 0.03));
  if (tracked.empty() || status.empty()) {
    return unavailable();
  }

  std::vector<cv::Point2f> displacements;
  displacements.reserve(features.size());
  for (size_t index = 0; index < features.size(); ++index) {
    if (status[index] != 0 && (errors.empty() || std::isfinite(errors[index]))) {
      displacements.push_back(tracked[index] - features[index]);
    }
  }
  if (displacements.size() < kMinTrackedFeatures) {
    return unavailable(static_cast<int>(displacements.size()));
  }

  std::vector<float> x_values;
  std::vector<float> y_values;
  x_values.reserve(displacements.size());
  y_values.reserve(displacements.size());
  for (const auto & displacement : displacements) {
    x_values.push_back(displacement.x);
    y_values.push_back(displacement.y);
  }
  const cv::Point2f center(median(x_values), median(y_values));

  std::vector<float> residuals;
  residuals.reserve(displacements.size());
  for (const auto & displacement : displacements) {
    residuals.push_back(cv::norm(displacement - center));
  }
  const float inlier_threshold = std::max(0.75F, 3.0F * median(residuals));

  cv::Point2f total(0.0F, 0.0F);
  int inlier_count = 0;
  for (size_t index = 0; index < displacements.size(); ++index) {
    if (residuals[index] <= inlier_threshold) {
      total += displacements[index];
      ++inlier_count;
    }
  }
  if (inlier_count < kMinTrackedFeatures) {
    return unavailable(inlier_count);
  }

  const cv::Point2f pixel_flow = total * (1.0F / static_cast<float>(inlier_count));
  const double focal_length =
      (static_cast<double>(previous.cols) / 2.0) / std::tan(horizontal_fov / 2.0);
  const int quality = std::clamp(
      static_cast<int>(std::lround(
          255.0 * std::min(1.0, static_cast<double>(inlier_count) / max_features))),
      1,
      255);
  return {
      static_cast<float>(std::atan2(pixel_flow.x, focal_length)),
      static_cast<float>(std::atan2(pixel_flow.y, focal_length)),
      quality,
      inlier_count,
  };
}

}  // namespace mtf01_px4_bridge
