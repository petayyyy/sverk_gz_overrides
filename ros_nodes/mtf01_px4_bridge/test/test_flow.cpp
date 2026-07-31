#include <cmath>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include "mtf01_px4_bridge/flow.hpp"

namespace {

cv::Mat textured_image() {
  cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC1);
  for (int y = 10; y <= 90; y += 10) {
    for (int x = 10; x <= 90; x += 10) {
      cv::circle(image, cv::Point(x, y), 2, cv::Scalar(255), cv::FILLED);
    }
  }
  return image;
}

cv::Mat shifted(const cv::Mat & image, double x, double y) {
  cv::Mat output;
  const cv::Mat transform = (cv::Mat_<double>(2, 3) << 1, 0, x, 0, 1, y);
  cv::warpAffine(image, output, transform, image.size());
  return output;
}

TEST(FlowEstimator, EstimatesPx4OpenCvFlowConvention) {
  const auto previous = textured_image();
  const auto estimate = mtf01_px4_bridge::estimate_flow(
      previous, shifted(previous, 2.0, -3.0), 42.0 * CV_PI / 180.0);
  const double focal = 50.0 / std::tan(21.0 * CV_PI / 180.0);

  EXPECT_GT(estimate.quality, 0);
  EXPECT_GE(estimate.tracked_features, 6);
  EXPECT_NEAR(estimate.integrated_x, std::atan2(2.0, focal), 0.005);
  EXPECT_NEAR(estimate.integrated_y, std::atan2(-3.0, focal), 0.005);
}

TEST(FlowEstimator, TexturelessFrameHasZeroQuality) {
  const cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC1);
  const auto estimate = mtf01_px4_bridge::estimate_flow(image, image, 42.0 * CV_PI / 180.0);

  EXPECT_EQ(estimate.quality, 0);
  EXPECT_FLOAT_EQ(estimate.integrated_x, 0.0F);
  EXPECT_FLOAT_EQ(estimate.integrated_y, 0.0F);
}

}  // namespace
