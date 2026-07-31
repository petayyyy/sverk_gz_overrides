#pragma once

#include <opencv2/core.hpp>

namespace mtf01_px4_bridge {

struct FlowEstimate {
  float integrated_x;
  float integrated_y;
  int quality;
  int tracked_features;
};

FlowEstimate estimate_flow(
    const cv::Mat & previous,
    const cv::Mat & current,
    double horizontal_fov,
    int max_features = 80);

}  // namespace mtf01_px4_bridge
