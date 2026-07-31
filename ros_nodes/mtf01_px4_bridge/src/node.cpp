#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <px4_msgs/msg/sensor_optical_flow.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "mtf01_px4_bridge/flow.hpp"

namespace mtf01_px4_bridge {
namespace {

constexpr int64_t kMaxIntegrationTimeUs = 1'000'000;
constexpr double kPi = 3.14159265358979323846;

float median(std::vector<float> samples) {
  std::sort(samples.begin(), samples.end());
  const size_t middle = samples.size() / 2;
  if (samples.size() % 2 != 0) {
    return samples[middle];
  }
  return 0.5F * (samples[middle - 1] + samples[middle]);
}

}  // namespace

class Mtf01Px4Bridge final : public rclcpp::Node {
 public:
  Mtf01Px4Bridge()
      : Node("mtf01_px4_bridge") {
    declare_parameter<std::string>("image_topic", "/obrik/mtf01/flow/image_raw");
    declare_parameter<std::string>("range_topic", "/obrik/mtf01/rangefinder/scan");
    declare_parameter<std::string>("output_topic", "/fmu/in/sensor_optical_flow");
    declare_parameter<int>("device_id", 2);
    declare_parameter<double>("horizontal_fov", 0.733038285838);
    declare_parameter<double>("min_ground_distance", 0.08);
    declare_parameter<double>("max_ground_distance", 8.0);
    declare_parameter<double>("max_flow_rate", 7.0);

    const int64_t device_id = get_parameter("device_id").as_int();
    if (device_id < 0) {
      throw std::invalid_argument("device_id must not be negative");
    }
    device_id_ = static_cast<uint32_t>(device_id);
    horizontal_fov_ = get_parameter("horizontal_fov").as_double();
    min_ground_distance_ = get_parameter("min_ground_distance").as_double();
    max_ground_distance_ = get_parameter("max_ground_distance").as_double();
    max_flow_rate_ = get_parameter("max_flow_rate").as_double();
    if (!(horizontal_fov_ > 0.0 && horizontal_fov_ < kPi)) {
      throw std::invalid_argument("horizontal_fov must be between 0 and pi");
    }
    if (!(min_ground_distance_ > 0.0 && min_ground_distance_ < max_ground_distance_)) {
      throw std::invalid_argument("ground-distance limits must satisfy 0 < min < max");
    }

    const auto image_topic = get_parameter("image_topic").as_string();
    const auto range_topic = get_parameter("range_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    publisher_ = create_publisher<px4_msgs::msg::SensorOpticalFlow>(
        output_topic, rclcpp::SensorDataQoS());
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&Mtf01Px4Bridge::on_image, this, std::placeholders::_1));
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        range_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&Mtf01Px4Bridge::on_scan, this, std::placeholders::_1));
    RCLCPP_INFO(
        get_logger(), "MTF-01 bridge: %s + %s -> %s",
        image_topic.c_str(), range_topic.c_str(), output_topic.c_str());
  }

 private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    const float lower = std::max(static_cast<float>(min_ground_distance_), scan->range_min);
    const float upper = std::min(static_cast<float>(max_ground_distance_), scan->range_max);
    std::vector<float> samples;
    samples.reserve(scan->ranges.size());
    for (const float sample : scan->ranges) {
      if (std::isfinite(sample) && sample >= lower && sample <= upper) {
        samples.push_back(sample);
      }
    }
    distance_m_ = samples.empty()
                      ? std::numeric_limits<float>::quiet_NaN()
                      : median(std::move(samples));
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr image) {
    cv_bridge::CvImageConstPtr cv_image;
    cv::Mat current;
    try {
      cv_image = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::MONO8);
      current = cv_image->image.clone();
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "MTF-01 image conversion failed: %s", error.what());
      return;
    }
    if (current.empty()) {
      return;
    }

    uint64_t stamp_us =
        static_cast<uint64_t>(image->header.stamp.sec) * 1'000'000ULL +
        static_cast<uint64_t>(image->header.stamp.nanosec) / 1'000ULL;
    if (stamp_us == 0) {
      stamp_us = now_us();
    }
    if (previous_image_.empty() || previous_stamp_us_ == 0) {
      previous_image_ = std::move(current);
      previous_stamp_us_ = stamp_us;
      return;
    }

    const int64_t integration_us =
        static_cast<int64_t>(stamp_us) - static_cast<int64_t>(previous_stamp_us_);
    cv::Mat previous = std::move(previous_image_);
    previous_image_ = std::move(current);
    previous_stamp_us_ = stamp_us;
    if (integration_us <= 0 || integration_us > kMaxIntegrationTimeUs) {
      return;
    }

    const FlowEstimate estimate = estimate_flow(previous, previous_image_, horizontal_fov_);
    px4_msgs::msg::SensorOpticalFlow message{};
    const uint64_t time_us = now_us();
    message.timestamp = time_us;
    message.timestamp_sample = time_us;
    message.device_id = device_id_;
    message.pixel_flow[0] = estimate.integrated_x;
    message.pixel_flow[1] = estimate.integrated_y;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    message.delta_angle[0] = nan;
    message.delta_angle[1] = nan;
    message.delta_angle[2] = nan;
    message.delta_angle_available = false;
    message.distance_m = distance_m_;
    message.distance_available = std::isfinite(distance_m_);
    message.integration_timespan_us = static_cast<uint32_t>(integration_us);
    message.quality = static_cast<uint8_t>(estimate.quality);
    message.error_count = 0;
    message.max_flow_rate = static_cast<float>(max_flow_rate_);
    message.min_ground_distance = static_cast<float>(min_ground_distance_);
    message.max_ground_distance = static_cast<float>(max_ground_distance_);
    message.mode = px4_msgs::msg::SensorOpticalFlow::MODE_BRIGHT;
    publisher_->publish(message);
  }

  uint64_t now_us() {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1'000);
  }

  uint32_t device_id_{};
  double horizontal_fov_{};
  double min_ground_distance_{};
  double max_ground_distance_{};
  double max_flow_rate_{};
  cv::Mat previous_image_;
  uint64_t previous_stamp_us_{};
  float distance_m_{std::numeric_limits<float>::quiet_NaN()};
  rclcpp::Publisher<px4_msgs::msg::SensorOpticalFlow>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
};

}  // namespace mtf01_px4_bridge

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtf01_px4_bridge::Mtf01Px4Bridge>());
  rclcpp::shutdown();
  return 0;
}
