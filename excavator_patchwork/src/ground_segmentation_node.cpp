#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "patchwork/patchworkpp.h"

namespace excavator_patchwork
{

class GroundSegmentationNode : public rclcpp::Node
{
public:
  GroundSegmentationNode()
  : Node("excavator_patchwork")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/localmap/body_filtered_points");
    ground_topic_ = declare_parameter<std::string>(
      "ground_topic", "/terrain/ground_points");
    nonground_topic_ = declare_parameter<std::string>(
      "nonground_topic", "/terrain/nonground_points");

    sensor_origin_x_ = declare_parameter<double>("sensor_origin_x", 0.3);
    sensor_origin_y_ = declare_parameter<double>("sensor_origin_y", -1.3);
    sensor_origin_z_ = declare_parameter<double>("sensor_origin_z", 0.624);

    patchwork::Params params;
    params.sensor_height = declare_parameter<double>("sensor_height", 0.624);
    params.num_iter = declare_parameter<int>("num_iter", params.num_iter);
    params.num_lpr = declare_parameter<int>("num_lpr", params.num_lpr);
    params.num_min_pts = declare_parameter<int>("num_min_pts", params.num_min_pts);
    params.th_seeds = declare_parameter<double>("th_seeds", params.th_seeds);
    params.th_dist = declare_parameter<double>("th_dist", params.th_dist);
    params.th_seeds_v = declare_parameter<double>("th_seeds_v", params.th_seeds_v);
    params.th_dist_v = declare_parameter<double>("th_dist_v", params.th_dist_v);
    params.max_range = declare_parameter<double>("max_range", params.max_range);
    params.min_range = declare_parameter<double>("min_range", params.min_range);
    params.uprightness_thr = declare_parameter<double>(
      "uprightness_thr", params.uprightness_thr);
    params.adaptive_seed_selection_margin = declare_parameter<double>(
      "adaptive_seed_selection_margin", params.adaptive_seed_selection_margin);
    params.enable_RVPF = declare_parameter<bool>("enable_rvpf", params.enable_RVPF);
    params.enable_TGR = declare_parameter<bool>("enable_tgr", params.enable_TGR);
    params.verbose = declare_parameter<bool>("verbose", params.verbose);
    params.enable_RNR = false;

    if (params.sensor_height <= 0.0) {
      throw std::invalid_argument("sensor_height must be greater than zero");
    }
    if (params.min_range < 0.0 || params.max_range <= params.min_range) {
      throw std::invalid_argument("Require 0 <= min_range < max_range");
    }
    if (params.num_min_pts < 1) {
      throw std::invalid_argument("num_min_pts must be at least 1");
    }

    patchwork_ = std::make_unique<patchwork::PatchWorkpp>(params);

    ground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      ground_topic_, rclcpp::QoS(5).reliable());
    nonground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      nonground_topic_, rclcpp::QoS(5).reliable());
    input_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GroundSegmentationNode::pointCloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Patchwork++: %s -> ground=%s nonground=%s, sensor_origin=(%.3f, %.3f, %.3f)",
      input_topic_.c_str(), ground_topic_.c_str(), nonground_topic_.c_str(),
      sensor_origin_x_, sensor_origin_y_, sensor_origin_z_);
  }

private:
  static sensor_msgs::msg::PointCloud2 copyPoints(
    const sensor_msgs::msg::PointCloud2 & input,
    const Eigen::VectorXi & algorithm_indices,
    const std::vector<std::size_t> & source_indices)
  {
    sensor_msgs::msg::PointCloud2 output = input;
    output.height = 1U;
    output.width = 0U;
    output.row_step = 0U;
    output.data.resize(
      static_cast<std::size_t>(algorithm_indices.size()) * input.point_step);

    std::size_t output_offset = 0U;
    for (Eigen::Index index = 0; index < algorithm_indices.size(); ++index) {
      const int algorithm_index = algorithm_indices(index);
      if (algorithm_index < 0 ||
        static_cast<std::size_t>(algorithm_index) >= source_indices.size())
      {
        continue;
      }
      const std::size_t source_index = source_indices[algorithm_index];
      const std::size_t row = source_index / input.width;
      const std::size_t column = source_index % input.width;
      const std::size_t input_offset = row * input.row_step + column * input.point_step;
      if (input_offset + input.point_step > input.data.size()) {
        continue;
      }
      std::memcpy(
        output.data.data() + output_offset,
        input.data.data() + input_offset,
        input.point_step);
      output_offset += input.point_step;
    }

    output.data.resize(output_offset);
    output.width = static_cast<std::uint32_t>(output_offset / input.point_step);
    output.row_step = output.width * output.point_step;
    return output;
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    if (message->width == 0U || message->height == 0U) {
      return;
    }

    const auto start = std::chrono::steady_clock::now();
    Eigen::MatrixXf points;
    std::vector<std::size_t> source_indices;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      const std::size_t point_count =
        static_cast<std::size_t>(message->width) * message->height;
      points.resize(static_cast<Eigen::Index>(point_count), 3);
      source_indices.reserve(point_count);

      Eigen::Index valid_count = 0;
      for (std::size_t index = 0; index < point_count; ++index, ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        points(valid_count, 0) = static_cast<float>(*x - sensor_origin_x_);
        points(valid_count, 1) = static_cast<float>(*y - sensor_origin_y_);
        points(valid_count, 2) = static_cast<float>(*z - sensor_origin_z_);
        source_indices.push_back(index);
        ++valid_count;
      }
      points.conservativeResize(valid_count, 3);
    } catch (const std::runtime_error & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input PointCloud2 requires FLOAT32 x/y/z fields: %s", exception.what());
      return;
    }

    if (points.rows() == 0) {
      return;
    }

    patchwork_->estimateGround(points);
    const Eigen::VectorXi ground_indices = patchwork_->getGroundIndices();
    const Eigen::VectorXi nonground_indices = patchwork_->getNongroundIndices();

    auto ground = copyPoints(*message, ground_indices, source_indices);
    auto nonground = copyPoints(*message, nonground_indices, source_indices);
    ground_publisher_->publish(ground);
    nonground_publisher_->publish(nonground);

    const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Ground segmentation: input=%zu ground=%u nonground=%u time=%.2f ms frame=%s",
      source_indices.size(), ground.width, nonground.width, elapsed,
      message->header.frame_id.c_str());
  }

  std::string input_topic_;
  std::string ground_topic_;
  std::string nonground_topic_;
  double sensor_origin_x_{0.3};
  double sensor_origin_y_{-1.3};
  double sensor_origin_z_{0.624};

  std::unique_ptr<patchwork::PatchWorkpp> patchwork_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_publisher_;
};

}  // namespace excavator_patchwork

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<excavator_patchwork::GroundSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}
