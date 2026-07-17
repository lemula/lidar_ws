#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace excavator_height_map
{

namespace
{

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr double kRadiansToDegrees = 57.295779513082320876;

struct CellStatistics
{
  std::uint32_t ground_count{0U};
  std::uint32_t nonground_count{0U};
  double ground_sum{0.0};
  double ground_squared_sum{0.0};
  float maximum_surface{-std::numeric_limits<float>::infinity()};
  float maximum_nonground{-std::numeric_limits<float>::infinity()};
};

}  // namespace

class HeightMapNode : public rclcpp::Node
{
public:
  HeightMapNode()
  : Node("excavator_height_map")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    ground_topic_ = declare_parameter<std::string>(
      "ground_topic", "/terrain/ground_points");
    nonground_topic_ = declare_parameter<std::string>(
      "nonground_topic", "/terrain/nonground_points");
    grid_map_topic_ = declare_parameter<std::string>(
      "grid_map_topic", "/terrain/grid_map");

    resolution_ = declare_parameter<double>("resolution", 0.10);
    length_x_ = declare_parameter<double>("length_x", 14.0);
    length_y_ = declare_parameter<double>("length_y", 16.0);
    center_x_ = declare_parameter<double>("center_x", 5.0);
    center_y_ = declare_parameter<double>("center_y", 0.0);
    minimum_ground_points_ = declare_parameter<int>(
      "minimum_ground_points_per_cell", 3);
    fill_radius_cells_ = declare_parameter<int>("fill_radius_cells", 2);
    maximum_slope_deg_ = declare_parameter<double>("maximum_slope_deg", 25.0);
    maximum_roughness_ = declare_parameter<double>("maximum_roughness", 0.08);
    maximum_step_height_ = declare_parameter<double>("maximum_step_height", 0.25);

    if (resolution_ <= 0.0 || length_x_ <= 0.0 || length_y_ <= 0.0) {
      throw std::invalid_argument("resolution and map lengths must be positive");
    }
    if (minimum_ground_points_ < 1 || fill_radius_cells_ < 0) {
      throw std::invalid_argument(
              "minimum_ground_points_per_cell must be >= 1 and fill_radius_cells >= 0");
    }

    auto output_qos = rclcpp::QoS(1).reliable().transient_local();
    grid_map_publisher_ = create_publisher<grid_map_msgs::msg::GridMap>(
      grid_map_topic_, output_qos);
    ground_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      ground_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HeightMapNode::groundCallback, this, std::placeholders::_1));
    nonground_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      nonground_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HeightMapNode::nongroundCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Height map: ground=%s nonground=%s -> %s, %.1fx%.1f m at %.3f m/cell",
      ground_topic_.c_str(), nonground_topic_.c_str(), grid_map_topic_.c_str(),
      length_x_, length_y_, resolution_);
  }

private:
  void groundCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_ground_ = message;
    tryBuildMap();
  }

  void nongroundCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_nonground_ = message;
    tryBuildMap();
  }

  void tryBuildMap()
  {
    if (!latest_ground_ || !latest_nonground_) {
      return;
    }
    const rclcpp::Time ground_stamp(latest_ground_->header.stamp);
    const rclcpp::Time nonground_stamp(latest_nonground_->header.stamp);
    if (ground_stamp != nonground_stamp) {
      return;
    }
    if (ground_stamp.nanoseconds() == last_processed_stamp_) {
      return;
    }
    last_processed_stamp_ = ground_stamp.nanoseconds();
    buildAndPublish(*latest_ground_, *latest_nonground_);
  }

  static std::size_t flatIndex(const grid_map::Index & index, const grid_map::Size & size)
  {
    return static_cast<std::size_t>(index(0)) * static_cast<std::size_t>(size(1)) +
           static_cast<std::size_t>(index(1));
  }

  std::size_t accumulateCloud(
    const sensor_msgs::msg::PointCloud2 & cloud, bool is_ground,
    grid_map::GridMap & map, std::vector<CellStatistics> & statistics) const
  {
    std::size_t accepted = 0U;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
      const std::size_t point_count =
        static_cast<std::size_t>(cloud.width) * cloud.height;
      const grid_map::Size size = map.getSize();

      for (std::size_t point = 0; point < point_count; ++point, ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        grid_map::Index index;
        if (!map.getIndex(grid_map::Position(*x, *y), index)) {
          continue;
        }
        auto & cell = statistics[flatIndex(index, size)];
        cell.maximum_surface = std::max(cell.maximum_surface, *z);
        if (is_ground) {
          ++cell.ground_count;
          cell.ground_sum += *z;
          cell.ground_squared_sum += static_cast<double>(*z) * *z;
        } else {
          ++cell.nonground_count;
          cell.maximum_nonground = std::max(cell.maximum_nonground, *z);
        }
        ++accepted;
      }
    } catch (const std::runtime_error & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "PointCloud2 requires FLOAT32 x/y/z fields: %s", exception.what());
    }
    return accepted;
  }

  void fillGroundHoles(grid_map::GridMap & map) const
  {
    if (fill_radius_cells_ == 0) {
      return;
    }
    const grid_map::Matrix source = map["ground_height"];
    auto & destination = map["ground_height"];
    auto & confidence = map["confidence"];
    const int rows = static_cast<int>(source.rows());
    const int columns = static_cast<int>(source.cols());

    for (int row = 0; row < rows; ++row) {
      for (int column = 0; column < columns; ++column) {
        if (std::isfinite(source(row, column))) {
          continue;
        }
        double weighted_sum = 0.0;
        double weight_sum = 0.0;
        int neighbors = 0;
        for (int dr = -fill_radius_cells_; dr <= fill_radius_cells_; ++dr) {
          for (int dc = -fill_radius_cells_; dc <= fill_radius_cells_; ++dc) {
            if (dr == 0 && dc == 0) {
              continue;
            }
            const int neighbor_row = row + dr;
            const int neighbor_column = column + dc;
            if (neighbor_row < 0 || neighbor_row >= rows ||
              neighbor_column < 0 || neighbor_column >= columns)
            {
              continue;
            }
            const float height = source(neighbor_row, neighbor_column);
            if (!std::isfinite(height)) {
              continue;
            }
            const double distance = std::hypot(static_cast<double>(dr), static_cast<double>(dc));
            const double weight = 1.0 / distance;
            weighted_sum += weight * height;
            weight_sum += weight;
            ++neighbors;
          }
        }
        if (neighbors >= 2 && weight_sum > 0.0) {
          destination(row, column) = static_cast<float>(weighted_sum / weight_sum);
          confidence(row, column) = 0.25F;
        }
      }
    }
  }

  static bool gradient(
    const grid_map::Matrix & height, int row, int column, int dr, int dc,
    double resolution, double & value)
  {
    const int previous_row = row - dr;
    const int previous_column = column - dc;
    const int next_row = row + dr;
    const int next_column = column + dc;
    const bool previous_valid = previous_row >= 0 && previous_row < height.rows() &&
      previous_column >= 0 && previous_column < height.cols() &&
      std::isfinite(height(previous_row, previous_column));
    const bool next_valid = next_row >= 0 && next_row < height.rows() &&
      next_column >= 0 && next_column < height.cols() &&
      std::isfinite(height(next_row, next_column));

    if (previous_valid && next_valid) {
      value = (height(next_row, next_column) - height(previous_row, previous_column)) /
        (2.0 * resolution);
      return true;
    }
    if (next_valid && std::isfinite(height(row, column))) {
      value = (height(next_row, next_column) - height(row, column)) / resolution;
      return true;
    }
    if (previous_valid && std::isfinite(height(row, column))) {
      value = (height(row, column) - height(previous_row, previous_column)) / resolution;
      return true;
    }
    return false;
  }

  void computeDerivedLayers(grid_map::GridMap & map) const
  {
    auto & ground = map["ground_height"];
    auto & surface = map["surface_height"];
    auto & soil = map["soil_height"];
    auto & slope = map["slope"];
    auto & roughness = map["roughness"];
    auto & reachability = map["reachability"];

    for (int row = 0; row < ground.rows(); ++row) {
      for (int column = 0; column < ground.cols(); ++column) {
        if (!std::isfinite(ground(row, column))) {
          continue;
        }
        if (!std::isfinite(surface(row, column))) {
          surface(row, column) = ground(row, column);
        } else {
          surface(row, column) = std::max(surface(row, column), ground(row, column));
        }
        soil(row, column) = std::max(0.0F, surface(row, column) - ground(row, column));

        double gradient_x = 0.0;
        double gradient_y = 0.0;
        const bool has_x = gradient(ground, row, column, 1, 0, resolution_, gradient_x);
        const bool has_y = gradient(ground, row, column, 0, 1, resolution_, gradient_y);
        if (has_x || has_y) {
          slope(row, column) = static_cast<float>(
            std::atan(std::hypot(gradient_x, gradient_y)) * kRadiansToDegrees);
        }

        const float slope_value = slope(row, column);
        const float roughness_value = roughness(row, column);
        const bool safe_slope = std::isfinite(slope_value) &&
          slope_value <= maximum_slope_deg_;
        const bool safe_roughness = !std::isfinite(roughness_value) ||
          roughness_value <= maximum_roughness_;
        const bool safe_step = soil(row, column) <= maximum_step_height_;
        reachability(row, column) = safe_slope && safe_roughness && safe_step ? 1.0F : 0.0F;
      }
    }
  }

  static grid_map_msgs::msg::GridMap toGridMapMessage(const grid_map::GridMap & map)
  {
    grid_map_msgs::msg::GridMap message;
    message.header.stamp = rclcpp::Time(map.getTimestamp());
    message.header.frame_id = map.getFrameId();
    message.info.resolution = map.getResolution();
    message.info.length_x = map.getLength().x();
    message.info.length_y = map.getLength().y();
    message.info.pose.position.x = map.getPosition().x();
    message.info.pose.position.y = map.getPosition().y();
    message.info.pose.position.z = 0.0;
    message.info.pose.orientation.w = 1.0;
    message.layers = map.getLayers();
    message.basic_layers = map.getBasicLayers();
    message.data.reserve(message.layers.size());

    for (const auto & layer : message.layers) {
      const auto & matrix = map[layer];
      std_msgs::msg::Float32MultiArray data;
      data.layout.dim.resize(2);
      data.layout.dim[0].label = "column_index";
      data.layout.dim[0].size = static_cast<std::uint32_t>(matrix.outerSize());
      data.layout.dim[0].stride = static_cast<std::uint32_t>(matrix.size());
      data.layout.dim[1].label = "row_index";
      data.layout.dim[1].size = static_cast<std::uint32_t>(matrix.innerSize());
      data.layout.dim[1].stride = static_cast<std::uint32_t>(matrix.innerSize());
      data.layout.data_offset = 0U;
      data.data.assign(matrix.data(), matrix.data() + matrix.size());
      message.data.emplace_back(std::move(data));
    }

    message.outer_start_index = static_cast<std::uint16_t>(map.getStartIndex()(0));
    message.inner_start_index = static_cast<std::uint16_t>(map.getStartIndex()(1));
    return message;
  }

  void buildAndPublish(
    const sensor_msgs::msg::PointCloud2 & ground_cloud,
    const sensor_msgs::msg::PointCloud2 & nonground_cloud)
  {
    const auto start = std::chrono::steady_clock::now();
    const std::vector<std::string> layers = {
      "ground_height", "surface_height", "soil_height", "slope", "roughness",
      "point_count", "confidence", "reachability"};
    grid_map::GridMap map(layers);
    map.setFrameId(frame_id_);
    map.setGeometry(
      grid_map::Length(length_x_, length_y_), resolution_,
      grid_map::Position(center_x_, center_y_));
    map.setTimestamp(rclcpp::Time(ground_cloud.header.stamp).nanoseconds());
    map.setBasicLayers({"ground_height"});
    for (const auto & layer : layers) {
      map[layer].setConstant(kNaN);
    }

    const grid_map::Size size = map.getSize();
    std::vector<CellStatistics> statistics(
      static_cast<std::size_t>(size(0)) * static_cast<std::size_t>(size(1)));
    const std::size_t accepted_ground = accumulateCloud(
      ground_cloud, true, map, statistics);
    const std::size_t accepted_nonground = accumulateCloud(
      nonground_cloud, false, map, statistics);

    std::size_t measured_cells = 0U;
    for (int row = 0; row < size(0); ++row) {
      for (int column = 0; column < size(1); ++column) {
        const grid_map::Index index(row, column);
        const auto & cell = statistics[flatIndex(index, size)];
        const std::uint32_t total_count = cell.ground_count + cell.nonground_count;
        if (total_count > 0U) {
          map.at("point_count", index) = static_cast<float>(total_count);
          map.at("surface_height", index) = cell.maximum_surface;
        }
        if (cell.ground_count >= static_cast<std::uint32_t>(minimum_ground_points_)) {
          const double mean = cell.ground_sum / cell.ground_count;
          const double variance = std::max(
            0.0, cell.ground_squared_sum / cell.ground_count - mean * mean);
          map.at("ground_height", index) = static_cast<float>(mean);
          map.at("roughness", index) = static_cast<float>(std::sqrt(variance));
          map.at("confidence", index) = std::min(
            1.0F, static_cast<float>(cell.ground_count) /
            static_cast<float>(minimum_ground_points_ * 2));
          ++measured_cells;
        }
      }
    }

    fillGroundHoles(map);
    computeDerivedLayers(map);

    grid_map_publisher_->publish(toGridMapMessage(map));

    const auto elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Grid map: ground=%zu nonground=%zu measured_cells=%zu size=%dx%d time=%.2f ms",
      accepted_ground, accepted_nonground, measured_cells, size(0), size(1), elapsed_ms);
  }

  std::string frame_id_;
  std::string ground_topic_;
  std::string nonground_topic_;
  std::string grid_map_topic_;
  double resolution_{0.10};
  double length_x_{14.0};
  double length_y_{16.0};
  double center_x_{5.0};
  double center_y_{0.0};
  int minimum_ground_points_{3};
  int fill_radius_cells_{2};
  double maximum_slope_deg_{25.0};
  double maximum_roughness_{0.08};
  double maximum_step_height_{0.25};

  std::mutex input_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_ground_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_nonground_;
  std::int64_t last_processed_stamp_{-1};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_subscription_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;
};

}  // namespace excavator_height_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<excavator_height_map::HeightMapNode>());
  rclcpp::shutdown();
  return 0;
}
