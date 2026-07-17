#include "excavator_dig_point/msg/entry_point.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace excavator_dig_point
{

namespace
{

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

float clamp01(float value)
{
  return std::clamp(value, 0.0F, 1.0F);
}

bool finite(float value)
{
  return std::isfinite(value);
}

double pointDistance(
  const geometry_msgs::msg::Point & first,
  const geometry_msgs::msg::Point & second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

struct Component
{
  int label{0};
  int area_cells{0};
  std::uint32_t pile_id{0U};
  double centroid_x{0.0};
  double centroid_y{0.0};
  float maximum_soil_height{0.0F};
};

struct PileTrack
{
  std::uint32_t pile_id{0U};
  double x{0.0};
  double y{0.0};
  std::uint64_t last_seen_frame{0U};
};

struct Candidate
{
  int row{0};
  int column{0};
  std::uint32_t pile_id{0U};
  geometry_msgs::msg::Point point;
  float score{0.0F};
  float confidence{0.0F};
  float relative_height{0.0F};
  float edge_distance_cells{0.0F};
};

struct LocalFeatures
{
  float soil_height{0.0F};
  float confidence{0.0F};
  float roughness{0.0F};
  float valid_ratio{0.0F};
};

}  // namespace

class DigPointNode : public rclcpp::Node
{
public:
  DigPointNode()
  : Node("excavator_dig_point")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/terrain/grid_map");
    required_frame_ = declare_parameter<std::string>("required_frame", "world");
    entry_point_topic_ = declare_parameter<std::string>(
      "entry_point_topic", "/digging/entry_point");
    entry_info_topic_ = declare_parameter<std::string>(
      "entry_info_topic", "/digging/entry_point_info");
    markers_topic_ = declare_parameter<std::string>(
      "markers_topic", "/digging/markers");
    pile_mask_topic_ = declare_parameter<std::string>(
      "pile_mask_topic", "/digging/pile_mask");
    score_map_topic_ = declare_parameter<std::string>(
      "score_map_topic", "/digging/candidate_scores");

    maximum_map_age_sec_ = declare_parameter<double>("maximum_map_age_sec", 1.0);
    minimum_valid_cells_ = declare_parameter<int>("minimum_valid_cells", 50);
    minimum_soil_height_ = declare_parameter<double>("minimum_soil_height", 0.15);
    minimum_confidence_ = declare_parameter<double>("minimum_confidence", 0.70);
    minimum_points_per_cell_ = declare_parameter<int>("minimum_points_per_cell", 3);
    opening_radius_cells_ = declare_parameter<int>("opening_radius_cells", 1);
    closing_radius_cells_ = declare_parameter<int>("closing_radius_cells", 2);
    minimum_pile_area_ = declare_parameter<double>("minimum_pile_area", 0.50);
    pile_tracking_max_distance_ = declare_parameter<double>(
      "pile_tracking_max_distance", 1.0);
    pile_track_timeout_frames_ = declare_parameter<int>("pile_track_timeout_frames", 5);

    minimum_relative_height_ = declare_parameter<double>("minimum_relative_height", 0.30);
    maximum_relative_height_ = declare_parameter<double>("maximum_relative_height", 0.60);
    minimum_slope_deg_ = declare_parameter<double>("minimum_slope_deg", 10.0);
    maximum_slope_deg_ = declare_parameter<double>("maximum_slope_deg", 45.0);
    minimum_edge_distance_cells_ = declare_parameter<double>(
      "minimum_edge_distance_cells", 2.0);
    edge_score_saturation_cells_ = declare_parameter<double>(
      "edge_score_saturation_cells", 8.0);
    local_window_radius_ = declare_parameter<double>("local_window_radius", 0.30);
    minimum_local_valid_ratio_ = declare_parameter<double>(
      "minimum_local_valid_ratio", 0.80);
    roughness_normalization_ = declare_parameter<double>(
      "roughness_normalization", 0.10);
    minimum_candidate_score_ = declare_parameter<double>("minimum_candidate_score", 0.50);
    surface_fit_radius_cells_ = declare_parameter<int>("surface_fit_radius_cells", 2);

    minimum_stable_frames_ = declare_parameter<int>("minimum_stable_frames", 3);
    maximum_target_jump_cells_ = declare_parameter<double>(
      "maximum_target_jump_cells", 2.0);
    maximum_candidate_dropout_frames_ = declare_parameter<int>(
      "maximum_candidate_dropout_frames", 2);
    switch_score_margin_ = declare_parameter<double>("switch_score_margin", 0.10);
    maximum_candidate_markers_ = declare_parameter<int>("maximum_candidate_markers", 200);

    validateParameters();

    auto output_qos = rclcpp::QoS(1).reliable().transient_local();
    entry_info_publisher_ = create_publisher<msg::EntryPoint>(entry_info_topic_, output_qos);
    entry_point_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(
      entry_point_topic_, output_qos);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      markers_topic_, output_qos);
    pile_mask_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      pile_mask_topic_, output_qos);
    score_map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      score_map_topic_, output_qos);

    subscription_ = create_subscription<grid_map_msgs::msg::GridMap>(
      input_topic_, output_qos,
      std::bind(&DigPointNode::mapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Dig-point selector: %s -> %s, soil>=%.2f m confidence>=%.2f stable_frames=%d",
      input_topic_.c_str(), entry_point_topic_.c_str(), minimum_soil_height_,
      minimum_confidence_, minimum_stable_frames_);
  }

private:
  void validateParameters() const
  {
    if (maximum_map_age_sec_ < 0.0 || minimum_valid_cells_ < 1 ||
      minimum_soil_height_ < 0.0 || minimum_confidence_ < 0.0 ||
      minimum_confidence_ > 1.0 || minimum_points_per_cell_ < 0 ||
      opening_radius_cells_ < 0 || closing_radius_cells_ < 0 ||
      minimum_pile_area_ <= 0.0 || local_window_radius_ <= 0.0 ||
      minimum_local_valid_ratio_ < 0.0 || minimum_local_valid_ratio_ > 1.0 ||
      minimum_relative_height_ < 0.0 ||
      maximum_relative_height_ <= minimum_relative_height_ ||
      minimum_stable_frames_ < 1 || maximum_candidate_dropout_frames_ < 0 ||
      maximum_candidate_markers_ < 1)
    {
      throw std::invalid_argument("invalid excavator_dig_point parameter range");
    }
  }

  bool decodeGridMap(
    const grid_map_msgs::msg::GridMap & message,
    grid_map::GridMap & map,
    std::string & status) const
  {
    if (message.layers.size() != message.data.size()) {
      status = "layer_count_mismatch";
      return false;
    }
    if (message.info.resolution <= 0.0 || message.info.length_x <= 0.0 ||
      message.info.length_y <= 0.0)
    {
      status = "invalid_geometry";
      return false;
    }
    if (std::abs(message.info.pose.orientation.x) > 1.0e-6 ||
      std::abs(message.info.pose.orientation.y) > 1.0e-6 ||
      std::abs(message.info.pose.orientation.z) > 1.0e-6 ||
      std::abs(message.info.pose.orientation.w - 1.0) > 1.0e-6)
    {
      status = "rotated_map_unsupported";
      return false;
    }

    map = grid_map::GridMap();
    map.setFrameId(message.header.frame_id);
    map.setTimestamp(rclcpp::Time(message.header.stamp).nanoseconds());
    map.setGeometry(
      grid_map::Length(message.info.length_x, message.info.length_y),
      message.info.resolution,
      grid_map::Position(
        message.info.pose.position.x, message.info.pose.position.y));

    const grid_map::Size expected_size = map.getSize();
    for (std::size_t layer_index = 0; layer_index < message.layers.size(); ++layer_index) {
      const auto & array = message.data[layer_index];
      if (array.layout.dim.size() != 2U) {
        status = "invalid_layer_layout:" + message.layers[layer_index];
        return false;
      }

      int rows = 0;
      int columns = 0;
      const bool column_major = array.layout.dim[0].label == "column_index" &&
        array.layout.dim[1].label == "row_index";
      const bool row_major = array.layout.dim[0].label == "row_index" &&
        array.layout.dim[1].label == "column_index";
      if (column_major) {
        columns = static_cast<int>(array.layout.dim[0].size);
        rows = static_cast<int>(array.layout.dim[1].size);
      } else if (row_major) {
        rows = static_cast<int>(array.layout.dim[0].size);
        columns = static_cast<int>(array.layout.dim[1].size);
      } else {
        status = "unsupported_layer_layout:" + message.layers[layer_index];
        return false;
      }
      if (rows != expected_size(0) || columns != expected_size(1) ||
        array.data.size() != static_cast<std::size_t>(rows * columns))
      {
        status = "layer_size_mismatch:" + message.layers[layer_index];
        return false;
      }

      grid_map::Matrix matrix(rows, columns);
      if (column_major) {
        std::copy(array.data.begin(), array.data.end(), matrix.data());
      } else {
        for (int row = 0; row < rows; ++row) {
          for (int column = 0; column < columns; ++column) {
            matrix(row, column) = array.data[static_cast<std::size_t>(row * columns + column)];
          }
        }
      }
      if (map.exists(message.layers[layer_index])) {
        status = "duplicate_layer:" + message.layers[layer_index];
        return false;
      }
      map.add(message.layers[layer_index], matrix);
    }
    map.setBasicLayers(message.basic_layers);
    map.setStartIndex(
      grid_map::Index(
        static_cast<int>(message.outer_start_index),
        static_cast<int>(message.inner_start_index)));
    return true;
  }

  bool validateInput(
    const grid_map_msgs::msg::GridMap & message,
    const grid_map::GridMap & map,
    std::string & status) const
  {
    if (message.header.frame_id.empty()) {
      status = "empty_frame_id";
      return false;
    }
    if (!required_frame_.empty() && message.header.frame_id != required_frame_) {
      status = "frame_mismatch:" + message.header.frame_id;
      return false;
    }
    const rclcpp::Time stamp(message.header.stamp);
    if (stamp.nanoseconds() <= 0) {
      status = "invalid_stamp";
      return false;
    }
    const double age = (now() - stamp).seconds();
    if (maximum_map_age_sec_ > 0.0 && age > maximum_map_age_sec_) {
      std::ostringstream stream;
      stream << "map_expired:" << age;
      status = stream.str();
      return false;
    }
    if (age < -0.5) {
      status = "map_stamp_in_future";
      return false;
    }
    for (const auto & layer : {"ground_height", "surface_height", "confidence"}) {
      if (!map.exists(layer)) {
        status = std::string("missing_layer:") + layer;
        return false;
      }
    }

    int valid_cells = 0;
    const auto & ground = map["ground_height"];
    const auto & surface = map["surface_height"];
    const auto & confidence = map["confidence"];
    for (int row = 0; row < ground.rows(); ++row) {
      for (int column = 0; column < ground.cols(); ++column) {
        if (finite(ground(row, column)) && finite(surface(row, column)) &&
          finite(confidence(row, column)))
        {
          ++valid_cells;
        }
      }
    }
    if (valid_cells < minimum_valid_cells_) {
      status = "insufficient_valid_cells:" + std::to_string(valid_cells);
      return false;
    }
    return true;
  }

  void ensureSoilHeight(grid_map::GridMap & map) const
  {
    if (map.exists("soil_height")) {
      return;
    }
    grid_map::Matrix soil = map["surface_height"] - map["ground_height"];
    for (int row = 0; row < soil.rows(); ++row) {
      for (int column = 0; column < soil.cols(); ++column) {
        if (finite(soil(row, column))) {
          soil(row, column) = std::max(0.0F, soil(row, column));
        }
      }
    }
    map.add("soil_height", soil);
  }

  cv::Mat buildRawPileMask(const grid_map::GridMap & map) const
  {
    const auto & ground = map["ground_height"];
    const auto & surface = map["surface_height"];
    const auto & soil = map["soil_height"];
    const auto & confidence = map["confidence"];
    const bool has_point_count = map.exists("point_count");
    const grid_map::Matrix * point_count = has_point_count ? &map["point_count"] : nullptr;
    cv::Mat mask(soil.rows(), soil.cols(), CV_8UC1, cv::Scalar(0));

    for (int row = 0; row < soil.rows(); ++row) {
      for (int column = 0; column < soil.cols(); ++column) {
        const bool enough_points = !has_point_count ||
          (finite((*point_count)(row, column)) &&
          (*point_count)(row, column) >= minimum_points_per_cell_);
        if (finite(ground(row, column)) && finite(surface(row, column)) &&
          finite(soil(row, column)) && finite(confidence(row, column)) &&
          soil(row, column) >= minimum_soil_height_ &&
          confidence(row, column) >= minimum_confidence_ && enough_points)
        {
          mask.at<std::uint8_t>(row, column) = 255U;
        }
      }
    }
    return mask;
  }

  static void morphology(cv::Mat & mask, int operation, int radius)
  {
    if (radius <= 0) {
      return;
    }
    const int diameter = 2 * radius + 1;
    const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(diameter, diameter));
    cv::morphologyEx(mask, mask, operation, kernel);
  }

  std::vector<Component> extractComponents(
    const grid_map::GridMap & map,
    const cv::Mat & clean_mask,
    cv::Mat & retained_mask,
    cv::Mat & labels)
  {
    cv::Mat statistics;
    cv::Mat centroids;
    const int label_count = cv::connectedComponentsWithStats(
      clean_mask, labels, statistics, centroids, 8, CV_32S);
    retained_mask = cv::Mat::zeros(clean_mask.size(), CV_8UC1);
    const int minimum_area_cells = std::max(
      1, static_cast<int>(std::ceil(minimum_pile_area_ /
      (map.getResolution() * map.getResolution()))));
    std::vector<Component> components;

    for (int label = 1; label < label_count; ++label) {
      const int area = statistics.at<int>(label, cv::CC_STAT_AREA);
      if (area < minimum_area_cells) {
        continue;
      }
      Component component;
      component.label = label;
      component.area_cells = area;
      const int centroid_column = std::clamp(
        static_cast<int>(std::lround(centroids.at<double>(label, 0))),
        0, labels.cols - 1);
      const int centroid_row = std::clamp(
        static_cast<int>(std::lround(centroids.at<double>(label, 1))),
        0, labels.rows - 1);
      grid_map::Position centroid_position;
      map.getPosition(
        grid_map::Index(centroid_row, centroid_column), centroid_position);
      component.centroid_x = centroid_position.x();
      component.centroid_y = centroid_position.y();

      float maximum_soil = 0.0F;
      for (int row = 0; row < labels.rows; ++row) {
        for (int column = 0; column < labels.cols; ++column) {
          if (labels.at<int>(row, column) == label) {
            retained_mask.at<std::uint8_t>(row, column) = 255U;
            maximum_soil = std::max(maximum_soil, map.at(
                "soil_height", grid_map::Index(row, column)));
          }
        }
      }
      component.maximum_soil_height = maximum_soil;
      components.push_back(component);
    }
    assignStablePileIds(components);
    return components;
  }

  void assignStablePileIds(std::vector<Component> & components)
  {
    std::set<std::uint32_t> used_tracks;
    for (auto & component : components) {
      PileTrack * best_track = nullptr;
      double best_distance = pile_tracking_max_distance_;
      for (auto & track : pile_tracks_) {
        if (used_tracks.count(track.pile_id) != 0U ||
          frame_sequence_ > track.last_seen_frame +
          static_cast<std::uint64_t>(pile_track_timeout_frames_))
        {
          continue;
        }
        const double distance = std::hypot(
          component.centroid_x - track.x, component.centroid_y - track.y);
        if (distance <= best_distance) {
          best_distance = distance;
          best_track = &track;
        }
      }
      if (best_track == nullptr) {
        PileTrack track;
        track.pile_id = next_pile_id_++;
        track.x = component.centroid_x;
        track.y = component.centroid_y;
        track.last_seen_frame = frame_sequence_;
        pile_tracks_.push_back(track);
        component.pile_id = track.pile_id;
      } else {
        best_track->x = component.centroid_x;
        best_track->y = component.centroid_y;
        best_track->last_seen_frame = frame_sequence_;
        component.pile_id = best_track->pile_id;
      }
      used_tracks.insert(component.pile_id);
    }
    pile_tracks_.erase(
      std::remove_if(
        pile_tracks_.begin(), pile_tracks_.end(),
        [this](const PileTrack & track) {
          return frame_sequence_ > track.last_seen_frame +
                 static_cast<std::uint64_t>(pile_track_timeout_frames_);
        }),
      pile_tracks_.end());
  }

  LocalFeatures computeLocalFeatures(
    const grid_map::GridMap & map,
    int center_row,
    int center_column,
    int radius_cells) const
  {
    LocalFeatures features;
    const auto & ground = map["ground_height"];
    const auto & surface = map["surface_height"];
    const auto & soil = map["soil_height"];
    const auto & confidence = map["confidence"];
    const bool has_roughness = map.exists("roughness");
    const grid_map::Matrix * roughness = has_roughness ? &map["roughness"] : nullptr;
    int total_cells = 0;
    int valid_cells = 0;
    int roughness_cells = 0;
    double soil_sum = 0.0;
    double confidence_sum = 0.0;
    double roughness_sum = 0.0;

    for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
      for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
        if (dr * dr + dc * dc > radius_cells * radius_cells) {
          continue;
        }
        ++total_cells;
        const int row = center_row + dr;
        const int column = center_column + dc;
        if (row < 0 || row >= soil.rows() || column < 0 || column >= soil.cols()) {
          continue;
        }
        if (!finite(ground(row, column)) || !finite(surface(row, column)) ||
          !finite(soil(row, column)) || !finite(confidence(row, column)))
        {
          continue;
        }
        ++valid_cells;
        soil_sum += soil(row, column);
        confidence_sum += confidence(row, column);
        if (has_roughness && finite((*roughness)(row, column))) {
          roughness_sum += (*roughness)(row, column);
          ++roughness_cells;
        }
      }
    }
    if (valid_cells > 0) {
      features.soil_height = static_cast<float>(soil_sum / valid_cells);
      features.confidence = static_cast<float>(confidence_sum / valid_cells);
    }
    if (roughness_cells > 0) {
      features.roughness = static_cast<float>(roughness_sum / roughness_cells);
    }
    if (total_cells > 0) {
      features.valid_ratio = static_cast<float>(valid_cells) / total_cells;
    }
    return features;
  }

  float fittedSurfaceHeight(
    const grid_map::GridMap & map,
    int center_row,
    int center_column) const
  {
    const auto & surface = map["surface_height"];
    std::vector<cv::Vec3d> rows;
    std::vector<double> heights;
    for (int dr = -surface_fit_radius_cells_; dr <= surface_fit_radius_cells_; ++dr) {
      for (int dc = -surface_fit_radius_cells_; dc <= surface_fit_radius_cells_; ++dc) {
        const int row = center_row + dr;
        const int column = center_column + dc;
        if (row < 0 || row >= surface.rows() || column < 0 || column >= surface.cols() ||
          !finite(surface(row, column)))
        {
          continue;
        }
        grid_map::Position position;
        if (!map.getPosition(grid_map::Index(row, column), position)) {
          continue;
        }
        rows.emplace_back(position.x(), position.y(), 1.0);
        heights.push_back(surface(row, column));
      }
    }
    if (rows.size() < 3U) {
      return surface(center_row, center_column);
    }
    cv::Mat design(static_cast<int>(rows.size()), 3, CV_64F);
    cv::Mat observations(static_cast<int>(rows.size()), 1, CV_64F);
    for (std::size_t index = 0; index < rows.size(); ++index) {
      design.at<double>(static_cast<int>(index), 0) = rows[index][0];
      design.at<double>(static_cast<int>(index), 1) = rows[index][1];
      design.at<double>(static_cast<int>(index), 2) = 1.0;
      observations.at<double>(static_cast<int>(index), 0) = heights[index];
    }
    cv::Mat coefficients;
    if (!cv::solve(design, observations, coefficients, cv::DECOMP_SVD)) {
      return surface(center_row, center_column);
    }
    grid_map::Position center;
    map.getPosition(grid_map::Index(center_row, center_column), center);
    return static_cast<float>(
      coefficients.at<double>(0, 0) * center.x() +
      coefficients.at<double>(1, 0) * center.y() +
      coefficients.at<double>(2, 0));
  }

  std::vector<Candidate> generateCandidates(
    const grid_map::GridMap & map,
    const cv::Mat & labels,
    const std::vector<Component> & components,
    cv::Mat & score_map) const
  {
    score_map = cv::Mat(
      labels.rows, labels.cols, CV_32FC1,
      cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    std::unordered_map<int, Component> component_by_label;
    for (const auto & component : components) {
      component_by_label.emplace(component.label, component);
    }
    std::unordered_map<int, cv::Mat> distance_by_label;
    for (const auto & component : components) {
      cv::Mat component_mask = labels == component.label;
      cv::Mat distance;
      cv::distanceTransform(component_mask, distance, cv::DIST_L2, 5);
      distance_by_label.emplace(component.label, std::move(distance));
    }

    const auto & soil = map["soil_height"];
    const auto & surface = map["surface_height"];
    const bool has_slope = map.exists("slope");
    const grid_map::Matrix * slope = has_slope ? &map["slope"] : nullptr;
    const int local_radius_cells = std::max(
      1, static_cast<int>(std::ceil(local_window_radius_ / map.getResolution())));
    std::vector<Candidate> candidates;
    int component_cells = 0;
    int relative_height_rejections = 0;
    int slope_rejections = 0;
    int edge_rejections = 0;
    int local_validity_rejections = 0;

    for (int row = 0; row < labels.rows; ++row) {
      for (int column = 0; column < labels.cols; ++column) {
        const int label = labels.at<int>(row, column);
        const auto component_iterator = component_by_label.find(label);
        if (component_iterator == component_by_label.end()) {
          continue;
        }
        ++component_cells;
        const Component & component = component_iterator->second;
        if (component.maximum_soil_height <= 0.0F || !finite(soil(row, column)) ||
          !finite(surface(row, column)))
        {
          continue;
        }
        const float relative_height = soil(row, column) / component.maximum_soil_height;
        if (relative_height < minimum_relative_height_ ||
          relative_height > maximum_relative_height_)
        {
          ++relative_height_rejections;
          continue;
        }
        if (has_slope) {
          const float slope_value = (*slope)(row, column);
          if (!finite(slope_value) || slope_value < minimum_slope_deg_ ||
            slope_value > maximum_slope_deg_)
          {
            ++slope_rejections;
            continue;
          }
        }
        const float edge_distance = distance_by_label.at(label).at<float>(row, column);
        if (edge_distance < minimum_edge_distance_cells_) {
          ++edge_rejections;
          continue;
        }
        const LocalFeatures local = computeLocalFeatures(
          map, row, column, local_radius_cells);
        if (local.valid_ratio < minimum_local_valid_ratio_) {
          ++local_validity_rejections;
          continue;
        }

        const float local_soil_score = clamp01(
          local.soil_height / component.maximum_soil_height);
        const float confidence_score = clamp01(local.confidence);
        const float edge_score = clamp01(
          edge_distance / static_cast<float>(edge_score_saturation_cells_));
        const float relative_midpoint = static_cast<float>(
          0.5 * (minimum_relative_height_ + maximum_relative_height_));
        const float relative_half_range = static_cast<float>(
          0.5 * (maximum_relative_height_ - minimum_relative_height_));
        const float relative_score = clamp01(
          1.0F - std::abs(relative_height - relative_midpoint) / relative_half_range);
        const float roughness_penalty = roughness_normalization_ > 0.0 ?
          clamp01(local.roughness / static_cast<float>(roughness_normalization_)) : 0.0F;
        const float score = clamp01(
          0.35F * local_soil_score +
          0.25F * confidence_score +
          0.20F * edge_score +
          0.10F * relative_score -
          0.10F * roughness_penalty);

        Candidate candidate;
        candidate.row = row;
        candidate.column = column;
        candidate.pile_id = component.pile_id;
        candidate.score = score;
        candidate.confidence = local.confidence;
        candidate.relative_height = relative_height;
        candidate.edge_distance_cells = edge_distance;
        grid_map::Position position;
        map.getPosition(grid_map::Index(row, column), position);
        candidate.point.x = position.x();
        candidate.point.y = position.y();
        candidate.point.z = fittedSurfaceHeight(map, row, column);
        candidates.push_back(candidate);
        score_map.at<float>(row, column) = score;
      }
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const Candidate & first, const Candidate & second) {
        return first.score > second.score;
      });
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Candidate filters: cells=%d relative=%d slope=%d edge=%d local=%d kept=%zu",
      component_cells, relative_height_rejections, slope_rejections,
      edge_rejections, local_validity_rejections, candidates.size());
    return candidates;
  }

  std::optional<Candidate> temporalSelection(
    const std::vector<Candidate> & candidates,
    double resolution,
    std::string & status)
  {
    if (candidates.empty() || candidates.front().score < minimum_candidate_score_) {
      ++candidate_dropout_frames_;
      if (candidate_dropout_frames_ > maximum_candidate_dropout_frames_) {
        resetTemporalState();
        status = "no_valid_candidate";
        return std::nullopt;
      }
      if (current_target_.has_value()) {
        status = "valid_held:" + std::to_string(candidate_dropout_frames_) + "/" +
          std::to_string(maximum_candidate_dropout_frames_);
        return current_target_;
      }
      if (pending_target_.has_value()) {
        status = "stabilizing_dropout:" + std::to_string(pending_frames_) + "/" +
          std::to_string(minimum_stable_frames_);
      } else {
        status = "no_valid_candidate";
      }
      return std::nullopt;
    }
    candidate_dropout_frames_ = 0;
    const Candidate & best = candidates.front();
    const double maximum_jump = maximum_target_jump_cells_ * resolution;

    if (!current_target_.has_value()) {
      updatePending(candidates, best, maximum_jump);
      if (pending_frames_ >= minimum_stable_frames_) {
        current_target_ = pending_target_;
        pending_target_.reset();
        pending_frames_ = 0;
        status = "valid";
        return current_target_;
      }
      status = "stabilizing:" + std::to_string(pending_frames_) + "/" +
        std::to_string(minimum_stable_frames_);
      return std::nullopt;
    }

    const Candidate * nearest_current = nullptr;
    double nearest_distance = maximum_jump;
    for (const auto & candidate : candidates) {
      if (candidate.pile_id != current_target_->pile_id) {
        continue;
      }
      const double distance = pointDistance(candidate.point, current_target_->point);
      if (distance <= nearest_distance) {
        nearest_distance = distance;
        nearest_current = &candidate;
      }
    }
    if (nearest_current == nullptr) {
      current_target_.reset();
      pending_target_.reset();
      pending_frames_ = 0;
      updatePending(candidates, best, maximum_jump);
      status = "stabilizing:" + std::to_string(pending_frames_) + "/" +
        std::to_string(minimum_stable_frames_);
      return std::nullopt;
    }

    const bool best_is_new_target = pointDistance(best.point, nearest_current->point) > maximum_jump ||
      best.pile_id != nearest_current->pile_id;
    if (best_is_new_target &&
      best.score >= nearest_current->score + switch_score_margin_)
    {
      updatePending(candidates, best, maximum_jump);
      if (pending_frames_ >= minimum_stable_frames_) {
        current_target_ = pending_target_;
        pending_target_.reset();
        pending_frames_ = 0;
      } else {
        current_target_ = *nearest_current;
      }
    } else {
      current_target_ = *nearest_current;
      pending_target_.reset();
      pending_frames_ = 0;
    }
    status = "valid";
    return current_target_;
  }

  void updatePending(
    const std::vector<Candidate> & candidates,
    const Candidate & fallback,
    double maximum_jump)
  {
    if (pending_target_.has_value()) {
      const Candidate * nearest_pending = nullptr;
      double nearest_distance = maximum_jump;
      for (const auto & candidate : candidates) {
        if (candidate.pile_id != pending_target_->pile_id) {
          continue;
        }
        const double distance = pointDistance(candidate.point, pending_target_->point);
        if (distance <= nearest_distance) {
          nearest_distance = distance;
          nearest_pending = &candidate;
        }
      }
      if (nearest_pending != nullptr) {
        pending_target_ = *nearest_pending;
        ++pending_frames_;
        return;
      }
    }
    pending_target_ = fallback;
    pending_frames_ = 1;
  }

  void resetTemporalState()
  {
    current_target_.reset();
    pending_target_.reset();
    pending_frames_ = 0;
    candidate_dropout_frames_ = 0;
  }

  nav_msgs::msg::OccupancyGrid makeDebugGrid(
    const grid_map::GridMap & map,
    const std_msgs::msg::Header & header,
    const cv::Mat & values,
    bool scores) const
  {
    nav_msgs::msg::OccupancyGrid output;
    output.header = header;
    output.info.map_load_time = header.stamp;
    output.info.resolution = static_cast<float>(map.getResolution());
    output.info.width = static_cast<std::uint32_t>(map.getSize()(0));
    output.info.height = static_cast<std::uint32_t>(map.getSize()(1));
    output.info.origin.position.x = map.getPosition().x() - 0.5 * map.getLength().x();
    output.info.origin.position.y = map.getPosition().y() - 0.5 * map.getLength().y();
    output.info.origin.orientation.w = 1.0;
    output.data.assign(
      static_cast<std::size_t>(output.info.width) * output.info.height,
      scores ? static_cast<std::int8_t>(-1) : static_cast<std::int8_t>(0));

    for (int row = 0; row < values.rows; ++row) {
      for (int column = 0; column < values.cols; ++column) {
        grid_map::Position position;
        if (!map.getPosition(grid_map::Index(row, column), position)) {
          continue;
        }
        const int x = static_cast<int>(std::floor(
          (position.x() - output.info.origin.position.x) / map.getResolution()));
        const int y = static_cast<int>(std::floor(
          (position.y() - output.info.origin.position.y) / map.getResolution()));
        if (x < 0 || y < 0 || x >= static_cast<int>(output.info.width) ||
          y >= static_cast<int>(output.info.height))
        {
          continue;
        }
        const std::size_t output_index = static_cast<std::size_t>(y) * output.info.width + x;
        if (scores) {
          const float score = values.at<float>(row, column);
          if (finite(score)) {
            output.data[output_index] = static_cast<std::int8_t>(
              std::lround(100.0F * clamp01(score)));
          }
        } else {
          output.data[output_index] = values.at<std::uint8_t>(row, column) > 0U ? 100 : 0;
        }
      }
    }
    return output;
  }

  visualization_msgs::msg::Marker makeDeleteAllMarker(
    const std_msgs::msg::Header & header) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    return marker;
  }

  visualization_msgs::msg::Marker makeStatusMarker(
    const grid_map::GridMap & map,
    const std_msgs::msg::Header & header,
    const std::string & status,
    bool valid) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "dig_point_status";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = map.getPosition().x();
    marker.pose.position.y = map.getPosition().y();
    marker.pose.position.z = 1.5;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.35;
    marker.color.r = valid ? 0.1F : 1.0F;
    marker.color.g = valid ? 1.0F : 0.2F;
    marker.color.b = 0.1F;
    marker.color.a = 1.0F;
    marker.text = status;
    return marker;
  }

  void publishMarkers(
    const grid_map::GridMap & map,
    const std_msgs::msg::Header & header,
    const cv::Mat & pile_mask,
    const std::vector<Candidate> & candidates,
    const std::optional<Candidate> & selected,
    const std::string & status) const
  {
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(makeDeleteAllMarker(header));

    visualization_msgs::msg::Marker pile_marker;
    pile_marker.header = header;
    pile_marker.ns = "clean_pile_mask";
    pile_marker.id = 0;
    pile_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    pile_marker.action = visualization_msgs::msg::Marker::ADD;
    pile_marker.pose.orientation.w = 1.0;
    pile_marker.scale.x = map.getResolution();
    pile_marker.scale.y = map.getResolution();
    pile_marker.scale.z = 0.04;
    pile_marker.color.r = 0.0F;
    pile_marker.color.g = 0.8F;
    pile_marker.color.b = 1.0F;
    pile_marker.color.a = 0.35F;
    const auto & surface = map["surface_height"];
    for (int row = 0; row < pile_mask.rows; ++row) {
      for (int column = 0; column < pile_mask.cols; ++column) {
        if (pile_mask.at<std::uint8_t>(row, column) == 0U) {
          continue;
        }
        grid_map::Position position;
        if (!map.getPosition(grid_map::Index(row, column), position)) {
          continue;
        }
        geometry_msgs::msg::Point point;
        point.x = position.x();
        point.y = position.y();
        point.z = finite(surface(row, column)) ? surface(row, column) + 0.02 : 0.02;
        pile_marker.points.push_back(point);
      }
    }
    markers.markers.push_back(std::move(pile_marker));

    visualization_msgs::msg::Marker candidate_marker;
    candidate_marker.header = header;
    candidate_marker.ns = "dig_candidates";
    candidate_marker.id = 0;
    candidate_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    candidate_marker.action = visualization_msgs::msg::Marker::ADD;
    candidate_marker.pose.orientation.w = 1.0;
    candidate_marker.scale.x = 0.10;
    candidate_marker.scale.y = 0.10;
    candidate_marker.scale.z = 0.10;
    const int marker_count = std::min(
      static_cast<int>(candidates.size()), maximum_candidate_markers_);
    for (int index = 0; index < marker_count; ++index) {
      candidate_marker.points.push_back(candidates[static_cast<std::size_t>(index)].point);
      std_msgs::msg::ColorRGBA color;
      color.r = 1.0F - candidates[static_cast<std::size_t>(index)].score;
      color.g = candidates[static_cast<std::size_t>(index)].score;
      color.b = 0.1F;
      color.a = 0.85F;
      candidate_marker.colors.push_back(color);
    }
    markers.markers.push_back(std::move(candidate_marker));

    if (selected.has_value()) {
      visualization_msgs::msg::Marker target;
      target.header = header;
      target.ns = "dig_entry_point";
      target.id = 0;
      target.type = visualization_msgs::msg::Marker::SPHERE;
      target.action = visualization_msgs::msg::Marker::ADD;
      target.pose.position = selected->point;
      target.pose.orientation.w = 1.0;
      target.scale.x = 0.30;
      target.scale.y = 0.30;
      target.scale.z = 0.30;
      target.color.r = 1.0F;
      target.color.g = 0.1F;
      target.color.b = 1.0F;
      target.color.a = 1.0F;
      markers.markers.push_back(target);

      visualization_msgs::msg::Marker label = target;
      label.ns = "dig_entry_label";
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.pose.position.z += 0.45;
      label.scale.x = 0.0;
      label.scale.y = 0.0;
      label.scale.z = 0.25;
      std::ostringstream stream;
      stream << "entry score=" << selected->score << " pile=" << selected->pile_id;
      label.text = stream.str();
      markers.markers.push_back(label);
    }
    markers.markers.push_back(makeStatusMarker(map, header, status, selected.has_value()));
    marker_publisher_->publish(markers);
  }

  void publishResult(
    const std_msgs::msg::Header & header,
    const std::optional<Candidate> & selected,
    const std::string & status)
  {
    msg::EntryPoint information;
    information.header = header;
    information.valid = selected.has_value();
    information.status = status;
    if (selected.has_value()) {
      information.point = selected->point;
      information.score = selected->score;
      information.confidence = selected->confidence;
      information.pile_id = selected->pile_id;
      geometry_msgs::msg::PointStamped point;
      point.header = header;
      point.point = selected->point;
      entry_point_publisher_->publish(point);
    }
    entry_info_publisher_->publish(information);
  }

  void publishInvalid(
    const grid_map_msgs::msg::GridMap & message,
    const std::string & status,
    const grid_map::GridMap * map = nullptr)
  {
    resetTemporalState();
    publishResult(message.header, std::nullopt, status);
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(makeDeleteAllMarker(message.header));
    if (map != nullptr) {
      markers.markers.push_back(makeStatusMarker(*map, message.header, status, false));
    }
    marker_publisher_->publish(markers);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000, "No dig point: %s", status.c_str());
  }

  void mapCallback(const grid_map_msgs::msg::GridMap::ConstSharedPtr message)
  {
    const auto start = std::chrono::steady_clock::now();
    ++frame_sequence_;
    grid_map::GridMap map;
    std::string status;
    if (!decodeGridMap(*message, map, status)) {
      publishInvalid(*message, status);
      return;
    }
    if (!validateInput(*message, map, status)) {
      publishInvalid(*message, status, &map);
      return;
    }
    ensureSoilHeight(map);

    cv::Mat raw_mask = buildRawPileMask(map);
    morphology(raw_mask, cv::MORPH_OPEN, opening_radius_cells_);
    morphology(raw_mask, cv::MORPH_CLOSE, closing_radius_cells_);
    cv::Mat retained_mask;
    cv::Mat labels;
    const std::vector<Component> components = extractComponents(
      map, raw_mask, retained_mask, labels);
    if (components.empty()) {
      cv::Mat empty_scores(
        retained_mask.rows, retained_mask.cols, CV_32FC1, cv::Scalar(kNaN));
      pile_mask_publisher_->publish(makeDebugGrid(
          map, message->header, retained_mask, false));
      score_map_publisher_->publish(makeDebugGrid(
          map, message->header, empty_scores, true));
      const std::vector<Candidate> no_candidates;
      const std::optional<Candidate> selected = temporalSelection(
        no_candidates, map.getResolution(), status);
      if (status == "no_valid_candidate") {
        status = "no_pile_after_filter";
      }
      publishResult(message->header, selected, status);
      publishMarkers(
        map, message->header, retained_mask, no_candidates, selected, status);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "No retained pile: valid=%s status=%s",
        selected.has_value() ? "true" : "false", status.c_str());
      return;
    }

    cv::Mat score_map;
    const std::vector<Candidate> candidates = generateCandidates(
      map, labels, components, score_map);
    pile_mask_publisher_->publish(makeDebugGrid(
        map, message->header, retained_mask, false));
    score_map_publisher_->publish(makeDebugGrid(
        map, message->header, score_map, true));

    const std::optional<Candidate> selected = temporalSelection(
      candidates, map.getResolution(), status);
    publishResult(message->header, selected, status);
    publishMarkers(map, message->header, retained_mask, candidates, selected, status);

    const auto elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Dig point: piles=%zu candidates=%zu valid=%s status=%s time=%.2f ms",
      components.size(), candidates.size(), selected.has_value() ? "true" : "false",
      status.c_str(), elapsed_ms);
  }

  std::string input_topic_;
  std::string required_frame_;
  std::string entry_point_topic_;
  std::string entry_info_topic_;
  std::string markers_topic_;
  std::string pile_mask_topic_;
  std::string score_map_topic_;

  double maximum_map_age_sec_{1.0};
  int minimum_valid_cells_{50};
  double minimum_soil_height_{0.15};
  double minimum_confidence_{0.70};
  int minimum_points_per_cell_{3};
  int opening_radius_cells_{1};
  int closing_radius_cells_{2};
  double minimum_pile_area_{0.50};
  double pile_tracking_max_distance_{1.0};
  int pile_track_timeout_frames_{5};
  double minimum_relative_height_{0.30};
  double maximum_relative_height_{0.60};
  double minimum_slope_deg_{10.0};
  double maximum_slope_deg_{45.0};
  double minimum_edge_distance_cells_{2.0};
  double edge_score_saturation_cells_{8.0};
  double local_window_radius_{0.30};
  double minimum_local_valid_ratio_{0.80};
  double roughness_normalization_{0.10};
  double minimum_candidate_score_{0.50};
  int surface_fit_radius_cells_{2};
  int minimum_stable_frames_{3};
  double maximum_target_jump_cells_{2.0};
  int maximum_candidate_dropout_frames_{2};
  double switch_score_margin_{0.10};
  int maximum_candidate_markers_{200};

  std::uint64_t frame_sequence_{0U};
  std::uint32_t next_pile_id_{1U};
  std::vector<PileTrack> pile_tracks_;
  std::optional<Candidate> current_target_;
  std::optional<Candidate> pending_target_;
  int pending_frames_{0};
  int candidate_dropout_frames_{0};

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscription_;
  rclcpp::Publisher<msg::EntryPoint>::SharedPtr entry_info_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr entry_point_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pile_mask_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr score_map_publisher_;
};

}  // namespace excavator_dig_point

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<excavator_dig_point::DigPointNode>());
  rclcpp::shutdown();
  return 0;
}
