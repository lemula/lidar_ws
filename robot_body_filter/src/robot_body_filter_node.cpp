#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace robot_body_filter
{

namespace
{

struct Triangle
{
  Eigen::Vector3d a;
  Eigen::Vector3d b;
  Eigen::Vector3d c;
};

enum class ShapeType {Box, Cylinder, Sphere, Mesh};

struct CollisionShape
{
  std::string link_name;
  std::string collision_name;
  ShapeType type{ShapeType::Box};
  Eigen::Isometry3d collision_from_link{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d dimensions{Eigen::Vector3d::Zero()};
  double radius{0.0};
  double length{0.0};
  std::vector<Triangle> triangles;
  Eigen::Vector3d mesh_min{Eigen::Vector3d::Zero()};
  Eigen::Vector3d mesh_max{Eigen::Vector3d::Zero()};
};

Eigen::Isometry3d poseToEigen(const urdf::Pose & pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  pose.rotation.getQuaternion(qx, qy, qz, qw);
  transform.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

Eigen::Isometry3d transformToEigen(const geometry_msgs::msg::Transform & value)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::Quaterniond(
    value.rotation.w, value.rotation.x, value.rotation.y, value.rotation.z)
    .normalized().toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    value.translation.x, value.translation.y, value.translation.z);
  return transform;
}

std::string resolveMeshUri(const std::string & uri)
{
  constexpr char package_prefix[] = "package://";
  constexpr char file_prefix[] = "file://";
  if (uri.rfind(package_prefix, 0) == 0) {
    const std::string remainder = uri.substr(sizeof(package_prefix) - 1);
    const auto slash = remainder.find('/');
    if (slash == std::string::npos) {
      throw std::runtime_error("Invalid package mesh URI: " + uri);
    }
    const auto package_name = remainder.substr(0, slash);
    const auto relative_path = remainder.substr(slash + 1);
    return ament_index_cpp::get_package_share_directory(package_name) + "/" + relative_path;
  }
  if (uri.rfind(file_prefix, 0) == 0) {
    return uri.substr(sizeof(file_prefix) - 1);
  }
  return uri;
}

std::vector<Triangle> loadStl(
  const std::string & path, const Eigen::Vector3d & mesh_scale)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("Cannot open STL mesh: " + path);
  }

  input.seekg(0, std::ios::end);
  const auto file_size = static_cast<std::size_t>(input.tellg());
  input.seekg(0, std::ios::beg);

  std::array<char, 80> header{};
  std::uint32_t triangle_count = 0;
  input.read(header.data(), static_cast<std::streamsize>(header.size()));
  input.read(reinterpret_cast<char *>(&triangle_count), sizeof(triangle_count));

  std::vector<Triangle> triangles;
  if (input && file_size >= 84U + static_cast<std::size_t>(triangle_count) * 50U) {
    triangles.reserve(triangle_count);
    for (std::uint32_t index = 0; index < triangle_count; ++index) {
      float values[12]{};
      std::uint16_t attribute = 0;
      input.read(reinterpret_cast<char *>(values), sizeof(values));
      input.read(reinterpret_cast<char *>(&attribute), sizeof(attribute));
      if (!input) {
        throw std::runtime_error("Truncated binary STL mesh: " + path);
      }
      Triangle triangle;
      triangle.a = Eigen::Vector3d(values[3], values[4], values[5]).cwiseProduct(mesh_scale);
      triangle.b = Eigen::Vector3d(values[6], values[7], values[8]).cwiseProduct(mesh_scale);
      triangle.c = Eigen::Vector3d(values[9], values[10], values[11]).cwiseProduct(mesh_scale);
      triangles.push_back(triangle);
    }
    return triangles;
  }

  input.close();
  std::ifstream ascii(path);
  if (!ascii) {
    throw std::runtime_error("Cannot reopen ASCII STL mesh: " + path);
  }
  std::string token;
  std::vector<Eigen::Vector3d> vertices;
  while (ascii >> token) {
    if (token == "vertex") {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      ascii >> x >> y >> z;
      vertices.emplace_back(Eigen::Vector3d(x, y, z).cwiseProduct(mesh_scale));
    }
  }
  if (vertices.size() % 3U != 0U) {
    throw std::runtime_error("Invalid ASCII STL mesh: " + path);
  }
  triangles.reserve(vertices.size() / 3U);
  for (std::size_t index = 0; index < vertices.size(); index += 3U) {
    triangles.push_back({vertices[index], vertices[index + 1U], vertices[index + 2U]});
  }
  return triangles;
}

double pointTriangleDistanceSquared(const Eigen::Vector3d & point, const Triangle & triangle)
{
  const Eigen::Vector3d ab = triangle.b - triangle.a;
  const Eigen::Vector3d ac = triangle.c - triangle.a;
  const Eigen::Vector3d ap = point - triangle.a;
  const double d1 = ab.dot(ap);
  const double d2 = ac.dot(ap);
  if (d1 <= 0.0 && d2 <= 0.0) {
    return ap.squaredNorm();
  }

  const Eigen::Vector3d bp = point - triangle.b;
  const double d3 = ab.dot(bp);
  const double d4 = ac.dot(bp);
  if (d3 >= 0.0 && d4 <= d3) {
    return bp.squaredNorm();
  }

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    const double v = d1 / (d1 - d3);
    return (point - (triangle.a + v * ab)).squaredNorm();
  }

  const Eigen::Vector3d cp = point - triangle.c;
  const double d5 = ab.dot(cp);
  const double d6 = ac.dot(cp);
  if (d6 >= 0.0 && d5 <= d6) {
    return cp.squaredNorm();
  }

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    const double w = d2 / (d2 - d6);
    return (point - (triangle.a + w * ac)).squaredNorm();
  }

  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return (point - (triangle.b + w * (triangle.c - triangle.b))).squaredNorm();
  }

  const Eigen::Vector3d normal = ab.cross(ac);
  const double normal_squared = normal.squaredNorm();
  if (normal_squared < 1e-20) {
    return std::min({ap.squaredNorm(), bp.squaredNorm(), cp.squaredNorm()});
  }
  const double distance = normal.dot(ap);
  return distance * distance / normal_squared;
}

bool rayIntersectsTriangle(
  const Eigen::Vector3d & origin, const Eigen::Vector3d & direction,
  const Triangle & triangle, double & distance)
{
  constexpr double epsilon = 1e-10;
  const Eigen::Vector3d edge1 = triangle.b - triangle.a;
  const Eigen::Vector3d edge2 = triangle.c - triangle.a;
  const Eigen::Vector3d h = direction.cross(edge2);
  const double determinant = edge1.dot(h);
  if (std::abs(determinant) < epsilon) {
    return false;
  }
  const double inverse = 1.0 / determinant;
  const Eigen::Vector3d s = origin - triangle.a;
  const double u = inverse * s.dot(h);
  if (u < -epsilon || u > 1.0 + epsilon) {
    return false;
  }
  const Eigen::Vector3d q = s.cross(edge1);
  const double v = inverse * direction.dot(q);
  if (v < -epsilon || u + v > 1.0 + epsilon) {
    return false;
  }
  distance = inverse * edge2.dot(q);
  return distance > epsilon;
}

bool pointInsideMesh(
  const Eigen::Vector3d & point, const CollisionShape & shape, double padding)
{
  if ((point.array() < (shape.mesh_min.array() - padding)).any() ||
    (point.array() > (shape.mesh_max.array() + padding)).any())
  {
    return false;
  }

  if (padding > 0.0) {
    const double padding_squared = padding * padding;
    for (const auto & triangle : shape.triangles) {
      if (pointTriangleDistanceSquared(point, triangle) <= padding_squared) {
        return true;
      }
    }
  }

  const Eigen::Vector3d direction = Eigen::Vector3d(1.0, 0.371, 0.529).normalized();
  std::vector<double> intersections;
  intersections.reserve(shape.triangles.size());
  for (const auto & triangle : shape.triangles) {
    double distance = 0.0;
    if (rayIntersectsTriangle(point, direction, triangle, distance)) {
      intersections.push_back(distance);
    }
  }
  std::sort(intersections.begin(), intersections.end());
  const auto unique_end = std::unique(
    intersections.begin(), intersections.end(),
    [](double lhs, double rhs) {return std::abs(lhs - rhs) < 1e-7;});
  return (static_cast<std::size_t>(std::distance(intersections.begin(), unique_end)) % 2U) == 1U;
}

bool readCoordinate(
  const std::uint8_t * point_data, const sensor_msgs::msg::PointField & field,
  double & value)
{
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    float float_value = 0.0F;
    std::memcpy(&float_value, point_data + field.offset, sizeof(float_value));
    value = static_cast<double>(float_value);
    return true;
  }
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
    std::memcpy(&value, point_data + field.offset, sizeof(value));
    return true;
  }
  return false;
}

}  // namespace

class RobotBodyFilterNode : public rclcpp::Node
{
public:
  RobotBodyFilterNode()
  : Node("robot_body_filter"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/localmap/world_points");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/localmap/body_filtered_points");
    const auto description_topic = declare_parameter<std::string>(
      "robot_description_topic", "/robot_description");
    padding_ = declare_parameter<double>("padding", 0.02);
    scale_ = declare_parameter<double>("scale", 1.0);
    use_latest_tf_ = declare_parameter<bool>("use_latest_tf", true);
    require_all_transforms_ = declare_parameter<bool>("require_all_transforms", true);
    const auto ignored = declare_parameter<std::vector<std::string>>(
      "ignored_links", std::vector<std::string>{"world"});
    ignored_links_ = std::set<std::string>(ignored.begin(), ignored.end());
    const auto robot_description = declare_parameter<std::string>("robot_description", "");

    output_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::QoS(10));
    input_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RobotBodyFilterNode::pointCloudCallback, this, std::placeholders::_1));

    auto description_qos = rclcpp::QoS(1).reliable().transient_local();
    description_subscription_ = create_subscription<std_msgs::msg::String>(
      description_topic, description_qos,
      [this](const std_msgs::msg::String::SharedPtr message) {
        if (!model_ready_) {
          loadRobotModel(message->data);
        }
      });

    if (!robot_description.empty()) {
      loadRobotModel(robot_description);
    } else {
      RCLCPP_INFO(
        get_logger(), "Waiting for URDF on %s", description_topic.c_str());
    }

    RCLCPP_INFO(
      get_logger(), "Self filter: %s -> %s, padding=%.3f m, scale=%.3f",
      input_topic_.c_str(), output_topic_.c_str(), padding_, scale_);
  }

private:
  void loadRobotModel(const std::string & xml)
  {
    urdf::Model model;
    if (!model.initString(xml)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse robot_description URDF");
      return;
    }

    std::vector<CollisionShape> new_shapes;
    try {
      for (const auto & entry : model.links_) {
        const auto & link = entry.second;
        if (!link || ignored_links_.count(link->name) > 0U) {
          continue;
        }
        for (std::size_t index = 0; index < link->collision_array.size(); ++index) {
          const auto & collision = link->collision_array[index];
          if (!collision || !collision->geometry) {
            continue;
          }
          CollisionShape shape;
          shape.link_name = link->name;
          shape.collision_name = collision->name.empty() ?
            link->name + "_collision_" + std::to_string(index) : collision->name;
          shape.collision_from_link = poseToEigen(collision->origin).inverse();

          switch (collision->geometry->type) {
            case urdf::Geometry::BOX: {
              const auto box = std::dynamic_pointer_cast<urdf::Box>(collision->geometry);
              shape.type = ShapeType::Box;
              shape.dimensions = Eigen::Vector3d(box->dim.x, box->dim.y, box->dim.z);
              break;
            }
            case urdf::Geometry::CYLINDER: {
              const auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(collision->geometry);
              shape.type = ShapeType::Cylinder;
              shape.radius = cylinder->radius;
              shape.length = cylinder->length;
              break;
            }
            case urdf::Geometry::SPHERE: {
              const auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(collision->geometry);
              shape.type = ShapeType::Sphere;
              shape.radius = sphere->radius;
              break;
            }
            case urdf::Geometry::MESH: {
              const auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(collision->geometry);
              shape.type = ShapeType::Mesh;
              const Eigen::Vector3d mesh_scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
              shape.triangles = loadStl(resolveMeshUri(mesh->filename), mesh_scale);
              if (shape.triangles.empty()) {
                throw std::runtime_error("Mesh has no triangles: " + mesh->filename);
              }
              shape.mesh_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
              shape.mesh_max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
              for (const auto & triangle : shape.triangles) {
                shape.mesh_min = shape.mesh_min.cwiseMin(triangle.a).cwiseMin(triangle.b).cwiseMin(triangle.c);
                shape.mesh_max = shape.mesh_max.cwiseMax(triangle.a).cwiseMax(triangle.b).cwiseMax(triangle.c);
              }
              break;
            }
            default:
              RCLCPP_WARN(
                get_logger(), "Unsupported collision geometry on link %s",
                link->name.c_str());
              continue;
          }
          new_shapes.push_back(std::move(shape));
        }
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Failed to load collision model: %s", exception.what());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(model_mutex_);
      shapes_ = std::move(new_shapes);
      model_ready_ = !shapes_.empty();
    }
    RCLCPP_INFO(
      get_logger(), "Loaded %zu collision shapes from URDF", shapes_.size());
  }

  bool pointInsideShape(const Eigen::Vector3d & point, const CollisionShape & shape) const
  {
    switch (shape.type) {
      case ShapeType::Box: {
        const Eigen::Vector3d half = shape.dimensions * (0.5 * scale_) +
          Eigen::Vector3d::Constant(padding_);
        return (point.cwiseAbs().array() <= half.array()).all();
      }
      case ShapeType::Cylinder: {
        const double radius = shape.radius * scale_ + padding_;
        const double half_length = shape.length * scale_ * 0.5 + padding_;
        return point.head<2>().squaredNorm() <= radius * radius &&
               std::abs(point.z()) <= half_length;
      }
      case ShapeType::Sphere: {
        const double radius = shape.radius * scale_ + padding_;
        return point.squaredNorm() <= radius * radius;
      }
      case ShapeType::Mesh:
        return pointInsideMesh(point / scale_, shape, padding_ / scale_);
    }
    return false;
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(model_mutex_);
    if (!model_ready_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "No URDF collision model available; cloud not filtered");
      return;
    }

    const sensor_msgs::msg::PointField * x_field = nullptr;
    const sensor_msgs::msg::PointField * y_field = nullptr;
    const sensor_msgs::msg::PointField * z_field = nullptr;
    for (const auto & field : message->fields) {
      if (field.name == "x") {x_field = &field;}
      if (field.name == "y") {y_field = &field;}
      if (field.name == "z") {z_field = &field;}
    }
    if (!x_field || !y_field || !z_field) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Input PointCloud2 has no x/y/z fields");
      return;
    }

    std::unordered_map<std::string, Eigen::Isometry3d> link_from_cloud;
    std::set<std::string> missing_links;
    for (const auto & shape : shapes_) {
      if (link_from_cloud.count(shape.link_name) > 0U ||
        missing_links.count(shape.link_name) > 0U)
      {
        continue;
      }
      try {
        geometry_msgs::msg::TransformStamped transform;
        if (use_latest_tf_) {
          transform = tf_buffer_.lookupTransform(
            shape.link_name, message->header.frame_id, tf2::TimePointZero);
        } else {
          transform = tf_buffer_.lookupTransform(
            shape.link_name, message->header.frame_id,
            rclcpp::Time(message->header.stamp), rclcpp::Duration::from_seconds(0.05));
        }
        link_from_cloud.emplace(shape.link_name, transformToEigen(transform.transform));
      } catch (const tf2::TransformException & exception) {
        missing_links.insert(shape.link_name);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000, "Missing TF %s <- %s: %s",
          shape.link_name.c_str(), message->header.frame_id.c_str(), exception.what());
      }
    }
    if (require_all_transforms_ && !missing_links.empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 output = *message;
    output.data.resize(message->data.size());
    std::size_t output_offset = 0U;
    std::size_t input_points = 0U;
    std::size_t removed_points = 0U;

    for (std::uint32_t row = 0; row < message->height; ++row) {
      for (std::uint32_t column = 0; column < message->width; ++column) {
        const std::size_t input_offset = static_cast<std::size_t>(row) * message->row_step +
          static_cast<std::size_t>(column) * message->point_step;
        if (input_offset + message->point_step > message->data.size()) {
          continue;
        }
        ++input_points;
        const auto * point_data = message->data.data() + input_offset;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!readCoordinate(point_data, *x_field, x) ||
          !readCoordinate(point_data, *y_field, y) ||
          !readCoordinate(point_data, *z_field, z))
        {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000, "Only FLOAT32/FLOAT64 x/y/z fields are supported");
          return;
        }

        bool remove = false;
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
          const Eigen::Vector3d cloud_point(x, y, z);
          for (const auto & shape : shapes_) {
            const auto transform = link_from_cloud.find(shape.link_name);
            if (transform == link_from_cloud.end()) {
              continue;
            }
            const Eigen::Vector3d link_point = transform->second * cloud_point;
            const Eigen::Vector3d collision_point = shape.collision_from_link * link_point;
            if (pointInsideShape(collision_point, shape)) {
              remove = true;
              break;
            }
          }
        }

        if (remove) {
          ++removed_points;
        } else {
          std::memcpy(
            output.data.data() + output_offset, point_data, message->point_step);
          output_offset += message->point_step;
        }
      }
    }

    output.data.resize(output_offset);
    output.height = 1U;
    output.width = static_cast<std::uint32_t>(output_offset / message->point_step);
    output.row_step = output.width * output.point_step;
    output.is_dense = message->is_dense;
    output_publisher_->publish(output);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Body filter: input=%zu removed=%zu output=%u frame=%s",
      input_points, removed_points, output.width, message->header.frame_id.c_str());
  }

  std::string input_topic_;
  std::string output_topic_;
  double padding_{0.02};
  double scale_{1.0};
  bool use_latest_tf_{true};
  bool require_all_transforms_{true};
  bool model_ready_{false};
  std::set<std::string> ignored_links_;
  std::vector<CollisionShape> shapes_;
  std::mutex model_mutex_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_publisher_;
};

}  // namespace robot_body_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_body_filter::RobotBodyFilterNode>());
  rclcpp::shutdown();
  return 0;
}
