#include "bievr_lio_ros/pole_extractor.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "bievr_lio/config_loader.h"
#include "bievr_lio/log++.h"
#include "bievr_ros_common/conversions.h"

namespace bievr {
namespace {

struct PolePoint {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

using PoleCloud = std::vector<PolePoint>;

struct Cell {
  int x = 0;
  int y = 0;

  bool operator==(const Cell& other) const { return x == other.x && y == other.y; }
};

struct CellHash {
  size_t operator()(const Cell& cell) const {
    const uint64_t ux = static_cast<uint32_t>(cell.x);
    const uint64_t uy = static_cast<uint32_t>(cell.y);
    uint64_t value = (ux << 32U) | uy;
    value ^= value >> 33U;
    value *= 0xff51afd7ed558ccdULL;
    value ^= value >> 33U;
    return static_cast<size_t>(value);
  }
};

struct Voxel {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const Voxel& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelHash {
  size_t operator()(const Voxel& voxel) const {
    size_t seed = CellHash()({voxel.x, voxel.y});
    seed ^= static_cast<size_t>(static_cast<uint32_t>(voxel.z)) + 0x9e3779b9U + (seed << 6U) +
            (seed >> 2U);
    return seed;
  }
};

struct PointSum {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  size_t count = 0;
};

struct GridStat {
  int count = 0;
  std::unordered_set<int> layers;
};

struct ExtractionFrame {
  uint64_t stamp = 0;
  double odom_distance_m = 0.0;
  Transform pose;
  Pointcloud cloud;
};

struct ReferencePose {
  uint64_t stamp = 0;
  Transform pose;
};

bool loadTumTrajectory(const std::string& path, std::vector<ReferencePose>& trajectory,
                       std::string* message) {
  std::ifstream input(path);
  if (!input) {
    if (message) *message = "Cannot open mapping trajectory '" + path + "'.";
    return false;
  }
  trajectory.clear();
  std::string line;
  size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') continue;
    std::istringstream stream(line);
    double stamp_s = 0.0;
    V3 position;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    if (!(stream >> stamp_s >> position.x() >> position.y() >> position.z() >>
          qx >> qy >> qz >> qw)) {
      if (message) {
        *message = "Malformed mapping trajectory row " + std::to_string(line_number) +
                   " in '" + path + "'.";
      }
      return false;
    }
    Quaternion orientation(qw, qx, qy, qz);
    if (!std::isfinite(stamp_s) || stamp_s < 0.0 || !position.allFinite() ||
        !orientation.coeffs().allFinite() || orientation.norm() < 1.0e-9) continue;
    orientation.normalize();
    trajectory.push_back({static_cast<uint64_t>(std::llround(stamp_s * 1.0e9)),
                          Transform(orientation, position)});
  }
  std::sort(trajectory.begin(), trajectory.end(),
            [](const ReferencePose& a, const ReferencePose& b) {
              return a.stamp < b.stamp;
            });
  trajectory.erase(
      std::unique(trajectory.begin(), trajectory.end(),
                  [](const ReferencePose& a, const ReferencePose& b) {
                    return a.stamp == b.stamp;
                  }),
      trajectory.end());
  if (trajectory.size() < 2) {
    if (message) *message = "Mapping trajectory contains fewer than two valid poses.";
    return false;
  }
  if (message) {
    *message = "Loaded " + std::to_string(trajectory.size()) +
               " mapping poses from '" + path + "'.";
  }
  return true;
}

std::optional<Transform> interpolateTrajectory(const std::vector<ReferencePose>& trajectory,
                                               uint64_t stamp, double max_gap_s) {
  const auto upper = std::lower_bound(
      trajectory.begin(), trajectory.end(), stamp,
      [](const ReferencePose& pose, uint64_t value) { return pose.stamp < value; });
  if (upper != trajectory.end() && upper->stamp == stamp) return upper->pose;
  if (upper == trajectory.begin() || upper == trajectory.end()) return std::nullopt;
  const ReferencePose& before = *(upper - 1);
  const ReferencePose& after = *upper;
  const double before_gap = static_cast<double>(stamp - before.stamp) * 1.0e-9;
  const double after_gap = static_cast<double>(after.stamp - stamp) * 1.0e-9;
  if (std::min(before_gap, after_gap) > max_gap_s ||
      static_cast<double>(after.stamp - before.stamp) * 1.0e-9 > 2.0 * max_gap_s) {
    return std::nullopt;
  }
  const double alpha = static_cast<double>(stamp - before.stamp) /
                       static_cast<double>(after.stamp - before.stamp);
  const V3 position = (1.0 - alpha) * before.pose.translation() +
                      alpha * after.pose.translation();
  const Quaternion first(before.pose.rotation());
  const Quaternion second(after.pose.rotation());
  return Transform(first.slerp(alpha, second).normalized(), position);
}

using ExtractionFramePtr = std::shared_ptr<ExtractionFrame>;

struct ExtractionTask {
  size_t index = 0;
  std::vector<ExtractionFramePtr> frames;
};

struct Detection {
  size_t keyframe = 0;
  uint64_t stamp = 0;
  V3 center = V3::Zero();
  V3 axis = V3::UnitZ();
  V3 axis_point = V3::Zero();
  double base_z = 0.0;
  double fusion_weight = 1.0;
  double radius = 0.0;
  double cluster_radius = 0.0;
  double fit_rms = 0.0;
  double inlier_ratio = 0.0;
  double angular_coverage = 0.0;
  double axis_tilt_deg = 0.0;
  double axial_min = 0.0;
  double axial_max = 0.0;
  size_t cylinder_segments = 1;
  double span_x = 0.0;
  double span_y = 0.0;
  size_t points = 0;
};

struct Landmark {
  size_t id = 0;
  V3 center = V3::Zero();
  V3 axis = V3::UnitZ();
  V3 axis_point = V3::Zero();
  double base_z = 0.0;
  double weight_sum = 0.0;
  double radius = 0.0;
  double fit_rms = 0.0;
  double axial_min = 0.0;
  double axial_max = 0.0;
  size_t observations = 0;
  size_t total_points = 0;
};

struct PriorPole {
  size_t id = 0;
  V3 point = V3::Zero();
  V3 axis = V3::UnitZ();
  double radius = 0.0;
  double quality = 1.0;
};

struct LocalPole {
  V3 point = V3::Zero();
  V3 axis = V3::UnitZ();
  double radius = 0.0;
  double weight = 1.0;
};

struct PoleMatch {
  size_t local = 0;
  size_t prior = 0;
  double cost = 0.0;
};

struct CylinderModel {
  V3 axis = V3::UnitZ();
  V3 axis_point = V3::Zero();
  double radius = 0.0;
  double rms = std::numeric_limits<double>::infinity();
  double inlier_ratio = 0.0;
  double angular_coverage = 0.0;
  double tilt_deg = 90.0;
  size_t point_count = 0;
  bool pca_valid = false;
  bool valid = false;
};

struct ExtractionResult {
  PoleCloud downsampled;
  PoleCloud candidates;
  PoleCloud refined;
  std::vector<Detection> detections;
};

struct RailSample {
  size_t segment = 0;
  size_t index = 0;
  uint64_t stamp = 0;
  V3 left = V3::Zero();
  V3 right = V3::Zero();
  V3 tangent = V3::UnitX();
  double station = 0.0;
  double gauge = 0.0;
  double quality = 0.0;
};

// Continuous piecewise-linear representation built from adjacent samples in
// the existing rail-vector map. Keeping this as a derived structure preserves
// compatibility with every rail_vector_map.txt already produced by the
// mapper, while localization no longer quantizes correspondences to samples.
struct RailSegment {
  RailSample start;
  RailSample finish;
  V3 center_start = V3::Zero();
  V3 center_finish = V3::Zero();
  V3 tangent = V3::UnitX();
};

std::vector<RailSegment> buildRailSegments(const std::vector<RailSample>& samples) {
  std::vector<RailSegment> segments;
  if (samples.size() < 2) return segments;
  segments.reserve(samples.size() - 1);
  for (size_t i = 0; i + 1 < samples.size(); ++i) {
    const RailSample& start = samples[i];
    const RailSample& finish = samples[i + 1];
    // Extraction restarts the sample index for every keyframe. Never bridge
    // two independent extraction windows, even if they spatially overlap.
    if (start.segment != finish.segment || finish.index != start.index + 1) continue;
    const V3 center_start = 0.5 * (start.left + start.right);
    const V3 center_finish = 0.5 * (finish.left + finish.right);
    const V3 delta = center_finish - center_start;
    if (!delta.allFinite() || delta.head<2>().squaredNorm() < 1.0e-8) continue;
    RailSegment segment;
    segment.start = start;
    segment.finish = finish;
    segment.center_start = center_start;
    segment.center_finish = center_finish;
    // The endpoints define the projection geometry, while the stored
    // trajectory tangents are a less noisy direction estimate than a 0.4 m
    // finite difference. Align their signs before averaging.
    V3 finish_tangent = finish.tangent;
    if (start.tangent.dot(finish_tangent) < 0.0) finish_tangent = -finish_tangent;
    const V3 averaged_tangent = start.tangent + finish_tangent;
    segment.tangent = averaged_tangent.norm() > 1.0e-6
        ? averaged_tangent.normalized()
        : delta.normalized();
    segments.push_back(std::move(segment));
  }
  return segments;
}

struct RailSegmentProjection {
  double ratio = 0.0;
  double planar_distance_sq = std::numeric_limits<double>::infinity();
  V3 center = V3::Zero();
  V3 left = V3::Zero();
  V3 right = V3::Zero();
};

RailSegmentProjection projectToRailSegment(const V3& point,
                                           const RailSegment& segment) {
  RailSegmentProjection projection;
  const Eigen::Vector2d delta =
      (segment.center_finish - segment.center_start).head<2>();
  const double length_sq = delta.squaredNorm();
  if (length_sq < 1.0e-8) return projection;
  projection.ratio = std::clamp(
      (point.head<2>() - segment.center_start.head<2>()).dot(delta) / length_sq,
      0.0, 1.0);
  projection.center = segment.center_start +
      projection.ratio * (segment.center_finish - segment.center_start);
  projection.left = segment.start.left +
      projection.ratio * (segment.finish.left - segment.start.left);
  projection.right = segment.start.right +
      projection.ratio * (segment.finish.right - segment.start.right);
  projection.planar_distance_sq =
      (point - projection.center).head<2>().squaredNorm();
  return projection;
}

struct RailCellStat {
  int count = 0;
  double top_z = -std::numeric_limits<double>::infinity();
};

struct RailProjection {
  Rotation body_rotation = Rotation::Identity();
  V3 position = V3::Zero();
  V3 tangent = V3::UnitX();
  double station = 0.0;
};

uint64_t railCellKey(int station, int lateral) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(station)) << 32U) |
         static_cast<uint32_t>(lateral);
}

RailProjection projectToRailTrajectory(
    const V3& point, const std::vector<ExtractionFramePtr>& frames,
    const std::vector<double>& cumulative) {
  RailProjection best;
  double best_distance_sq = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < frames.size(); ++i) {
    const V3 start = frames[i]->pose.translation();
    const V3 finish = frames[i + 1]->pose.translation();
    const Eigen::Vector2d segment = (finish - start).head<2>();
    const double length_sq = segment.squaredNorm();
    if (length_sq < 1.0e-8) continue;
    const double ratio = std::clamp(
        (point.head<2>() - start.head<2>()).dot(segment) / length_sq, 0.0, 1.0);
    const V3 position = start + ratio * (finish - start);
    const double distance_sq = (point.head<2>() - position.head<2>()).squaredNorm();
    if (distance_sq >= best_distance_sq) continue;
    best_distance_sq = distance_sq;
    best.position = position;
    best.tangent = (finish - start).normalized();
    best.station = cumulative[i] + ratio * segment.head<2>().norm();
  }
  return best;
}

// Invert trajectory arc length directly. Vehicle orientation may oppose the
// direction of motion, so heading-based queries cannot locate a station.
RailProjection railTrajectoryAtStation(
    double station, const std::vector<ExtractionFramePtr>& frames,
    const std::vector<double>& cumulative) {
  RailProjection result;
  if (frames.empty()) return result;
  result.position = frames.front()->pose.translation();
  result.body_rotation = frames.front()->pose.rotation();
  if (frames.size() < 2) return result;
  station = std::clamp(station, 0.0, cumulative.back());
  for (size_t i = 0; i + 1 < frames.size(); ++i) {
    const double length = cumulative[i + 1] - cumulative[i];
    if (length <= 1e-8) continue;
    if (station > cumulative[i + 1] && i + 2 < frames.size()) continue;
    const V3 start = frames[i]->pose.translation();
    const V3 delta = frames[i + 1]->pose.translation() - start;
    const double ratio = std::clamp((station - cumulative[i]) / length, 0.0, 1.0);
    result.position = start + ratio * delta;
    result.body_rotation = Quaternion(frames[i]->pose.rotation()).slerp(
        ratio, Quaternion(frames[i + 1]->pose.rotation())).normalized().toRotationMatrix();
    result.tangent = delta.normalized();
    result.station = station;
    return result;
  }
  return result;
}

std::vector<RailSample> extractRailVector(
    const ExtractionTask& task, const PoleCloud& cloud,
    const PoleExtractionConfig& config) {
  std::vector<RailSample> output;
  if (!config.rail_enable || task.frames.size() < 2 || cloud.empty()) return output;

  std::vector<double> cumulative(task.frames.size(), 0.0);
  for (size_t i = 1; i < task.frames.size(); ++i) {
    cumulative[i] = cumulative[i - 1] +
        (task.frames[i]->pose.translation() -
         task.frames[i - 1]->pose.translation()).head<2>().norm();
  }
  if (cumulative.back() < config.rail_station_resolution) return output;

  std::unordered_map<uint64_t, RailCellStat> cells;
  cells.reserve(cloud.size() / 8 + 1);
  const int station_count =
      static_cast<int>(std::ceil(cumulative.back() / config.rail_station_resolution));
  const int lateral_min_bin =
      static_cast<int>(std::floor(config.rail_lateral_min /
                                  config.rail_lateral_resolution));
  const int lateral_max_bin =
      static_cast<int>(std::ceil(config.rail_lateral_max /
                                 config.rail_lateral_resolution));
  for (const PolePoint& point : cloud) {
    const V3 world(point.x, point.y, point.z);
    const RailProjection projection =
        projectToRailTrajectory(world, task.frames, cumulative);
    const Eigen::Vector2d tangent = projection.tangent.head<2>().normalized();
    // Endpoint-clamped projections must not collapse distant points outside
    // this keyframe's trajectory into its first/last cross section.
    const double along_offset =
        (world.head<2>() - projection.position.head<2>()).dot(tangent);
    if (std::abs(along_offset) > 0.5 * config.rail_station_resolution) continue;
    const Eigen::Vector2d left(-tangent.y(), tangent.x());
    const double lateral = (world.head<2>() - projection.position.head<2>()).dot(left);
    const double relative_z = world.z() - projection.position.z();
    if (lateral < config.rail_lateral_min || lateral > config.rail_lateral_max ||
        relative_z < config.rail_relative_z_min ||
        relative_z > config.rail_relative_z_max) {
      continue;
    }
    const int station_bin = static_cast<int>(std::floor(
        projection.station / config.rail_station_resolution));
    const int lateral_bin = static_cast<int>(std::floor(
        lateral / config.rail_lateral_resolution));
    RailCellStat& cell = cells[railCellKey(station_bin, lateral_bin)];
    ++cell.count;
    cell.top_z = std::max(cell.top_z, relative_z);
  }

  struct Candidate {
    int lateral_bin = 0;
    double lateral = 0.0;
    double top_z = 0.0;
    double prominence = 0.0;
  };
  bool have_previous = false;
  double previous_left = 0.0;
  double previous_right = 0.0;
  double previous_left_z = 0.0;
  double previous_right_z = 0.0;
  int missing = 0;
  for (int station_bin = 0; station_bin <= station_count; ++station_bin) {
    const double station =
        (static_cast<double>(station_bin) + 0.5) * config.rail_station_resolution;
    const RailProjection projection =
        railTrajectoryAtStation(station, task.frames, cumulative);
    Eigen::Vector2d tangent_xy = projection.tangent.head<2>();
    if (tangent_xy.norm() < 1.0e-6) continue;
    tangent_xy.normalize();
    const V3 left_axis(-tangent_xy.y(), tangent_xy.x(), 0.0);
    struct ExpectedRail { double lateral; double z; };
    std::array<ExpectedRail, 2> expected{};
    if (config.rail_mount_prior_enable) {
      for (size_t side = 0; side < 2; ++side) {
        const double y = config.rail_mount_center_y +
            (side == 0 ? -0.5 : 0.5) * config.rail_nominal_gauge;
        const V3 offset = projection.body_rotation * V3(0.0, y, -config.rail_mount_height);
        expected[side] = {offset.dot(left_axis), offset.z()};
      }
      // Traversal may oppose the body X axis. Sort in the extraction frame
      // rather than treating body left/right as travel left/right.
      if (expected[0].lateral > expected[1].lateral) std::swap(expected[0], expected[1]);
    }
    auto inside_mount_prior = [&](double lateral, double z, const ExpectedRail& prior) {
      return std::abs(lateral - prior.lateral) <= config.rail_mount_lateral_tolerance &&
             std::abs(z - prior.z) <= config.rail_mount_height_tolerance;
    };
    std::vector<Candidate> candidates;
    for (int lateral_bin = lateral_min_bin + 3;
         lateral_bin <= lateral_max_bin - 3; ++lateral_bin) {
      const auto found = cells.find(railCellKey(station_bin, lateral_bin));
      if (found == cells.end() || found->second.count < config.rail_min_cell_points) {
        continue;
      }
      double neighbour_sum = 0.0;
      int neighbours = 0;
      double neighbour_max = -std::numeric_limits<double>::infinity();
      for (int offset : {-3, -2, 2, 3}) {
        const auto neighbour =
            cells.find(railCellKey(station_bin, lateral_bin + offset));
        if (neighbour == cells.end() ||
            neighbour->second.count < config.rail_min_cell_points / 2) continue;
        neighbour_sum += neighbour->second.top_z;
        neighbour_max = std::max(neighbour_max, neighbour->second.top_z);
        ++neighbours;
      }
      if (neighbours < 2) continue;
      const double background = neighbour_sum / static_cast<double>(neighbours);
      const double prominence = found->second.top_z - background;
      if (prominence < config.rail_min_prominence ||
          found->second.top_z + 0.01 < neighbour_max) continue;
      const double lateral =
          (static_cast<double>(lateral_bin) + 0.5) * config.rail_lateral_resolution;
      if (config.rail_mount_prior_enable &&
          !inside_mount_prior(lateral, found->second.top_z, expected[0]) &&
          !inside_mount_prior(lateral, found->second.top_z, expected[1])) continue;
      candidates.push_back({lateral_bin, lateral, found->second.top_z, prominence});
    }

    const Candidate* best_left = nullptr;
    const Candidate* best_right = nullptr;
    double best_cost = std::numeric_limits<double>::infinity();
    for (const Candidate& left : candidates) {
      for (const Candidate& right : candidates) {
        const double gauge = right.lateral - left.lateral;
        if (gauge < config.rail_min_gauge || gauge > config.rail_max_gauge) continue;
        double cost = 2.0 * std::abs(gauge - config.rail_nominal_gauge) -
                      left.prominence - right.prominence;
        if (config.rail_mount_prior_enable) {
          if (!inside_mount_prior(left.lateral, left.top_z, expected[0]) ||
              !inside_mount_prior(right.lateral, right.top_z, expected[1])) continue;
          const double left_y = (left.lateral - expected[0].lateral) /
                                config.rail_mount_lateral_tolerance;
          const double right_y = (right.lateral - expected[1].lateral) /
                                 config.rail_mount_lateral_tolerance;
          const double left_z = (left.top_z - expected[0].z) /
                                config.rail_mount_height_tolerance;
          const double right_z = (right.top_z - expected[1].z) /
                                 config.rail_mount_height_tolerance;
          cost += config.rail_mount_prior_weight *
                  (left_y * left_y + right_y * right_y + left_z * left_z + right_z * right_z);
        }
        if (have_previous) {
          const double left_jump = std::abs(left.lateral - previous_left);
          const double right_jump = std::abs(right.lateral - previous_right);
          if (left_jump > config.rail_max_lateral_jump ||
              right_jump > config.rail_max_lateral_jump ||
              std::abs(left.top_z - previous_left_z) >
                  config.rail_max_vertical_jump ||
              std::abs(right.top_z - previous_right_z) >
                  config.rail_max_vertical_jump) continue;
          cost += 3.0 * (left_jump + right_jump);
        }
        if (std::abs(left.top_z - right.top_z) >
            config.rail_max_cross_level) continue;
        if (cost < best_cost) {
          best_cost = cost;
          best_left = &left;
          best_right = &right;
        }
      }
    }
    if (best_left == nullptr || best_right == nullptr) {
      if (++missing > 3) have_previous = false;
      continue;
    }
    missing = 0;
    have_previous = true;
    previous_left = best_left->lateral;
    previous_right = best_right->lateral;
    previous_left_z = best_left->top_z;
    previous_right_z = best_right->top_z;

    RailSample sample;
    sample.segment = task.index;
    sample.index = output.size();
    sample.stamp = task.frames.back()->stamp;
    sample.station = station;
    sample.tangent = projection.tangent.normalized();
    sample.left = projection.position + best_left->lateral * left_axis;
    sample.right = projection.position + best_right->lateral * left_axis;
    sample.left.z() = projection.position.z() + best_left->top_z;
    sample.right.z() = projection.position.z() + best_right->top_z;
    sample.gauge = (sample.right - sample.left).head<2>().norm();
    sample.quality = std::min(best_left->prominence, best_right->prominence);
    output.push_back(sample);
  }
  if (output.size() < config.rail_min_segment_samples) output.clear();
  return output;
}

// Select measured points from the full-resolution scans around detected rails.
// Segment interpolation only defines the selection corridor; output coordinates
// always come from the input scans, without voxel filtering or synthetic points.
PoleCloud extractRailPoints(const ExtractionTask& task,
                            const std::vector<RailSample>& rails,
                            const PoleExtractionConfig& config) {
  struct Corridor { V3 start; V3 finish; };
  std::vector<Corridor> corridors;
  for (size_t i = 0; i < rails.size(); ++i) {
    const RailSample& sample = rails[i];
    V3 tangent(sample.tangent.x(), sample.tangent.y(), 0.0);
    if (tangent.norm() < 1e-6) continue;
    tangent.normalize();
    for (bool left : {true, false}) {
      const V3& center = left ? sample.left : sample.right;
      const V3 half = 0.5 * config.rail_station_resolution * tangent;
      corridors.push_back({center - half, center + half});
      if (i == 0) continue;
      const RailSample& previous = rails[i - 1];
      const double gap = sample.station - previous.station;
      if (previous.segment != sample.segment || gap <= 0.0 ||
          gap > 3.0 * config.rail_station_resolution) continue;
      const V3& start = left ? previous.left : previous.right;
      if ((center - start).head<2>().norm() > gap + config.rail_max_lateral_jump ||
          std::abs(center.z() - start.z()) > 3.0 * config.rail_max_vertical_jump) continue;
      corridors.push_back({start, center});
    }
  }

  // Spatial buckets avoid comparing every raw point to every rail segment.
  std::unordered_map<Cell, std::vector<size_t>, CellHash> buckets;
  const double width = config.rail_cloud_half_width;
  for (size_t i = 0; i < corridors.size(); ++i) {
    const auto& c = corridors[i];
    const int xmin = static_cast<int>(std::floor(std::min(c.start.x(), c.finish.x()) - width));
    const int xmax = static_cast<int>(std::floor(std::max(c.start.x(), c.finish.x()) + width));
    const int ymin = static_cast<int>(std::floor(std::min(c.start.y(), c.finish.y()) - width));
    const int ymax = static_cast<int>(std::floor(std::max(c.start.y(), c.finish.y()) + width));
    for (int x = xmin; x <= xmax; ++x) {
      for (int y = ymin; y <= ymax; ++y) buckets[{x, y}].push_back(i);
    }
  }
  PoleCloud output;
  if (corridors.empty()) return output;
  for (const auto& frame : task.frames) {
    for (size_t i = 0; i < frame->cloud.size(); ++i) {
      const V3 local = frame->cloud[i];
      if (!local.allFinite()) continue;
      const V3 world = frame->pose * local;
      const Cell cell{static_cast<int>(std::floor(world.x())),
                      static_cast<int>(std::floor(world.y()))};
      const auto found = buckets.find(cell);
      if (found == buckets.end()) continue;
      for (size_t index : found->second) {
        const Corridor& c = corridors[index];
        const V3 delta = c.finish - c.start;
        const double length_sq = delta.head<2>().squaredNorm();
        const double ratio = length_sq > 1e-12 ? std::clamp(
            (world - c.start).head<2>().dot(delta.head<2>()) / length_sq, 0.0, 1.0) : 0.0;
        const V3 projected = c.start + ratio * delta;
        const double dz = world.z() - projected.z();
        if ((world - projected).head<2>().squaredNorm() <= width * width &&
            dz >= -config.rail_cloud_depth && dz <= config.rail_cloud_above) {
          output.push_back({static_cast<float>(world.x()), static_cast<float>(world.y()),
                            static_cast<float>(world.z())});
          break;
        }
      }
    }
  }
  return output;
}

bool loadRailMap(const std::string& path, std::vector<RailSample>& rails,
                 std::string* message) {
  std::ifstream input(path);
  if (!input) {
    if (message) *message = "Cannot open rail vector map '" + path + "'.";
    return false;
  }
  std::string line;
  std::getline(input, line);
  rails.clear();
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') continue;
    std::istringstream stream(line);
    RailSample sample;
    double stamp_s = 0.0;
    if (!(stream >> sample.segment >> sample.index >> stamp_s >> sample.station >>
          sample.left.x() >> sample.left.y() >> sample.left.z() >>
          sample.right.x() >> sample.right.y() >> sample.right.z() >>
          sample.tangent.x() >> sample.tangent.y() >> sample.tangent.z() >>
          sample.gauge >> sample.quality)) continue;
    sample.stamp = static_cast<uint64_t>(std::llround(stamp_s * 1.0e9));
    if (sample.left.allFinite() && sample.right.allFinite() &&
        sample.tangent.allFinite()) rails.push_back(sample);
  }
  if (rails.empty()) {
    if (message) *message = "Rail vector map has no valid samples: '" + path + "'.";
    return false;
  }
  if (message) *message = "Loaded " + std::to_string(rails.size()) +
                          " rail-vector samples from '" + path + "'.";
  return true;
}

Cell toCell(const PolePoint& point, double resolution) {
  return {static_cast<int>(std::floor(static_cast<double>(point.x) / resolution)),
          static_cast<int>(std::floor(static_cast<double>(point.y) / resolution))};
}

int longestContinuousRun(const std::unordered_set<int>& layer_set) {
  if (layer_set.empty()) return 0;
  std::vector<int> layers(layer_set.begin(), layer_set.end());
  std::sort(layers.begin(), layers.end());
  int longest = 1;
  int current = 1;
  for (size_t i = 1; i < layers.size(); ++i) {
    if (layers[i] == layers[i - 1] + 1) {
      longest = std::max(longest, ++current);
    } else {
      current = 1;
    }
  }
  return longest;
}

std::unordered_set<Cell, CellHash> closeMask(
    const std::unordered_set<Cell, CellHash>& candidates) {
  std::unordered_set<Cell, CellHash> dilated;
  dilated.reserve(candidates.size() * 9);
  for (const Cell& cell : candidates) {
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        dilated.insert({cell.x + dx, cell.y + dy});
      }
    }
  }

  std::unordered_set<Cell, CellHash> closed;
  closed.reserve(dilated.size());
  for (const Cell& cell : dilated) {
    bool keep = true;
    for (int dx = -1; dx <= 1 && keep; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dilated.find({cell.x + dx, cell.y + dy}) == dilated.end()) {
          keep = false;
          break;
        }
      }
    }
    if (keep) closed.insert(cell);
  }
  return closed;
}

std::unordered_set<Cell, CellHash> smallComponents(
    const std::unordered_set<Cell, CellHash>& mask, int max_component_cells) {
  std::unordered_set<Cell, CellHash> visited;
  std::unordered_set<Cell, CellHash> valid;
  visited.reserve(mask.size());
  valid.reserve(mask.size());

  for (const Cell& seed : mask) {
    if (visited.find(seed) != visited.end()) continue;
    std::vector<Cell> component;
    std::deque<Cell> frontier;
    frontier.push_back(seed);
    visited.insert(seed);
    while (!frontier.empty()) {
      const Cell cell = frontier.front();
      frontier.pop_front();
      component.push_back(cell);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          const Cell neighbor{cell.x + dx, cell.y + dy};
          if (mask.find(neighbor) != mask.end() && visited.insert(neighbor).second) {
            frontier.push_back(neighbor);
          }
        }
      }
    }
    if (component.size() <= static_cast<size_t>(max_component_cells)) {
      valid.insert(component.begin(), component.end());
    }
  }
  return valid;
}

Pointcloud toBievrCloud(const PoleCloud& cloud) {
  Pointcloud result;
  result.resize(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    result.data().col(static_cast<Eigen::Index>(i)) =
        V3(cloud[i].x, cloud[i].y, cloud[i].z);
  }
  return result;
}

Pointcloud centersToCloud(const std::vector<Detection>& detections) {
  Pointcloud cloud;
  cloud.resize(detections.size());
  for (size_t i = 0; i < detections.size(); ++i) {
    cloud.data().col(static_cast<Eigen::Index>(i)) = detections[i].center;
  }
  return cloud;
}

Pointcloud landmarksToCloud(const std::vector<Landmark>& landmarks) {
  Pointcloud cloud;
  cloud.resize(landmarks.size());
  for (size_t i = 0; i < landmarks.size(); ++i) {
    cloud.data().col(static_cast<Eigen::Index>(i)) = landmarks[i].center;
  }
  return cloud;
}

Pointcloud priorPolesToCloud(const std::vector<PriorPole>& poles) {
  Pointcloud cloud;
  cloud.resize(poles.size());
  for (size_t i = 0; i < poles.size(); ++i) {
    cloud.data().col(static_cast<Eigen::Index>(i)) = poles[i].point;
  }
  return cloud;
}

Pointcloud railsToCloud(const std::vector<RailSample>& rails) {
  Pointcloud cloud;
  cloud.resize(2 * rails.size());
  for (size_t i = 0; i < rails.size(); ++i) {
    cloud.data().col(static_cast<Eigen::Index>(2 * i)) = rails[i].left;
    cloud.data().col(static_cast<Eigen::Index>(2 * i + 1)) = rails[i].right;
  }
  return cloud;
}

bool writeXYZPcd(const std::filesystem::path& path, const PoleCloud& cloud) {
  std::ofstream file(path, std::ios::binary);
  if (!file) return false;
  file << "# .PCD v0.7 - Point Cloud Data file format\n"
       << "VERSION 0.7\n"
       << "FIELDS x y z\n"
       << "SIZE 4 4 4\n"
       << "TYPE F F F\n"
       << "COUNT 1 1 1\n"
       << "WIDTH " << cloud.size() << "\n"
       << "HEIGHT 1\n"
       << "VIEWPOINT 0 0 0 1 0 0 0\n"
       << "POINTS " << cloud.size() << "\n"
       << "DATA binary\n";
  for (const PolePoint& point : cloud) {
    file.write(reinterpret_cast<const char*>(&point.x), sizeof(float));
    file.write(reinterpret_cast<const char*>(&point.y), sizeof(float));
    file.write(reinterpret_cast<const char*>(&point.z), sizeof(float));
  }
  return static_cast<bool>(file);
}

bool writeLandmarkPcd(const std::filesystem::path& path,
                      const std::vector<Landmark>& landmarks) {
  std::ofstream file(path, std::ios::binary);
  if (!file) return false;
  file << "# .PCD v0.7 - Point Cloud Data file format\n"
       << "VERSION 0.7\n"
       << "FIELDS x y z intensity\n"
       << "SIZE 4 4 4 4\n"
       << "TYPE F F F F\n"
       << "COUNT 1 1 1 1\n"
       << "WIDTH " << landmarks.size() << "\n"
       << "HEIGHT 1\n"
       << "VIEWPOINT 0 0 0 1 0 0 0\n"
       << "POINTS " << landmarks.size() << "\n"
       << "DATA binary\n";
  for (const Landmark& landmark : landmarks) {
    const float values[4] = {static_cast<float>(landmark.center.x()),
                             static_cast<float>(landmark.center.y()),
                             static_cast<float>(landmark.center.z()),
                             static_cast<float>(landmark.observations)};
    file.write(reinterpret_cast<const char*>(values), sizeof(values));
  }
  return static_cast<bool>(file);
}

PoleCloud voxelDownsample(PoleCloud input, double resolution) {
  if (resolution <= 0.0 || input.empty()) return input;
  std::unordered_map<Voxel, PointSum, VoxelHash> voxels;
  voxels.reserve(input.size() / 4 + 1);
  for (const PolePoint& point : input) {
    const Voxel voxel{
        static_cast<int>(std::floor(static_cast<double>(point.x) / resolution)),
        static_cast<int>(std::floor(static_cast<double>(point.y) / resolution)),
        static_cast<int>(std::floor(static_cast<double>(point.z) / resolution))};
    PointSum& sum = voxels[voxel];
    sum.x += point.x;
    sum.y += point.y;
    sum.z += point.z;
    ++sum.count;
  }
  PoleCloud output;
  output.reserve(voxels.size());
  for (const auto& [voxel, sum] : voxels) {
    const double count = static_cast<double>(sum.count);
    output.push_back({static_cast<float>(sum.x / count), static_cast<float>(sum.y / count),
                      static_cast<float>(sum.z / count)});
  }
  return output;
}

std::vector<Eigen::Vector2d> trajectoryTangents(
    const std::vector<ExtractionFramePtr>& frames) {
  std::vector<Eigen::Vector2d> tangents(frames.size(), Eigen::Vector2d::UnitX());
  for (size_t i = 0; i < frames.size(); ++i) {
    Eigen::Vector2d tangent;
    if (frames.size() > 1) {
      const size_t before = i == 0 ? 0 : i - 1;
      const size_t after = i + 1 < frames.size() ? i + 1 : i;
      tangent =
          (frames[after]->pose.translation() - frames[before]->pose.translation()).head<2>();
    }
    if (frames.size() == 1 || tangent.norm() < 0.05) {
      tangent = frames[i]->pose.rotation().block<2, 1>(0, 0);
    }
    if (tangent.norm() < 1e-6) tangent = Eigen::Vector2d::UnitX();
    tangents[i] = tangent.normalized();
  }
  return tangents;
}

struct TrackProjection {
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d tangent = Eigen::Vector2d::UnitX();
  double z = 0.0;
};

TrackProjection projectToTrajectory(const V3& point,
                                    const std::vector<ExtractionFramePtr>& frames,
                                    const std::vector<Eigen::Vector2d>& tangents) {
  TrackProjection best;
  double best_distance_sq = std::numeric_limits<double>::infinity();
  if (frames.size() == 1) {
    best.position = frames.front()->pose.translation().head<2>();
    best.tangent = tangents.front();
    best.z = frames.front()->pose.translation().z();
    return best;
  }
  for (size_t i = 0; i + 1 < frames.size(); ++i) {
    const V3 start = frames[i]->pose.translation();
    const V3 finish = frames[i + 1]->pose.translation();
    const Eigen::Vector2d segment = (finish - start).head<2>();
    const double length_sq = segment.squaredNorm();
    const double ratio = length_sq > 1e-12
                             ? std::clamp((point.head<2>() - start.head<2>()).dot(segment) /
                                              length_sq,
                                          0.0, 1.0)
                             : 0.0;
    const Eigen::Vector2d position = start.head<2>() + ratio * segment;
    const double distance_sq = (point.head<2>() - position).squaredNorm();
    if (distance_sq >= best_distance_sq) continue;
    best_distance_sq = distance_sq;
    best.position = position;
    best.tangent = length_sq >= 0.05 * 0.05 ? segment.normalized() : tangents[i];
    best.z = start.z() + ratio * (finish.z() - start.z());
  }
  return best;
}

PoleCloud accumulateFrames(const std::vector<ExtractionFramePtr>& frames) {
  PoleCloud cloud;
  size_t reserve = 0;
  for (const auto& frame : frames) reserve += frame->cloud.size();
  cloud.reserve(reserve);

  for (size_t frame_index = 0; frame_index < frames.size(); ++frame_index) {
    const ExtractionFrame& frame = *frames[frame_index];
    for (size_t i = 0; i < frame.cloud.size(); ++i) {
      const V3 local = frame.cloud[i];
      if (!local.allFinite()) continue;
      const V3 world = frame.pose * local;
      cloud.push_back({static_cast<float>(world.x()), static_cast<float>(world.y()),
                       static_cast<float>(world.z())});
    }
  }
  return cloud;
}

PoleCloud filterByTrajectoryRoi(PoleCloud input,
                                const std::vector<ExtractionFramePtr>& frames,
                                const PoleExtractionConfig& config,
                                const std::vector<PriorPole>& guided_poles) {
  if (!config.use_roi || input.empty()) return input;
  const auto tangents = trajectoryTangents(frames);
  PoleCloud output;
  output.reserve(input.size());
  for (const PolePoint& point : input) {
    const V3 world(point.x, point.y, point.z);
    const TrackProjection projection = projectToTrajectory(world, frames, tangents);
    const Eigen::Vector2d left(-projection.tangent.y(), projection.tangent.x());
    const double rel_y = (world.head<2>() - projection.position).dot(left);
    const double rel_z = world.z() - projection.z;
    if (rel_z < config.roi_z_min || rel_z > config.roi_z_max) continue;
    bool keep = rel_y >= config.roi_y_min && rel_y <= config.roi_y_max;
    if (!keep) {
      for (const PriorPole& pole : guided_poles) {
        const V3 on_axis = pole.point +
            ((world.z() - pole.point.z()) / pole.axis.z()) * pole.axis;
        if ((world - on_axis).head<2>().squaredNorm() <=
            config.localization_guided_roi_radius * config.localization_guided_roi_radius) {
          keep = true;
          break;
        }
      }
    }
    if (keep) output.push_back(point);
  }
  return output;
}

double nearestTrajectoryZ(double x, double y,
                          const std::vector<ExtractionFramePtr>& frames) {
  const auto tangents = trajectoryTangents(frames);
  return projectToTrajectory(V3(x, y, 0.0), frames, tangents).z;
}

V3 pointOnAxisAtZ(const V3& axis_point, const V3& axis, double z) {
  if (std::abs(axis.z()) < 1e-6) return axis_point;
  return axis_point + (z - axis_point.z()) / axis.z() * axis;
}

bool loadPriorPoles(const std::string& path, std::vector<PriorPole>& poles,
                    std::string* message) {
  std::ifstream input(path);
  if (!input) {
    if (message) *message = "Cannot open prior pole map '" + path + "'.";
    return false;
  }
  std::string line;
  if (!std::getline(input, line)) {
    if (message) *message = "Prior pole map is empty: '" + path + "'.";
    return false;
  }
  std::istringstream header_stream(line);
  std::unordered_map<std::string, size_t> columns;
  std::string name;
  size_t column = 0;
  while (header_stream >> name) columns.emplace(name, column++);
  for (const char* required : {"X", "Y", "Z"}) {
    if (columns.find(required) == columns.end()) {
      if (message) *message = "Prior pole map misses column '" + std::string(required) + "'.";
      return false;
    }
  }
  auto value = [&columns](const std::vector<double>& row, const char* key, double fallback) {
    const auto it = columns.find(key);
    return it == columns.end() || it->second >= row.size() ? fallback : row[it->second];
  };
  poles.clear();
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    std::istringstream row_stream(line);
    std::vector<double> row;
    double number = 0.0;
    while (row_stream >> number) row.push_back(number);
    if (row.size() < columns.size()) continue;
    PriorPole pole;
    pole.id = static_cast<size_t>(std::max(0.0, value(row, "ID", poles.size())));
    pole.point = V3(value(row, "X", 0.0), value(row, "Y", 0.0), value(row, "Z", 0.0));
    pole.axis = V3(value(row, "AxisX", 0.0), value(row, "AxisY", 0.0),
                   value(row, "AxisZ", 1.0));
    if (pole.axis.norm() < 1e-6) pole.axis = V3::UnitZ();
    pole.axis.normalize();
    if (pole.axis.z() < 0.0) pole.axis = -pole.axis;
    pole.radius = value(row, "Radius", 0.25);
    const double observations = std::max(1.0, value(row, "Observations", 1.0));
    const double rms = std::max(0.05, value(row, "FitRMS", 0.10));
    pole.quality = observations / (rms * rms);
    if (pole.point.allFinite() && pole.axis.allFinite() && std::isfinite(pole.radius)) {
      poles.push_back(pole);
    }
  }
  if (poles.size() < 3) {
    if (message) *message = "Prior pole map must contain at least three valid landmarks.";
    return false;
  }
  if (message) {
    *message = "Loaded " + std::to_string(poles.size()) + " prior pole landmarks from '" +
               path + "'.";
  }
  return true;
}

std::vector<PoleMatch> hungarianAssignment(const std::vector<std::vector<double>>& input,
                                           double invalid_cost) {
  if (input.empty() || input.front().empty()) return {};
  const size_t original_rows = input.size();
  const size_t original_columns = input.front().size();
  const bool transposed = original_rows > original_columns;
  const size_t rows = transposed ? original_columns : original_rows;
  const size_t columns = transposed ? original_rows : original_columns;
  auto cost = [&input, transposed](size_t row, size_t column) {
    return transposed ? input[column][row] : input[row][column];
  };

  std::vector<double> row_potential(rows + 1, 0.0);
  std::vector<double> column_potential(columns + 1, 0.0);
  std::vector<size_t> matched_row(columns + 1, 0);
  std::vector<size_t> predecessor(columns + 1, 0);
  for (size_t row = 1; row <= rows; ++row) {
    matched_row[0] = row;
    size_t column0 = 0;
    std::vector<double> minimum(columns + 1, std::numeric_limits<double>::infinity());
    std::vector<uint8_t> used(columns + 1, 0);
    do {
      used[column0] = 1;
      const size_t row0 = matched_row[column0];
      double delta = std::numeric_limits<double>::infinity();
      size_t column1 = 0;
      for (size_t column = 1; column <= columns; ++column) {
        if (used[column]) continue;
        const double reduced = cost(row0 - 1, column - 1) - row_potential[row0] -
                               column_potential[column];
        if (reduced < minimum[column]) {
          minimum[column] = reduced;
          predecessor[column] = column0;
        }
        if (minimum[column] < delta) {
          delta = minimum[column];
          column1 = column;
        }
      }
      for (size_t column = 0; column <= columns; ++column) {
        if (used[column]) {
          row_potential[matched_row[column]] += delta;
          column_potential[column] -= delta;
        } else {
          minimum[column] -= delta;
        }
      }
      column0 = column1;
    } while (matched_row[column0] != 0);
    do {
      const size_t column1 = predecessor[column0];
      matched_row[column0] = matched_row[column1];
      column0 = column1;
    } while (column0 != 0);
  }

  std::vector<PoleMatch> matches;
  matches.reserve(rows);
  for (size_t column = 1; column <= columns; ++column) {
    if (matched_row[column] == 0) continue;
    size_t local = matched_row[column] - 1;
    size_t prior = column - 1;
    if (transposed) std::swap(local, prior);
    const double original_cost = input[local][prior];
    if (original_cost < invalid_cost) matches.push_back({local, prior, original_cost});
  }
  return matches;
}

double yawOf(const Rotation& rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

double wrappedAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

Rotation yawRotation(double yaw) {
  return Eigen::AngleAxisd(yaw, V3::UnitZ()).toRotationMatrix();
}

std::vector<std::vector<size_t>> euclideanClusters(const PoleCloud& cloud, double tolerance) {
  std::unordered_map<Voxel, std::vector<size_t>, VoxelHash> buckets;
  buckets.reserve(cloud.size() / 4 + 1);
  auto voxelOf = [tolerance](const PolePoint& point) {
    return Voxel{static_cast<int>(std::floor(static_cast<double>(point.x) / tolerance)),
                 static_cast<int>(std::floor(static_cast<double>(point.y) / tolerance)),
                 static_cast<int>(std::floor(static_cast<double>(point.z) / tolerance))};
  };
  for (size_t i = 0; i < cloud.size(); ++i) buckets[voxelOf(cloud[i])].push_back(i);

  const double tolerance_sq = tolerance * tolerance;
  std::vector<uint8_t> visited(cloud.size(), 0);
  std::vector<std::vector<size_t>> clusters;
  for (size_t seed = 0; seed < cloud.size(); ++seed) {
    if (visited[seed]) continue;
    visited[seed] = 1;
    std::vector<size_t> cluster;
    std::deque<size_t> frontier;
    frontier.push_back(seed);
    while (!frontier.empty()) {
      const size_t current = frontier.front();
      frontier.pop_front();
      cluster.push_back(current);
      const PolePoint& point = cloud[current];
      const Voxel voxel = voxelOf(point);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            const auto bucket = buckets.find({voxel.x + dx, voxel.y + dy, voxel.z + dz});
            if (bucket == buckets.end()) continue;
            for (size_t neighbor : bucket->second) {
              if (visited[neighbor]) continue;
              const PolePoint& other = cloud[neighbor];
              const double px = static_cast<double>(point.x) - other.x;
              const double py = static_cast<double>(point.y) - other.y;
              const double pz = static_cast<double>(point.z) - other.z;
              if (px * px + py * py + pz * pz <= tolerance_sq) {
                visited[neighbor] = 1;
                frontier.push_back(neighbor);
              }
            }
          }
        }
      }
    }
    clusters.push_back(std::move(cluster));
  }
  return clusters;
}

struct CirclePoint {
  double x = 0.0;
  double y = 0.0;
};

bool circleFromThree(const CirclePoint& a, const CirclePoint& b, const CirclePoint& c,
                     double& center_x, double& center_y, double& radius) {
  const double denominator =
      2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
  if (std::abs(denominator) < 1e-10) return false;
  const double a2 = a.x * a.x + a.y * a.y;
  const double b2 = b.x * b.x + b.y * b.y;
  const double c2 = c.x * c.x + c.y * c.y;
  center_x = (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) /
             denominator;
  center_y = (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) /
             denominator;
  radius = std::hypot(a.x - center_x, a.y - center_y);
  return std::isfinite(radius) && radius > 1e-6;
}

bool leastSquaresCircle(const std::vector<CirclePoint>& points,
                        const std::vector<size_t>& inliers, double& center_x,
                        double& center_y, double& radius) {
  if (inliers.size() < 3) return false;
  Eigen::MatrixXd system(static_cast<Eigen::Index>(inliers.size()), 3);
  Eigen::VectorXd rhs(static_cast<Eigen::Index>(inliers.size()));
  for (size_t row = 0; row < inliers.size(); ++row) {
    const CirclePoint& point = points[inliers[row]];
    system.row(static_cast<Eigen::Index>(row)) << 2.0 * point.x, 2.0 * point.y, 1.0;
    rhs(static_cast<Eigen::Index>(row)) = point.x * point.x + point.y * point.y;
  }
  const Eigen::Vector3d solution = system.colPivHouseholderQr().solve(rhs);
  center_x = solution.x();
  center_y = solution.y();
  const double radius_sq = solution.z() + center_x * center_x + center_y * center_y;
  if (!solution.allFinite() || radius_sq <= 1e-12) return false;
  radius = std::sqrt(radius_sq);
  return std::isfinite(radius);
}

std::vector<size_t> circleInliers(const std::vector<CirclePoint>& points, double center_x,
                                  double center_y, double radius, double threshold,
                                  double* residual_sum = nullptr) {
  std::vector<size_t> inliers;
  inliers.reserve(points.size());
  double sum = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    const double residual =
        std::abs(std::hypot(points[i].x - center_x, points[i].y - center_y) - radius);
    if (residual <= threshold) {
      inliers.push_back(i);
      sum += residual;
    }
  }
  if (residual_sum != nullptr) *residual_sum = sum;
  return inliers;
}

CylinderModel fitCylinderOnce(const PoleCloud& cloud, const std::vector<size_t>& indices,
                              const PoleExtractionConfig& config, uint32_t random_seed) {
  constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
  CylinderModel model;
  model.point_count = indices.size();
  if (indices.size() < 3) return model;

  V3 mean = V3::Zero();
  for (size_t index : indices) {
    const PolePoint& point = cloud[index];
    mean += V3(point.x, point.y, point.z);
  }
  mean /= static_cast<double>(indices.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (size_t index : indices) {
    const PolePoint& point = cloud[index];
    const V3 offset = V3(point.x, point.y, point.z) - mean;
    covariance.noalias() += offset * offset.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);
  if (eigensolver.info() != Eigen::Success) return model;
  model.axis = eigensolver.eigenvectors().col(2).normalized();
  if (model.axis.z() < 0.0) model.axis = -model.axis;
  if (!model.axis.allFinite()) return model;
  model.pca_valid = true;
  model.tilt_deg =
      std::acos(std::clamp(model.axis.z(), -1.0, 1.0)) * kRadiansToDegrees;

  const V3 reference =
      std::abs(model.axis.z()) < 0.9 ? V3::UnitZ() : V3::UnitX();
  const V3 basis_x = model.axis.cross(reference).normalized();
  const V3 basis_y = model.axis.cross(basis_x).normalized();
  std::vector<CirclePoint> projected;
  projected.reserve(indices.size());
  for (size_t index : indices) {
    const PolePoint& point = cloud[index];
    const V3 offset = V3(point.x, point.y, point.z) - mean;
    projected.push_back({offset.dot(basis_x), offset.dot(basis_y)});
  }

  std::mt19937 generator(random_seed);
  std::uniform_int_distribution<size_t> distribution(0, projected.size() - 1);
  std::vector<size_t> best_inliers;
  double best_residual_sum = std::numeric_limits<double>::infinity();
  double best_center_x = 0.0;
  double best_center_y = 0.0;
  double best_radius = 0.0;
  for (int iteration = 0; iteration < config.cylinder_ransac_iterations; ++iteration) {
    size_t first = distribution(generator);
    size_t second = distribution(generator);
    size_t third = distribution(generator);
    for (int retry = 0; retry < 8 && (second == first || third == first || third == second);
         ++retry) {
      second = distribution(generator);
      third = distribution(generator);
    }
    if (second == first || third == first || third == second) continue;
    double center_x = 0.0;
    double center_y = 0.0;
    double radius = 0.0;
    if (!circleFromThree(projected[first], projected[second], projected[third], center_x,
                         center_y, radius) ||
        radius < config.cylinder_min_radius || radius > config.cylinder_max_radius) {
      continue;
    }
    double residual_sum = 0.0;
    std::vector<size_t> inliers =
        circleInliers(projected, center_x, center_y, radius,
                      config.cylinder_inlier_threshold, &residual_sum);
    if (inliers.size() > best_inliers.size() ||
        (inliers.size() == best_inliers.size() && residual_sum < best_residual_sum)) {
      best_inliers = std::move(inliers);
      best_residual_sum = residual_sum;
      best_center_x = center_x;
      best_center_y = center_y;
      best_radius = radius;
    }
  }
  if (best_inliers.size() < 3) return model;

  // Refit twice: the first algebraic fit improves the RANSAC hypothesis, and
  // the second uses its updated consensus set.
  for (int refinement = 0; refinement < 2; ++refinement) {
    if (!leastSquaresCircle(projected, best_inliers, best_center_x, best_center_y,
                            best_radius)) {
      return model;
    }
    best_inliers = circleInliers(projected, best_center_x, best_center_y, best_radius,
                                 config.cylinder_inlier_threshold);
    if (best_inliers.size() < 3) return model;
  }

  double squared_error_sum = 0.0;
  for (const CirclePoint& point : projected) {
    const double residual =
        std::hypot(point.x - best_center_x, point.y - best_center_y) - best_radius;
    squared_error_sum += residual * residual;
  }
  model.radius = best_radius;
  model.rms = std::sqrt(squared_error_sum / static_cast<double>(projected.size()));
  model.inlier_ratio =
      static_cast<double>(best_inliers.size()) / static_cast<double>(projected.size());
  model.axis_point = mean + best_center_x * basis_x + best_center_y * basis_y;

  constexpr int kAngularBins = 36;
  std::array<bool, kAngularBins> occupied{};
  for (size_t index : best_inliers) {
    const double angle = std::atan2(projected[index].y - best_center_y,
                                    projected[index].x - best_center_x);
    int bin = static_cast<int>(std::floor((angle + 3.14159265358979323846) /
                                          (2.0 * 3.14159265358979323846) *
                                          kAngularBins));
    bin = std::clamp(bin, 0, kAngularBins - 1);
    occupied[static_cast<size_t>(bin)] = true;
  }
  model.angular_coverage = static_cast<double>(
      std::count(occupied.begin(), occupied.end(), true)) / kAngularBins;
  model.valid = model.radius >= config.cylinder_min_radius &&
                model.radius <= config.cylinder_max_radius &&
                model.rms <= config.cylinder_max_rms &&
                model.inlier_ratio >= config.cylinder_min_inlier_ratio &&
                model.angular_coverage >= config.cylinder_min_angular_coverage &&
                model.tilt_deg <= config.cylinder_max_axis_tilt_deg;
  return model;
}

bool fitCylinderAdaptive(const PoleCloud& cloud, const std::vector<size_t>& indices,
                         const PoleExtractionConfig& config, int depth, uint32_t random_seed,
                         std::vector<CylinderModel>& accepted) {
  const CylinderModel model = fitCylinderOnce(cloud, indices, config, random_seed);
  if (model.valid) {
    accepted.push_back(model);
    return true;
  }
  if (!model.pca_valid || depth >= config.cylinder_max_piecewise_depth ||
      indices.size() < 2U * static_cast<size_t>(config.cylinder_min_segment_points)) {
    return false;
  }

  std::vector<size_t> ordered = indices;
  std::sort(ordered.begin(), ordered.end(), [&cloud, &model](size_t lhs, size_t rhs) {
    const PolePoint& left = cloud[lhs];
    const PolePoint& right = cloud[rhs];
    return V3(left.x, left.y, left.z).dot(model.axis) <
           V3(right.x, right.y, right.z).dot(model.axis);
  });
  const size_t middle = ordered.size() / 2;
  std::vector<size_t> lower(ordered.begin(), ordered.begin() + middle);
  std::vector<size_t> upper(ordered.begin() + middle, ordered.end());
  if (lower.size() < static_cast<size_t>(config.cylinder_min_segment_points) ||
      upper.size() < static_cast<size_t>(config.cylinder_min_segment_points)) {
    return false;
  }

  const size_t rollback = accepted.size();
  const bool lower_valid = fitCylinderAdaptive(cloud, lower, config, depth + 1,
                                               random_seed + 0x9e3779b9U, accepted);
  const bool upper_valid = fitCylinderAdaptive(cloud, upper, config, depth + 1,
                                               random_seed + 0x85ebca6bU, accepted);
  if (!lower_valid || !upper_valid) {
    accepted.resize(rollback);
    return false;
  }
  return true;
}

ExtractionResult extract(const ExtractionTask& task, const PoleExtractionConfig& config,
                         const std::vector<PriorPole>& guided_poles) {
  ExtractionResult result;
  result.downsampled = filterByTrajectoryRoi(
      voxelDownsample(accumulateFrames(task.frames), config.input_voxel_size), task.frames,
      config, guided_poles);
  if (result.downsampled.empty()) return result;

  std::unordered_map<Cell, GridStat, CellHash> grid;
  grid.reserve(result.downsampled.size() / 8 + 1);
  for (const PolePoint& point : result.downsampled) {
    GridStat& stat = grid[toCell(point, config.grid_resolution)];
    ++stat.count;
    stat.layers.insert(static_cast<int>(
        std::floor(static_cast<double>(point.z) / config.layer_resolution)));
  }

  std::unordered_set<Cell, CellHash> candidate_cells;
  candidate_cells.reserve(grid.size());
  const int required_continuous_layers =
      std::max(config.min_continuous_layers,
               static_cast<int>(std::ceil(config.min_height_span /
                                          config.layer_resolution)));
  for (const auto& [cell, stat] : grid) {
    if (stat.count > config.min_points_per_cell &&
        longestContinuousRun(stat.layers) >= required_continuous_layers) {
      candidate_cells.insert(cell);
    }
  }
  if (candidate_cells.empty()) return result;

  const auto valid_cells =
      smallComponents(closeMask(candidate_cells), config.max_component_cells);
  if (valid_cells.empty()) return result;

  result.candidates.reserve(result.downsampled.size() / 20 + 1);
  for (const PolePoint& point : result.downsampled) {
    const Cell cell = toCell(point, config.grid_resolution);
    // Closing is only allowed to connect original vertical cells. Never bring
    // points from a filled bridge cell back into the 3-D candidate cloud.
    if (candidate_cells.find(cell) != candidate_cells.end() &&
        valid_cells.find(cell) != valid_cells.end()) {
      result.candidates.push_back(point);
    }
  }
  if (result.candidates.empty()) return result;

  const auto clusters = euclideanClusters(result.candidates, config.cluster_tolerance);
  for (const std::vector<size_t>& indices : clusters) {
    if (indices.size() < static_cast<size_t>(config.min_cluster_points) ||
        indices.size() > static_cast<size_t>(config.max_cluster_points)) {
      continue;
    }
    double sum_x = 0.0;
    double sum_y = 0.0;
    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    for (size_t index : indices) {
      const PolePoint& point = result.candidates[index];
      sum_x += point.x;
      sum_y += point.y;
      min_x = std::min(min_x, point.x);
      min_y = std::min(min_y, point.y);
      max_x = std::max(max_x, point.x);
      max_y = std::max(max_y, point.y);
    }
    const double span_x = static_cast<double>(max_x - min_x);
    const double span_y = static_cast<double>(max_y - min_y);
    if (span_x > config.max_xy_span || span_y > config.max_xy_span) continue;

    const double coarse_center_x = sum_x / static_cast<double>(indices.size());
    const double coarse_center_y = sum_y / static_cast<double>(indices.size());
    double cluster_radius = 0.0;
    for (size_t index : indices) {
      const PolePoint& point = result.candidates[index];
      cluster_radius =
          std::max(cluster_radius,
                   std::hypot(static_cast<double>(point.x) - coarse_center_x,
                              static_cast<double>(point.y) - coarse_center_y));
    }

    std::vector<CylinderModel> cylinder_segments;
    if (config.cylinder_fit_enable) {
      const uint32_t seed = static_cast<uint32_t>(task.index * 2654435761ULL + indices.front());
      if (!fitCylinderAdaptive(result.candidates, indices, config, 0, seed,
                               cylinder_segments)) {
        continue;
      }
    } else {
      CylinderModel model;
      model.axis = V3::UnitZ();
      model.axis_point = V3(coarse_center_x, coarse_center_y, 0.0);
      model.radius = cluster_radius;
      model.rms = 0.0;
      model.inlier_ratio = 1.0;
      model.angular_coverage = 1.0;
      model.tilt_deg = 0.0;
      model.point_count = indices.size();
      model.pca_valid = true;
      model.valid = true;
      cylinder_segments.push_back(model);
    }

    const double target_z =
        nearestTrajectoryZ(coarse_center_x, coarse_center_y, task.frames) +
        config.center_z_offset;
    V3 weighted_center = V3::Zero();
    V3 weighted_axis = V3::Zero();
    V3 weighted_axis_point = V3::Zero();
    double weighted_radius = 0.0;
    double weighted_squared_rms = 0.0;
    double weighted_inlier_ratio = 0.0;
    double weighted_angular_coverage = 0.0;
    size_t fitted_points = 0;
    bool usable_axis = true;
    for (const CylinderModel& model : cylinder_segments) {
      if (std::abs(model.axis.z()) < 1e-3) {
        usable_axis = false;
        break;
      }
      const double weight = static_cast<double>(model.point_count);
      const V3 center = pointOnAxisAtZ(model.axis_point, model.axis, target_z);
      weighted_center += weight * center;
      weighted_axis += weight * model.axis;
      weighted_axis_point += weight * model.axis_point;
      weighted_radius += weight * model.radius;
      weighted_squared_rms += weight * model.rms * model.rms;
      weighted_inlier_ratio += weight * model.inlier_ratio;
      weighted_angular_coverage += weight * model.angular_coverage;
      fitted_points += model.point_count;
    }
    if (!usable_axis || fitted_points == 0 || weighted_axis.norm() < 1e-9) continue;

    const double weight = static_cast<double>(fitted_points);
    const V3 axis = weighted_axis.normalized();
    const V3 center = weighted_center / weight;
    constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
    double axial_min = std::numeric_limits<double>::infinity();
    double axial_max = -std::numeric_limits<double>::infinity();
    for (size_t index : indices) {
      const PolePoint& point = result.candidates[index];
      const double axial_position =
          (V3(point.x, point.y, point.z) - center).dot(axis);
      axial_min = std::min(axial_min, axial_position);
      axial_max = std::max(axial_max, axial_position);
    }

    Detection detection;
    detection.keyframe = task.index;
    detection.stamp = task.frames.back()->stamp;
    detection.center = center;
    detection.axis = axis;
    detection.axis_point = weighted_axis_point / weight;
    detection.base_z = target_z;
    detection.radius = weighted_radius / weight;
    detection.cluster_radius = cluster_radius;
    detection.fit_rms = std::sqrt(weighted_squared_rms / weight);
    detection.inlier_ratio = weighted_inlier_ratio / weight;
    detection.angular_coverage = weighted_angular_coverage / weight;
    detection.axis_tilt_deg =
        std::acos(std::clamp(axis.z(), -1.0, 1.0)) * kRadiansToDegrees;
    detection.axial_min = axial_min;
    detection.axial_max = axial_max;
    detection.cylinder_segments = cylinder_segments.size();
    detection.span_x = span_x;
    detection.span_y = span_y;
    detection.points = indices.size();
    const double residual_variance =
        std::max(detection.fit_rms * detection.fit_rms, 0.0025);
    const double support =
        std::sqrt(static_cast<double>(std::min<size_t>(detection.points, 2000)) / 100.0);
    detection.fusion_weight = std::max(
        0.01, support * detection.inlier_ratio * detection.angular_coverage /
                  residual_variance);
    result.detections.push_back(detection);
    for (size_t index : indices) result.refined.push_back(result.candidates[index]);
  }
  return result;
}

template <typename T>
bool loadPositive(const config_internal::MergedYaml& yaml, const char* key, const T& fallback,
                  T& output) {
  output = yaml.get<T>("pole_extraction", key, fallback);
  if (output <= T(0)) {
    LOG(E, "Config error: 'pole_extraction." << key << "' must be positive, got " << output
                                               << ".");
    return false;
  }
  return true;
}

}  // namespace

bool loadPoleExtractionConfig(const std::vector<std::string>& yaml_paths,
                              PoleExtractionConfig& config) {
  config_internal::MergedYaml yaml;
  YAML::Node snapshot;
  for (const std::string& path : yaml_paths) {
    if (path.empty()) continue;
    try {
      const YAML::Node root = YAML::LoadFile(path);
      yaml.add(root);
      for (const char* section : {"pole_extraction", "rail_extraction"}) {
        if (!root[section]) continue;
        for (const auto& entry : root[section]) {
          snapshot[section][entry.first.as<std::string>()] = YAML::Clone(entry.second);
        }
      }
    } catch (const std::exception& error) {
      LOG(E, "Failed to load pole extraction config from '" << path << "': " << error.what());
      return false;
    }
  }

  config.extraction_config_yaml = YAML::Dump(snapshot);
  config.enable = yaml.get<bool>("pole_extraction", "enable", config.enable);
  config.process_partial_keyframe = yaml.get<bool>(
      "pole_extraction", "process_partial_keyframe", config.process_partial_keyframe);
  config.use_roi = yaml.get<bool>("pole_extraction", "use_roi", config.use_roi);
  config.localization_guided_roi = yaml.get<bool>(
      "pole_extraction", "localization_guided_roi", config.localization_guided_roi);
  if (!loadPositive(yaml, "localization_guided_roi_radius_m", 5.0,
                    config.localization_guided_roi_radius) ||
      !loadPositive(yaml, "localization_guided_roi_max_range_m", 40.0,
                    config.localization_guided_roi_max_range)) return false;
  config.cylinder_fit_enable = yaml.get<bool>(
      "pole_extraction", "cylinder_fit_enable", config.cylinder_fit_enable);
  config.save_keyframe_clouds = yaml.get<bool>(
      "pole_extraction", "save_keyframe_clouds", config.save_keyframe_clouds);
  config.localization_enable = yaml.get<bool>(
      "pole_extraction", "localization_enable", config.localization_enable);
  config.localization_use_initial_guess = yaml.get<bool>(
      "pole_extraction", "localization_use_initial_guess",
      config.localization_use_initial_guess);

  int frames_per_keyframe = static_cast<int>(config.frames_per_keyframe);
  int min_frames_per_keyframe = static_cast<int>(config.min_frames_per_keyframe);
  int min_partial_frames = static_cast<int>(config.min_partial_frames);
  int max_queue = static_cast<int>(config.max_queue);
  if (!loadPositive(yaml, "frames_per_keyframe", frames_per_keyframe, frames_per_keyframe) ||
      !loadPositive(yaml, "min_frames_per_keyframe", min_frames_per_keyframe,
                    min_frames_per_keyframe) ||
      !loadPositive(yaml, "min_partial_frames", min_partial_frames, min_partial_frames) ||
      !loadPositive(yaml, "max_queue", max_queue, max_queue) ||
      !loadPositive(yaml, "input_voxel_resolution_m", config.input_voxel_size,
                    config.input_voxel_size) ||
      !loadPositive(yaml, "grid_resolution_m", config.grid_resolution,
                    config.grid_resolution) ||
      !loadPositive(yaml, "min_points_per_cell", config.min_points_per_cell,
                    config.min_points_per_cell) ||
      !loadPositive(yaml, "min_height_span_m", config.min_height_span,
                    config.min_height_span) ||
      !loadPositive(yaml, "layer_resolution_m", config.layer_resolution,
                    config.layer_resolution) ||
      !loadPositive(yaml, "min_continuous_layers", config.min_continuous_layers,
                    config.min_continuous_layers) ||
      !loadPositive(yaml, "max_component_cells", config.max_component_cells,
                    config.max_component_cells) ||
      !loadPositive(yaml, "cluster_tolerance_m", config.cluster_tolerance,
                    config.cluster_tolerance) ||
      !loadPositive(yaml, "max_xy_span_m", config.max_xy_span, config.max_xy_span) ||
      !loadPositive(yaml, "min_cluster_points", config.min_cluster_points,
                    config.min_cluster_points) ||
      !loadPositive(yaml, "max_cluster_points", config.max_cluster_points,
                    config.max_cluster_points) ||
      !loadPositive(yaml, "cylinder_ransac_iterations", config.cylinder_ransac_iterations,
                    config.cylinder_ransac_iterations) ||
      !loadPositive(yaml, "cylinder_inlier_threshold_m",
                    config.cylinder_inlier_threshold,
                    config.cylinder_inlier_threshold) ||
      !loadPositive(yaml, "cylinder_min_inlier_ratio",
                    config.cylinder_min_inlier_ratio,
                    config.cylinder_min_inlier_ratio) ||
      !loadPositive(yaml, "cylinder_max_rms_m", config.cylinder_max_rms,
                    config.cylinder_max_rms) ||
      !loadPositive(yaml, "cylinder_min_radius_m", config.cylinder_min_radius,
                    config.cylinder_min_radius) ||
      !loadPositive(yaml, "cylinder_max_radius_m", config.cylinder_max_radius,
                    config.cylinder_max_radius) ||
      !loadPositive(yaml, "cylinder_max_axis_tilt_deg",
                    config.cylinder_max_axis_tilt_deg,
                    config.cylinder_max_axis_tilt_deg) ||
      !loadPositive(yaml, "cylinder_min_angular_coverage",
                    config.cylinder_min_angular_coverage,
                    config.cylinder_min_angular_coverage) ||
      !loadPositive(yaml, "cylinder_min_segment_points",
                    config.cylinder_min_segment_points,
                    config.cylinder_min_segment_points) ||
      !loadPositive(yaml, "landmark_merge_radius_m", config.landmark_merge_radius,
                    config.landmark_merge_radius) ||
      !loadPositive(yaml, "landmark_max_axis_angle_deg",
                    config.landmark_max_axis_angle_deg,
                    config.landmark_max_axis_angle_deg) ||
      !loadPositive(yaml, "landmark_max_radius_difference_m",
                    config.landmark_max_radius_difference,
                    config.landmark_max_radius_difference)) {
    return false;
  }
  config.frames_per_keyframe = static_cast<size_t>(frames_per_keyframe);
  config.min_frames_per_keyframe = static_cast<size_t>(min_frames_per_keyframe);
  config.min_partial_frames = static_cast<size_t>(min_partial_frames);
  config.max_queue = static_cast<size_t>(max_queue);
  const int overlap_frames = yaml.get<int>(
      "pole_extraction", "keyframe_overlap_frames",
      static_cast<int>(config.keyframe_overlap_frames));
  config.keyframe_distance = yaml.get<double>(
      "pole_extraction", "keyframe_distance_m", config.keyframe_distance);
  if (overlap_frames < 0 || config.keyframe_distance < 0.0) {
    LOG(E, "Config error: pole keyframe overlap and distance must be non-negative.");
    return false;
  }
  config.keyframe_overlap_frames = static_cast<size_t>(overlap_frames);
  config.cylinder_max_piecewise_depth = yaml.get<int>(
      "pole_extraction", "cylinder_max_piecewise_depth",
      config.cylinder_max_piecewise_depth);

  config.roi_y_min = yaml.get<double>("pole_extraction", "roi_y_min_m", config.roi_y_min);
  config.roi_y_max = yaml.get<double>("pole_extraction", "roi_y_max_m", config.roi_y_max);
  config.roi_z_min = yaml.get<double>("pole_extraction", "roi_z_min_m", config.roi_z_min);
  config.roi_z_max = yaml.get<double>("pole_extraction", "roi_z_max_m", config.roi_z_max);
  config.center_z_offset =
      yaml.get<double>("pole_extraction", "center_z_offset_m", config.center_z_offset);
  config.output_directory = yaml.get<std::string>(
      "pole_extraction", "output_directory", config.output_directory);
  config.mapping_trajectory_path = yaml.get<std::string>(
      "pole_extraction", "mapping_trajectory_path", config.mapping_trajectory_path);
  config.mapping_trajectory_max_time_gap_s = yaml.get<double>(
      "pole_extraction", "mapping_trajectory_max_time_gap_s",
      config.mapping_trajectory_max_time_gap_s);
  if (config.mapping_trajectory_max_time_gap_s <= 0.0 ||
      (config.localization_enable && !config.mapping_trajectory_path.empty())) {
    LOG(E, "Config error: mapping trajectory gap must be positive and a mapping "
           "trajectory cannot be used during pole localization.");
    return false;
  }

  if (config.localization_enable) {
    int window_keyframes = static_cast<int>(config.localization_window_keyframes);
    config.localization_map_path = yaml.get<std::string>(
        "pole_extraction", "localization_map_path", config.localization_map_path);
    config.localization_map_frame = yaml.get<std::string>(
        "pole_extraction", "localization_map_frame", config.localization_map_frame);
    if (!loadPositive(yaml, "localization_window_keyframes", window_keyframes,
                      window_keyframes) ||
        !loadPositive(yaml, "localization_match_distance_m",
                      config.localization_match_distance,
                      config.localization_match_distance) ||
        !loadPositive(yaml, "localization_radius_difference_m",
                      config.localization_radius_difference,
                      config.localization_radius_difference) ||
        !loadPositive(yaml, "localization_axis_angle_deg",
                      config.localization_axis_angle_deg,
                      config.localization_axis_angle_deg) ||
        !loadPositive(yaml, "localization_min_matches",
                      config.localization_min_matches,
                      config.localization_min_matches) ||
        !loadPositive(yaml, "localization_min_baseline_m",
                      config.localization_min_baseline,
                      config.localization_min_baseline) ||
        !loadPositive(yaml, "localization_max_rms_m", config.localization_max_rms,
                      config.localization_max_rms) ||
        !loadPositive(yaml, "localization_huber_delta_m",
                      config.localization_huber_delta,
                      config.localization_huber_delta) ||
        !loadPositive(yaml, "localization_max_translation_step_m",
                      config.localization_max_translation_step,
                      config.localization_max_translation_step) ||
        !loadPositive(yaml, "localization_max_yaw_step_deg",
                      config.localization_max_yaw_step_deg,
                      config.localization_max_yaw_step_deg)) {
      return false;
    }
    config.localization_window_keyframes = static_cast<size_t>(window_keyframes);
    config.localization_min_match_ratio = yaml.get<double>(
        "pole_extraction", "localization_min_match_ratio",
        config.localization_min_match_ratio);
    config.localization_initial_yaw_deg = yaml.get<double>(
        "pole_extraction", "localization_initial_yaw_deg",
        config.localization_initial_yaw_deg);
    const std::vector<double> initial_translation = yaml.get<std::vector<double>>(
        "pole_extraction", "localization_initial_translation", {0.0, 0.0, 0.0});
    const std::vector<double> initial_orientation = yaml.get<std::vector<double>>(
        "pole_extraction", "localization_initial_orientation_xyzw", {});
    if (initial_translation.size() != 3 || config.localization_map_path.empty() ||
        config.localization_map_frame.empty() ||
        (!initial_orientation.empty() && initial_orientation.size() != 4) ||
        config.localization_min_match_ratio <= 0.0 ||
        config.localization_min_match_ratio > 1.0 ||
        config.localization_axis_angle_deg > 90.0) {
      LOG(E, "Config error: invalid pole localization map, initial translation, or gate.");
      return false;
    }
    config.localization_initial_translation =
        V3(initial_translation[0], initial_translation[1], initial_translation[2]);
    if (initial_orientation.size() == 4) {
      config.localization_initial_orientation =
          Quaternion(initial_orientation[3], initial_orientation[0],
                     initial_orientation[1], initial_orientation[2]);
      if (!config.localization_initial_orientation.coeffs().allFinite() ||
          config.localization_initial_orientation.norm() < 1.0e-9) {
        LOG(E, "Config error: invalid pole localization initial quaternion.");
        return false;
      }
      config.localization_initial_orientation.normalize();
      config.localization_initial_orientation_enable = true;
    }
  }

  config.rail_enable = yaml.get<bool>("rail_extraction", "enable", config.rail_enable);
  if (config.rail_enable) {
    config.rail_mount_prior_enable = yaml.get<bool>(
        "rail_extraction", "mount_prior_enable", config.rail_mount_prior_enable);
    config.rail_mount_height = yaml.get<double>(
        "rail_extraction", "mount_height_m", config.rail_mount_height);
    config.rail_mount_center_y = yaml.get<double>(
        "rail_extraction", "mount_center_y_m", config.rail_mount_center_y);
    config.rail_mount_lateral_tolerance = yaml.get<double>(
        "rail_extraction", "mount_lateral_tolerance_m", config.rail_mount_lateral_tolerance);
    config.rail_mount_height_tolerance = yaml.get<double>(
        "rail_extraction", "mount_height_tolerance_m", config.rail_mount_height_tolerance);
    config.rail_mount_prior_weight = yaml.get<double>(
        "rail_extraction", "mount_prior_weight", config.rail_mount_prior_weight);
    config.rail_station_resolution = yaml.get<double>(
        "rail_extraction", "station_resolution_m", config.rail_station_resolution);
    config.rail_lateral_resolution = yaml.get<double>(
        "rail_extraction", "lateral_resolution_m", config.rail_lateral_resolution);
    config.rail_lateral_min = yaml.get<double>(
        "rail_extraction", "lateral_min_m", config.rail_lateral_min);
    config.rail_lateral_max = yaml.get<double>(
        "rail_extraction", "lateral_max_m", config.rail_lateral_max);
    config.rail_relative_z_min = yaml.get<double>(
        "rail_extraction", "relative_z_min_m", config.rail_relative_z_min);
    config.rail_relative_z_max = yaml.get<double>(
        "rail_extraction", "relative_z_max_m", config.rail_relative_z_max);
    config.rail_nominal_gauge = yaml.get<double>(
        "rail_extraction", "nominal_gauge_m", config.rail_nominal_gauge);
    config.rail_min_gauge = yaml.get<double>(
        "rail_extraction", "min_gauge_m", config.rail_min_gauge);
    config.rail_max_gauge = yaml.get<double>(
        "rail_extraction", "max_gauge_m", config.rail_max_gauge);
    config.rail_min_cell_points = yaml.get<int>(
        "rail_extraction", "min_cell_points", config.rail_min_cell_points);
    config.rail_min_prominence = yaml.get<double>(
        "rail_extraction", "min_prominence_m", config.rail_min_prominence);
    config.rail_max_lateral_jump = yaml.get<double>(
        "rail_extraction", "max_lateral_jump_m", config.rail_max_lateral_jump);
    config.rail_max_vertical_jump = yaml.get<double>(
        "rail_extraction", "max_vertical_jump_m", config.rail_max_vertical_jump);
    config.rail_max_cross_level = yaml.get<double>(
        "rail_extraction", "max_cross_level_m", config.rail_max_cross_level);
    const int rail_min_segment_samples = yaml.get<int>(
        "rail_extraction", "min_segment_samples",
        static_cast<int>(config.rail_min_segment_samples));
    config.rail_cloud_half_width = yaml.get<double>(
        "rail_extraction", "cloud_half_width_m", config.rail_cloud_half_width);
    config.rail_cloud_depth = yaml.get<double>(
        "rail_extraction", "cloud_depth_m", config.rail_cloud_depth);
    config.rail_cloud_above = yaml.get<double>(
        "rail_extraction", "cloud_above_m", config.rail_cloud_above);
    config.rail_map_path = yaml.get<std::string>(
        "rail_extraction", "map_path", config.rail_map_path);
    config.rail_match_distance = yaml.get<double>(
        "rail_extraction", "match_distance_m", config.rail_match_distance);
    config.rail_max_tangent_angle_deg = yaml.get<double>(
        "rail_extraction", "max_tangent_angle_deg",
        config.rail_max_tangent_angle_deg);
    const int rail_min_matches = yaml.get<int>(
        "rail_extraction", "min_matches", static_cast<int>(config.rail_min_matches));
    config.rail_min_baseline = yaml.get<double>(
        "rail_extraction", "min_baseline_m", config.rail_min_baseline);
    config.rail_huber_delta = yaml.get<double>(
        "rail_extraction", "huber_delta_m", config.rail_huber_delta);
    config.rail_max_rms = yaml.get<double>(
        "rail_extraction", "max_rms_m", config.rail_max_rms);
    config.rail_max_planar_rms = yaml.get<double>(
        "rail_extraction", "max_planar_rms_m", config.rail_max_planar_rms);
    config.rail_max_cross_track_step = yaml.get<double>(
        "rail_extraction", "max_cross_track_step_m",
        config.rail_max_cross_track_step);
    config.rail_max_yaw_step_deg = yaml.get<double>(
        "rail_extraction", "max_yaw_step_deg", config.rail_max_yaw_step_deg);
    config.rail_max_z_step = yaml.get<double>(
        "rail_extraction", "max_z_step_m", config.rail_max_z_step);
    config.rail_max_tilt_step_deg = yaml.get<double>(
        "rail_extraction", "max_tilt_step_deg", config.rail_max_tilt_step_deg);
    config.rail_update_roll = yaml.get<bool>(
        "rail_extraction", "update_roll", config.rail_update_roll);
    config.rail_update_pitch = yaml.get<bool>(
        "rail_extraction", "update_pitch", config.rail_update_pitch);
    if (config.rail_mount_prior_enable &&
        (!std::isfinite(config.rail_mount_height) || config.rail_mount_height <= 0.0 ||
         !std::isfinite(config.rail_mount_center_y) ||
         !std::isfinite(config.rail_mount_lateral_tolerance) || config.rail_mount_lateral_tolerance <= 0.0 ||
         !std::isfinite(config.rail_mount_height_tolerance) || config.rail_mount_height_tolerance <= 0.0 ||
         !std::isfinite(config.rail_mount_prior_weight) || config.rail_mount_prior_weight <= 0.0)) {
      LOG(E, "Config error: invalid rail mounting prior.");
      return false;
    }
    if (!std::isfinite(config.rail_cloud_half_width) || config.rail_cloud_half_width <= 0.0 ||
        !std::isfinite(config.rail_cloud_depth) || config.rail_cloud_depth <= 0.0 ||
        !std::isfinite(config.rail_cloud_above) || config.rail_cloud_above <= 0.0 ||
        config.rail_station_resolution <= 0.0 ||
        config.rail_lateral_resolution <= 0.0 ||
        config.rail_lateral_min >= config.rail_lateral_max ||
        config.rail_relative_z_min >= config.rail_relative_z_max ||
        config.rail_min_gauge <= 0.0 ||
        config.rail_min_gauge >= config.rail_nominal_gauge ||
        config.rail_nominal_gauge >= config.rail_max_gauge ||
        config.rail_min_cell_points <= 0 || rail_min_segment_samples <= 0 ||
        rail_min_matches <= 0 ||
        config.rail_max_vertical_jump <= 0.0 || config.rail_max_cross_level <= 0.0 ||
        config.rail_match_distance <= 0.0 ||
        config.rail_min_baseline <= 0.0 || config.rail_huber_delta <= 0.0 ||
        config.rail_max_rms <= 0.0 || config.rail_max_planar_rms <= 0.0 ||
        config.rail_max_cross_track_step <= 0.0 ||
        config.rail_max_yaw_step_deg <= 0.0 || config.rail_max_z_step <= 0.0 ||
        config.rail_max_tilt_step_deg <= 0.0) {
      LOG(E, "Config error: invalid rail_extraction parameter.");
      return false;
    }
    config.rail_min_segment_samples =
        static_cast<size_t>(rail_min_segment_samples);
    config.rail_min_matches = static_cast<size_t>(rail_min_matches);
    if (config.localization_enable && config.rail_map_path.empty()) {
      LOG(E, "Config error: rail_extraction.map_path is required in sparse-map "
             "localization mode.");
      return false;
    }
  }

  if (config.min_partial_frames > config.frames_per_keyframe) {
    LOG(E, "Config error: pole_extraction.min_partial_frames cannot exceed "
               "frames_per_keyframe.");
    return false;
  }
  if (config.min_frames_per_keyframe > config.frames_per_keyframe ||
      config.keyframe_overlap_frames >= config.min_frames_per_keyframe) {
    LOG(E, "Config error: min_frames_per_keyframe must not exceed frames_per_keyframe, "
           "and overlap must be smaller than min_frames_per_keyframe.");
    return false;
  }
  if (config.roi_y_min >= config.roi_y_max || config.roi_z_min >= config.roi_z_max) {
    LOG(E, "Config error: pole extraction ROI minimums must be smaller than maximums.");
    return false;
  }
  if (config.min_cluster_points > config.max_cluster_points) {
    LOG(E, "Config error: pole_extraction.min_cluster_points cannot exceed "
               "max_cluster_points.");
    return false;
  }
  if (config.cylinder_max_piecewise_depth < 0 ||
      config.cylinder_min_radius >= config.cylinder_max_radius ||
      config.cylinder_min_inlier_ratio > 1.0 ||
      config.cylinder_min_angular_coverage > 1.0 ||
      config.cylinder_max_axis_tilt_deg > 90.0 ||
      config.landmark_max_axis_angle_deg > 90.0) {
    LOG(E, "Config error: invalid pole cylinder fitting range or ratio.");
    return false;
  }

  if (config.enable) {
    LOG(I, "Pole extraction enabled: " << config.frames_per_keyframe
                                        << " max scans/keyframe, "
                                        << config.keyframe_distance << " m trigger, grid "
                                        << config.grid_resolution << " m, output '"
                                        << config.output_directory << "', cylinder fit "
                                        << (config.cylinder_fit_enable ? "on" : "off") << ".");
    if (config.localization_enable) {
      LOG(I, "Pole map localization enabled: prior '" << config.localization_map_path
                                                        << "', "
                                                        << config.localization_window_keyframes
                                                        << " keyframe window, minimum "
                                                        << config.localization_min_matches
                                                        << " matches.");
    } else if (!config.mapping_trajectory_path.empty()) {
      LOG(I, "Pole mapping will use reference trajectory '"
                 << config.mapping_trajectory_path << "'.");
    }
    if (config.rail_enable) {
      LOG(I, "Rail-vector extraction enabled: station resolution "
                 << config.rail_station_resolution << " m, gauge "
                 << config.rail_min_gauge << ".." << config.rail_max_gauge << " m.");
      if (config.rail_mount_prior_enable) {
        LOG(I, "Rail mounting prior: body center Y " << config.rail_mount_center_y
                   << " m, height " << config.rail_mount_height << " m, lateral/height tolerances "
                   << config.rail_mount_lateral_tolerance << "/"
                   << config.rail_mount_height_tolerance << " m.");
      }
    }
  }
  return true;
}

class PoleExtractor::Impl {
 public:
  Impl(ros::NodeHandle* nh, const std::shared_ptr<Pipeline>& pipeline,
       PoleExtractionConfig config, const std::string& topic_prefix,
       const std::string& map_frame = "map")
      : config_(std::move(config)), map_frame_(pipeline ? pipeline->config().map_frame : map_frame),
        offline_(!nh) {
    if (nh) {
      centers_pub_ =
          nh->advertise<sensor_msgs::PointCloud2>(topic_prefix + "/poles/centers", 2, true);
      points_pub_ =
          nh->advertise<sensor_msgs::PointCloud2>(topic_prefix + "/poles/points", 2, true);
      map_pub_ = nh->advertise<sensor_msgs::PointCloud2>(topic_prefix + "/poles/map", 1, true);
      prior_map_pub_ =
          nh->advertise<sensor_msgs::PointCloud2>(topic_prefix + "/poles/prior_map", 1, true);
      correction_pub_ =
          nh->advertise<nav_msgs::Odometry>(topic_prefix + "/poles/correction", 1, true);
      markers_pub_ = nh->advertise<visualization_msgs::MarkerArray>(
          topic_prefix + "/poles/markers", 1, true);
      rail_points_pub_ = nh->advertise<sensor_msgs::PointCloud2>(
          topic_prefix + "/rails/observed", 2, true);
      rail_map_pub_ = nh->advertise<sensor_msgs::PointCloud2>(
          topic_prefix + "/rails/map", 1, true);
    }
    if (config_.localization_enable) {
      std::string message;
      localization_ready_ =
          loadPriorPoles(config_.localization_map_path, prior_poles_, &message);
      if (localization_ready_) {
        LOG(I, message);
        if (config_.localization_use_initial_guess) {
          const Rotation rotation = config_.localization_initial_orientation_enable
              ? config_.localization_initial_orientation.toRotationMatrix()
              : yawRotation(config_.localization_initial_yaw_deg *
                            3.14159265358979323846 / 180.0);
          configured_seed_ =
              Transform(rotation, config_.localization_initial_translation);
        }
      } else {
        LOG(E, message);
      }
      if (config_.rail_enable) {
        std::string rail_message;
        rail_localization_ready_ =
            loadRailMap(config_.rail_map_path, prior_rails_, &rail_message);
        if (rail_localization_ready_) {
          prior_rail_segments_ = buildRailSegments(prior_rails_);
          rail_localization_ready_ = !prior_rail_segments_.empty();
          if (rail_localization_ready_) {
            LOG(I, rail_message << " Built " << prior_rail_segments_.size()
                                << " continuous rail segments.");
          } else {
            LOG(E, "Rail vector map contains no adjacent samples that can form segments: '"
                       << config_.rail_map_path << "'.");
          }
        } else {
          LOG(E, rail_message);
        }
      }
    }
    if (!config_.mapping_trajectory_path.empty()) {
      std::string message;
      mapping_trajectory_ready_ =
          loadTumTrajectory(config_.mapping_trajectory_path, mapping_trajectory_, &message);
      if (mapping_trajectory_ready_) {
        LOG(I, message);
      } else {
        LOG(E, message);
      }
    }
    worker_ = std::thread(&Impl::workerLoop, this);
    if (pipeline) pipeline->addFrameObserver(
        [this](uint64_t stamp, const Transform& pose, const Pointcloud& cloud) {
          addFrame(stamp, pose, cloud);
        });
  }

  ~Impl() { finish(); }

  void addFrame(uint64_t stamp, const Transform& pose, const Pointcloud& cloud) {
    if (!accepting_.load(std::memory_order_acquire)) return;
    Transform extraction_pose = pose;
    if (!config_.mapping_trajectory_path.empty()) {
      const auto reference = interpolateTrajectory(
          mapping_trajectory_, stamp, config_.mapping_trajectory_max_time_gap_s);
      if (!reference) {
        ++rejected_trajectory_frames_;
        return;
      }
      extraction_pose = *reference;
    }
    auto frame = std::make_shared<ExtractionFrame>();
    frame->stamp = stamp;
    frame->pose = extraction_pose;
    frame->cloud = cloud;

    std::unique_lock<std::mutex> lock(queue_mutex_);
    if (!accepting_.load(std::memory_order_relaxed)) return;
    ++received_frames_;
    if (has_last_frame_position_) {
      const double traveled =
          (extraction_pose.translation() - last_frame_position_).head<2>().norm();
      distance_since_keyframe_ += traveled;
      odom_distance_m_ += traveled;
    }
    frame->odom_distance_m = odom_distance_m_;
    last_frame_position_ = extraction_pose.translation();
    has_last_frame_position_ = true;
    ++new_frames_since_keyframe_;
    current_frames_.push_back(std::move(frame));
    const bool reached_frame_limit =
        current_frames_.size() >= config_.frames_per_keyframe;
    const bool reached_distance =
        config_.keyframe_distance > 0.0 &&
        current_frames_.size() >= config_.min_frames_per_keyframe &&
        distance_since_keyframe_ >= config_.keyframe_distance;
    if (!reached_frame_limit && !reached_distance) return;
    if (offline_) {
      queue_cv_.wait(lock, [this] { return queue_.size() < config_.max_queue || stop_; });
      if (stop_) return;
    }
    queueCurrentKeyframeLocked();
    queue_cv_.notify_one();
  }

  void finish() {
    bool expected = true;
    if (!accepting_.compare_exchange_strong(expected, false)) {
      if (worker_.joinable()) worker_.join();
      return;
    }
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (config_.process_partial_keyframe &&
          new_frames_since_keyframe_ >= config_.min_partial_frames) {
        queueCurrentKeyframeLocked();
      } else {
        current_frames_.clear();
      }
      stop_ = true;
    }
    queue_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

  PoleExtractionStats stats() const {
    PoleExtractionStats output;
    output.received_frames = received_frames_.load();
    output.processed_keyframes = processed_keyframes_.load();
    output.dropped_keyframes = dropped_keyframes_.load();
    output.rejected_trajectory_frames = rejected_trajectory_frames_.load();
    output.detections = detection_count_.load();
    output.localization_attempts = localization_attempts_.load();
    output.localization_accepted = localization_accepted_.load();
    output.rail_localization_attempts = rail_localization_attempts_.load();
    output.rail_localization_accepted = rail_localization_accepted_.load();
    std::lock_guard<std::mutex> lock(results_mutex_);
    output.landmarks = landmarks_.size();
    output.rail_samples = rail_samples_.size();
    return output;
  }

  void setCorrectionProvider(PoleExtractor::CorrectionProvider provider) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    correction_provider_ = std::move(provider);
  }

  void setCorrectionObserver(PoleExtractor::CorrectionObserver observer) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    correction_observer_ = std::move(observer);
  }

  void setKeyframeObserver(PoleExtractor::KeyframeObserver observer) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    keyframe_observer_ = std::move(observer);
  }

  PoleLocalizationCorrection localizationCorrection() const {
    std::lock_guard<std::mutex> lock(correction_mutex_);
    return localization_correction_;
  }

  bool localizationReady() const {
    const bool mapping_ready =
        config_.mapping_trajectory_path.empty() || mapping_trajectory_ready_;
    const bool rail_ready = !config_.localization_enable || !config_.rail_enable ||
                            rail_localization_ready_;
    return mapping_ready && (!config_.localization_enable || localization_ready_) &&
           rail_ready;
  }

  bool saveResults(std::string* message) const {
    namespace fs = std::filesystem;
    PoleCloud pole_points;
    PoleCloud rail_points;
    std::vector<Detection> detections;
    std::vector<Landmark> landmarks;
    std::vector<RailSample> rails;
    std::vector<PoleLocalizationCorrection> corrections;
    {
      std::lock_guard<std::mutex> lock(results_mutex_);
      pole_points = pole_points_;
      rail_points = rail_points_;
      detections = detections_;
      landmarks = landmarks_;
      rails = rail_samples_;
    }
    {
      std::lock_guard<std::mutex> lock(correction_mutex_);
      corrections = correction_history_;
    }
    if (config_.output_directory.empty()) {
      if (message) *message = "Pole output directory is empty; results were not saved.";
      return false;
    }

    std::error_code error;
    fs::create_directories(config_.output_directory, error);
    if (error) {
      if (message) *message = "Failed to create pole output directory: " + error.message();
      return false;
    }

    std::sort(landmarks.begin(), landmarks.end(),
              [](const Landmark& lhs, const Landmark& rhs) { return lhs.id < rhs.id; });

    const fs::path directory(config_.output_directory);
    if (!config_.extraction_config_yaml.empty()) {
      YAML::Node snapshot = YAML::Load(config_.extraction_config_yaml);
      snapshot["pole_extraction"]["enable"] = config_.enable;
      snapshot["pole_extraction"]["localization_enable"] = config_.localization_enable;
      snapshot["pole_extraction"]["mapping_trajectory_path"] = config_.mapping_trajectory_path;
      snapshot["pole_extraction"]["output_directory"] = config_.output_directory;
      snapshot["pole_extraction"]["save_keyframe_clouds"] = config_.save_keyframe_clouds;
      snapshot["rail_extraction"]["enable"] = config_.rail_enable;
      std::ofstream snapshot_file(directory / "extraction_config.yaml");
      snapshot_file << snapshot << "\n";
      snapshot_file.close();
      if (!snapshot_file) {
        if (message) *message = "Failed to save extraction configuration.";
        return false;
      }
    }

    std::ofstream landmark_file(directory / "landmarks.txt");
    std::ofstream detection_file(directory / "detections.txt");
    std::ofstream summary_file(directory / "summary.yaml");
    std::ofstream rail_file;
    if (config_.rail_enable) rail_file.open(directory / "rail_vector_map.txt");
    std::ofstream correction_file;
    if (config_.localization_enable) {
      correction_file.open(directory / "localization_corrections.txt");
    }
    if (!landmark_file || !detection_file || !summary_file ||
        (config_.rail_enable && !rail_file) ||
        (config_.localization_enable && !correction_file)) {
      if (message) *message = "Failed to open one or more pole result files.";
      return false;
    }

    if (config_.rail_enable) {
      rail_file << "Segment Index Timestamp Station LeftX LeftY LeftZ RightX RightY RightZ "
                   "TangentX TangentY TangentZ Gauge Quality\n"
                << std::fixed << std::setprecision(9);
      for (const RailSample& rail : rails) {
        rail_file << rail.segment << " " << rail.index << " " << nsToS(rail.stamp)
                  << " " << rail.station << " " << rail.left.x() << " "
                  << rail.left.y() << " " << rail.left.z() << " " << rail.right.x()
                  << " " << rail.right.y() << " " << rail.right.z() << " "
                  << rail.tangent.x() << " " << rail.tangent.y() << " "
                  << rail.tangent.z() << " " << rail.gauge << " " << rail.quality
                  << "\n";
      }
    }

    landmark_file
        << "ID X Y Z AxisX AxisY AxisZ AxialMin AxialMax Observations TotalPoints Radius FitRMS\n"
        << std::fixed << std::setprecision(6);
    for (const Landmark& landmark : landmarks) {
      landmark_file << landmark.id << " " << landmark.center.x() << " "
                    << landmark.center.y() << " "
                    << landmark.center.z() << " " << landmark.axis.x() << " "
                    << landmark.axis.y() << " " << landmark.axis.z() << " "
                    << landmark.axial_min << " " << landmark.axial_max << " "
                    << landmark.observations << " " << landmark.total_points << " "
                    << landmark.radius << " " << landmark.fit_rms << "\n";
    }

    detection_file
        << "Keyframe Timestamp X Y Z AxisX AxisY AxisZ CylinderRadius FitRMS "
           "InlierRatio AngularCoverage AxisTiltDeg Segments ClusterRadius SpanX SpanY "
           "AxialMin AxialMax TotalPoints\n"
        << std::fixed << std::setprecision(9);
    for (const Detection& detection : detections) {
      detection_file << detection.keyframe << " " << nsToS(detection.stamp) << " "
                     << detection.center.x() << " " << detection.center.y() << " "
                     << detection.center.z() << " " << detection.axis.x() << " "
                     << detection.axis.y() << " " << detection.axis.z() << " "
                     << detection.radius << " " << detection.fit_rms << " "
                     << detection.inlier_ratio << " " << detection.angular_coverage << " "
                     << detection.axis_tilt_deg << " " << detection.cylinder_segments << " "
                     << detection.cluster_radius << " " << detection.span_x << " "
                     << detection.span_y << " " << detection.axial_min << " "
                     << detection.axial_max << " " << detection.points << "\n";
    }

    if (config_.localization_enable) {
      correction_file
          << "Timestamp Type OdomDistance X Y Z QX QY QZ QW Matches MatchRatio "
             "Baseline RMS PlanarRMS RailTangentX RailTangentY "
             "CrossTrackEnabled YawEnabled\n"
          << std::fixed << std::setprecision(9);
      for (const PoleLocalizationCorrection& correction : corrections) {
        const V3& translation = correction.T_map_odom.translation();
        const Quaternion quaternion(correction.T_map_odom.rotation());
        correction_file << nsToS(correction.stamp) << " "
                        << (correction.constrain_vertical ? "RAIL" : "POLE") << " "
                        << correction.odom_distance_m
                        << " " << translation.x() << " "
                        << translation.y() << " " << translation.z() << " "
                        << quaternion.x() << " " << quaternion.y() << " "
                        << quaternion.z() << " " << quaternion.w() << " "
                        << correction.matches << " " << correction.match_ratio << " "
                        << correction.baseline << " " << correction.rms << " "
                        << correction.planar_rms << " "
                        << correction.rail_tangent_map.x() << " "
                        << correction.rail_tangent_map.y() << " "
                        << (correction.constrain_cross_track ? 1 : 0) << " "
                        << (correction.constrain_yaw ? 1 : 0) << "\n";
      }
    }

    const PoleExtractionStats snapshot = stats();
    summary_file << "received_frames: " << snapshot.received_frames << "\n"
                 << "processed_keyframes: " << snapshot.processed_keyframes << "\n"
                 << "dropped_keyframes: " << snapshot.dropped_keyframes << "\n"
                 << "rejected_trajectory_frames: "
                 << snapshot.rejected_trajectory_frames << "\n"
                 << "detections: " << snapshot.detections << "\n"
                 << "landmarks: " << snapshot.landmarks << "\n"
                 << "localization_attempts: " << snapshot.localization_attempts << "\n"
                 << "localization_accepted: " << snapshot.localization_accepted << "\n"
                 << "rail_samples: " << snapshot.rail_samples << "\n"
                 << "rail_localization_attempts: "
                 << snapshot.rail_localization_attempts << "\n"
                 << "rail_localization_accepted: "
                 << snapshot.rail_localization_accepted << "\n";
    if (!landmark_file || !detection_file || !summary_file ||
        (config_.rail_enable && !rail_file) ||
        (config_.localization_enable && !correction_file)) {
      if (message) *message = "Failed while writing pole text results.";
      return false;
    }

    if (config_.save_keyframe_clouds &&
        !writeXYZPcd(directory / "poles.pcd", pole_points)) {
      if (message) *message = "Failed to save accumulated pole point cloud.";
      return false;
    }
    if (config_.rail_enable && !writeXYZPcd(directory / "rails.pcd", rail_points)) {
      if (message) *message = "Failed to save accumulated rail point cloud.";
      return false;
    }

    if (!writeLandmarkPcd(directory / "landmarks.pcd", landmarks)) {
      if (message) *message = "Failed to save pole landmark PCD.";
      return false;
    }

    if (message) {
      *message = "Saved " + std::to_string(landmarks.size()) + " pole landmarks (" +
                 std::to_string(detections.size()) + " detections) to '" +
                 directory.string() + "'.";
    }
    return true;
  }

 private:
  std::vector<PriorPole> guidedRoiPoles(const ExtractionTask& task) const {
    std::vector<PriorPole> result;
    if (!config_.localization_enable || !config_.localization_guided_roi ||
        !config_.use_roi) return result;
    const auto seed = correctionSeed();
    if (!seed || !seed->matrix().allFinite()) return result;
    const Transform inverse(Eigen::Isometry3d(seed->inverse()));
    for (const PriorPole& prior : prior_poles_) {
      PriorPole pole = prior;
      pole.point = inverse * prior.point;
      pole.axis = inverse.rotation() * prior.axis;
      if (std::abs(pole.axis.z()) < 0.5) continue;
      // Bound work to axes near at least one actual sensor position in this
      // keyframe, including the vertical displacement of a tilted axis.
      for (const auto& frame : task.frames) {
        const V3 position = frame->pose.translation();
        const V3 on_axis = pole.point +
            ((position.z() - pole.point.z()) / pole.axis.z()) * pole.axis;
        if ((position - on_axis).head<2>().norm() <=
            config_.localization_guided_roi_max_range) {
          result.push_back(pole);
          break;
        }
      }
    }
    return result;
  }

  std::vector<LocalPole> fuseRecentDetections() const {
    std::vector<LocalPole> local;
    for (const auto& frame : recent_detection_frames_) {
      for (const Detection& detection : frame) {
        LocalPole observation;
        observation.point = detection.center;
        observation.axis = detection.axis;
        observation.radius = detection.radius;
        observation.weight = detection.fusion_weight;
        LocalPole* match = nullptr;
        double best_distance = config_.landmark_merge_radius;
        for (LocalPole& candidate : local) {
          const double distance =
              (candidate.point - observation.point).head<2>().norm();
          if (distance <= best_distance &&
              std::abs(candidate.radius - observation.radius) <=
                  config_.landmark_max_radius_difference) {
            match = &candidate;
            best_distance = distance;
          }
        }
        if (match == nullptr) {
          local.push_back(observation);
          continue;
        }
        const double total_weight = match->weight + observation.weight;
        V3 aligned_axis = observation.axis;
        if (match->axis.dot(aligned_axis) < 0.0) aligned_axis = -aligned_axis;
        match->point = (match->weight * match->point + observation.weight * observation.point) /
                       total_weight;
        match->axis =
            (match->weight * match->axis + observation.weight * aligned_axis).normalized();
        match->radius =
            (match->weight * match->radius + observation.weight * observation.radius) /
            total_weight;
        match->weight = total_weight;
      }
    }
    return local;
  }

  std::vector<PoleMatch> associatePoles(const std::vector<LocalPole>& local,
                                        const Transform& T_map_odom) const {
    constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
    constexpr double kInvalidCost = 1.0e6;
    const double minimum_axis_dot =
        std::cos(config_.localization_axis_angle_deg * kDegreesToRadians);
    std::vector<std::vector<double>> costs(
        local.size(), std::vector<double>(prior_poles_.size(), kInvalidCost));
    for (size_t i = 0; i < local.size(); ++i) {
      const V3 moved_point = T_map_odom * local[i].point;
      V3 moved_axis = T_map_odom.rotation() * local[i].axis;
      moved_axis.normalize();
      for (size_t j = 0; j < prior_poles_.size(); ++j) {
        const PriorPole& prior = prior_poles_[j];
        const double common_z = 0.5 * (moved_point.z() + prior.point.z());
        const V3 current_at_z = pointOnAxisAtZ(moved_point, moved_axis, common_z);
        const V3 prior_at_z = pointOnAxisAtZ(prior.point, prior.axis, common_z);
        const double distance = (current_at_z - prior_at_z).head<2>().norm();
        const double radius_difference = std::abs(local[i].radius - prior.radius);
        const double axis_dot =
            std::clamp(std::abs(moved_axis.dot(prior.axis)), 0.0, 1.0);
        if (distance > config_.localization_match_distance ||
            radius_difference > config_.localization_radius_difference ||
            axis_dot < minimum_axis_dot) {
          continue;
        }
        const double axis_angle = std::acos(axis_dot) / kDegreesToRadians;
        const double radius_term =
            radius_difference / config_.localization_radius_difference;
        const double axis_term = axis_angle / config_.localization_axis_angle_deg;
        costs[i][j] = distance * distance + 0.25 * radius_term * radius_term +
                      0.25 * axis_term * axis_term;
      }
    }
    return hungarianAssignment(costs, kInvalidCost);
  }

  bool fitPlanarCorrection(const std::vector<LocalPole>& local,
                           const std::vector<PoleMatch>& matches,
                           const Rotation& roll_pitch, const Transform& current,
                           double& yaw, Eigen::Vector2d& translation) const {
    if (matches.size() < static_cast<size_t>(config_.localization_min_matches)) return false;
    double total_weight = 0.0;
    Eigen::Vector2d source_mean = Eigen::Vector2d::Zero();
    Eigen::Vector2d target_mean = Eigen::Vector2d::Zero();
    std::vector<double> weights;
    std::vector<Eigen::Vector2d> sources;
    std::vector<Eigen::Vector2d> targets;
    weights.reserve(matches.size());
    sources.reserve(matches.size());
    targets.reserve(matches.size());
    for (const PoleMatch& match : matches) {
      const Eigen::Vector2d source =
          (roll_pitch * local[match.local].point).head<2>();
      const Eigen::Vector2d target = prior_poles_[match.prior].point.head<2>();
      const double residual =
          ((current * local[match.local].point).head<2>() - target).norm();
      const double robust =
          std::min(1.0, config_.localization_huber_delta / std::max(residual, 1.0e-6));
      const double quality = std::sqrt(
          std::max(0.01, local[match.local].weight * prior_poles_[match.prior].quality));
      const double weight = robust * quality;
      sources.push_back(source);
      targets.push_back(target);
      weights.push_back(weight);
      source_mean += weight * source;
      target_mean += weight * target;
      total_weight += weight;
    }
    if (total_weight <= 1e-12) return false;
    source_mean /= total_weight;
    target_mean /= total_weight;
    double dot = 0.0;
    double cross = 0.0;
    for (size_t i = 0; i < sources.size(); ++i) {
      const Eigen::Vector2d source = sources[i] - source_mean;
      const Eigen::Vector2d target = targets[i] - target_mean;
      dot += weights[i] * source.dot(target);
      cross += weights[i] * (source.x() * target.y() - source.y() * target.x());
    }
    if (std::hypot(dot, cross) < 1e-9) return false;
    yaw = std::atan2(cross, dot);
    const Eigen::Matrix2d rotation = yawRotation(yaw).topLeftCorner<2, 2>();
    translation = target_mean - rotation * source_mean;
    return true;
  }

  std::optional<Transform> correctionSeed() const {
    PoleExtractor::CorrectionProvider provider;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      provider = correction_provider_;
    }
    if (provider) {
      if (const std::optional<Transform> supplied = provider()) return supplied;
    }
    const PoleLocalizationCorrection previous = localizationCorrection();
    if (previous.valid) return previous.T_map_odom;
    if (configured_seed_) return configured_seed_;
    return std::nullopt;
  }

  std::optional<PoleLocalizationCorrection> estimatePoleCorrection(
      uint64_t stamp, double odom_distance_m) {
    const std::vector<LocalPole> local = fuseRecentDetections();
    if (local.size() < static_cast<size_t>(config_.localization_min_matches)) {
      LOG(I, "Pole localization skipped: only " << local.size()
                                                << " local landmarks in the window.");
      return std::nullopt;
    }
    const std::optional<Transform> seed = correctionSeed();
    if (!seed) return std::nullopt;
    ++localization_attempts_;

    const double seed_yaw = yawOf(seed->rotation());
    const Rotation roll_pitch = yawRotation(-seed_yaw) * seed->rotation();
    Transform current = *seed;
    std::vector<PoleMatch> matches;
    for (int iteration = 0; iteration < 8; ++iteration) {
      matches = associatePoles(local, current);
      if (matches.size() < static_cast<size_t>(config_.localization_min_matches)) {
        LOG(I, "Pole localization skipped: " << matches.size() << "/" << local.size()
                                              << " associations after distance/axis/radius gates.");
        return std::nullopt;
      }
      double yaw = 0.0;
      Eigen::Vector2d translation = Eigen::Vector2d::Zero();
      if (!fitPlanarCorrection(local, matches, roll_pitch, current, yaw, translation)) {
        return std::nullopt;
      }
      Transform updated(yawRotation(yaw) * roll_pitch,
                        V3(translation.x(), translation.y(), seed->translation().z()));
      const double change =
          (updated.translation() - current.translation()).head<2>().norm();
      const double angle_change =
          std::abs(wrappedAngle(yawOf(updated.rotation()) - yawOf(current.rotation())));
      current = updated;
      if (change < 1e-6 && angle_change < 1e-8) break;
    }

    matches = associatePoles(local, current);
    if (matches.size() < static_cast<size_t>(config_.localization_min_matches)) {
      return std::nullopt;
    }
    const double match_ratio =
        static_cast<double>(matches.size()) / static_cast<double>(local.size());
    double squared_error = 0.0;
    double baseline = 0.0;
    for (size_t i = 0; i < matches.size(); ++i) {
      const PoleMatch& match = matches[i];
      const V3 moved = current * local[match.local].point;
      const V3& target = prior_poles_[match.prior].point;
      squared_error += (moved - target).head<2>().squaredNorm();
      for (size_t j = i + 1; j < matches.size(); ++j) {
        baseline = std::max(
            baseline,
            (local[match.local].point - local[matches[j].local].point).head<2>().norm());
      }
    }
    const double rms = std::sqrt(squared_error / static_cast<double>(matches.size()));
    V3 centre = V3::Zero();
    for (const PoleMatch& match : matches) centre += local[match.local].point;
    centre /= static_cast<double>(matches.size());
    const double translation_step =
        ((current * centre) - (*seed * centre)).head<2>().norm();
    constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
    const double yaw_step = std::abs(wrappedAngle(yawOf(current.rotation()) -
                                                   yawOf(seed->rotation()))) *
                            kRadiansToDegrees;
    if (match_ratio < config_.localization_min_match_ratio ||
        baseline < config_.localization_min_baseline ||
        rms > config_.localization_max_rms ||
        translation_step > config_.localization_max_translation_step ||
        yaw_step > config_.localization_max_yaw_step_deg) {
      LOG(W, "Rejected pole correction: " << matches.size() << "/" << local.size()
                                            << " matches, baseline " << baseline
                                            << " m, RMS " << rms << " m, step "
                                            << translation_step << " m / " << yaw_step
                                            << " deg.");
      return std::nullopt;
    }

    PoleLocalizationCorrection correction;
    correction.T_map_odom = current;
    correction.stamp = stamp;
    correction.odom_distance_m = odom_distance_m;
    correction.correction_anchor_odom = centre;
    correction.matches = matches.size();
    correction.match_ratio = match_ratio;
    correction.baseline = baseline;
    correction.rms = rms;
    correction.valid = true;
    return correction;
  }

  struct RailMatch {
    size_t local = 0;
    size_t prior_segment = 0;
    double projection_ratio = 0.0;
    bool swap_sides = false;
  };

  RailSegmentProjection railTarget(const RailMatch& match) const {
    const RailSegment& segment = prior_rail_segments_[match.prior_segment];
    RailSegmentProjection target;
    target.ratio = match.projection_ratio;
    target.center = segment.center_start + target.ratio *
        (segment.center_finish - segment.center_start);
    target.left = segment.start.left + target.ratio *
        (segment.finish.left - segment.start.left);
    target.right = segment.start.right + target.ratio *
        (segment.finish.right - segment.start.right);
    target.planar_distance_sq = 0.0;
    return target;
  }

  std::vector<RailMatch> associateRails(const std::vector<RailSample>& local,
                                        const Transform& correction) const {
    constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
    const double tangent_dot_gate =
        std::cos(config_.rail_max_tangent_angle_deg * kDegreesToRadians);
    const double distance_sq_gate =
        config_.rail_match_distance * config_.rail_match_distance;
    std::vector<RailMatch> matches;
    std::unordered_set<size_t> used_segments;
    for (size_t i = 0; i < local.size(); ++i) {
      const V3 local_center = 0.5 * (local[i].left + local[i].right);
      const V3 moved_center = correction * local_center;
      const V3 moved_tangent = correction.rotation() * local[i].tangent;
      size_t best = prior_rail_segments_.size();
      double best_distance_sq = distance_sq_gate;
      RailSegmentProjection best_projection;
      for (size_t j = 0; j < prior_rail_segments_.size(); ++j) {
        if (used_segments.find(j) != used_segments.end()) continue;
        const RailSegment& segment = prior_rail_segments_[j];
        const RailSegmentProjection projection =
            projectToRailSegment(moved_center, segment);
        const double tangent_dot = std::abs(
            moved_tangent.normalized().dot(segment.tangent));
        if (projection.planar_distance_sq <= best_distance_sq &&
            tangent_dot >= tangent_dot_gate) {
          best = j;
          best_distance_sq = projection.planar_distance_sq;
          best_projection = projection;
        }
      }
      if (best == prior_rail_segments_.size()) continue;
      const double direct =
          ((correction * local[i].left) - best_projection.left).head<2>().squaredNorm() +
          ((correction * local[i].right) - best_projection.right).head<2>().squaredNorm();
      const double swapped =
          ((correction * local[i].left) - best_projection.right).head<2>().squaredNorm() +
          ((correction * local[i].right) - best_projection.left).head<2>().squaredNorm();
      matches.push_back({i, best, best_projection.ratio, swapped < direct});
      used_segments.insert(best);
    }
    return matches;
  }

  std::optional<PoleLocalizationCorrection> estimateRailCorrection(
      uint64_t stamp, double odom_distance_m,
      const std::vector<RailSample>& local) {
    if (local.size() < config_.rail_min_matches) return std::nullopt;
    const std::optional<Transform> seed = correctionSeed();
    if (!seed) return std::nullopt;
    ++rail_localization_attempts_;
    Transform current = *seed;
    std::vector<RailMatch> matches;
    // A rigid initial map seed cannot represent accumulated LIO height drift.
    // At the first rail-covered keyframe the vertical discrepancy can already
    // be large, so give bootstrap enough range to converge.  Tracking remains
    // deliberately conservative after the first accepted rail correction.
    const int max_iterations = 30;
    const double max_z_iteration = 5.0;
    auto referenceTangent = [this](
        const std::vector<RailMatch>& rail_matches) -> Eigen::Vector2d {
      Eigen::Vector2d reference = Eigen::Vector2d::Zero();
      for (const RailMatch& match : rail_matches) {
        Eigen::Vector2d tangent =
            prior_rail_segments_[match.prior_segment].tangent.head<2>();
        if (tangent.norm() < 1.0e-8) continue;
        tangent.normalize();
        if (reference.squaredNorm() > 1.0e-8 && reference.dot(tangent) < 0.0) {
          tangent = -tangent;
        }
        reference += tangent;
      }
      if (reference.norm() < 1.0e-8) return Eigen::Vector2d(1.0, 0.0);
      return reference.normalized();
    };
    for (int iteration = 0; iteration < max_iterations; ++iteration) {
      matches = associateRails(local, current);
      if (matches.size() < config_.rail_min_matches) return std::nullopt;
      V3 anchor = V3::Zero();
      for (const RailMatch& match : matches) {
        anchor += current * (0.5 * (local[match.local].left +
                                    local[match.local].right));
      }
      anchor /= static_cast<double>(matches.size());
      const Eigen::Vector2d tangent_reference = referenceTangent(matches);
      const Eigen::Vector2d normal_reference(-tangent_reference.y(),
                                             tangent_reference.x());
      // [roll, pitch, yaw, cross-track translation, z]. Along-track
      // translation is absent by construction, leaving the rail factor with
      // exactly five observable degrees of freedom.
      Eigen::Matrix<double, 5, 5> normal =
          Eigen::Matrix<double, 5, 5>::Zero();
      Eigen::Matrix<double, 5, 1> rhs =
          Eigen::Matrix<double, 5, 1>::Zero();
      for (const RailMatch& match : matches) {
        const RailSample& observed = local[match.local];
        const RailSegmentProjection target_projection = railTarget(match);
        const V3 source_center = current * (0.5 * (observed.left + observed.right));
        Eigen::Vector2d segment_tangent =
            prior_rail_segments_[match.prior_segment].tangent.head<2>();
        if (segment_tangent.norm() > 1.0e-8) {
          segment_tangent.normalize();
          if (segment_tangent.dot(tangent_reference) < 0.0) {
            segment_tangent = -segment_tangent;
          }
          const Eigen::Vector2d segment_normal(-segment_tangent.y(),
                                               segment_tangent.x());
          const double residual = segment_normal.dot(
              target_projection.center.head<2>() - source_center.head<2>());
          const double robust = std::min(
              1.0, config_.rail_huber_delta /
                       std::max(std::abs(residual), 1.0e-6));
          const Eigen::Vector2d lever = (source_center - anchor).head<2>();
          Eigen::Matrix<double, 5, 1> jacobian =
              Eigen::Matrix<double, 5, 1>::Zero();
          jacobian(2) = segment_normal.dot(Eigen::Vector2d(-lever.y(), lever.x()));
          jacobian(3) = segment_normal.dot(normal_reference);
          normal.noalias() += robust * jacobian * jacobian.transpose();
          rhs.noalias() += robust * jacobian * residual;
        }
        for (int side = 0; side < 2; ++side) {
          const V3 source = current * (side == 0 ? observed.left : observed.right);
          const bool target_left = (side == 0) != match.swap_sides;
          const V3& target = target_left ? target_projection.left
                                         : target_projection.right;
          const double residual = target.z() - source.z();
          const double robust = std::min(
              1.0, config_.rail_huber_delta / std::max(std::abs(residual), 1.0e-6));
          Eigen::Matrix<double, 5, 1> jacobian =
              Eigen::Matrix<double, 5, 1>::Zero();
          jacobian(0) = source.y() - anchor.y();
          jacobian(1) = -(source.x() - anchor.x());
          jacobian(4) = 1.0;
          normal.noalias() += robust * jacobian * jacobian.transpose();
          rhs.noalias() += robust * jacobian * residual;
        }
      }
      const Eigen::LDLT<Eigen::Matrix<double, 5, 5>> decomposition(normal);
      if (decomposition.info() != Eigen::Success) return std::nullopt;
      Eigen::Matrix<double, 5, 1> delta = decomposition.solve(rhs);
      if (!delta.allFinite()) return std::nullopt;
      delta(0) = std::clamp(delta(0), -0.01, 0.01);
      delta(1) = std::clamp(delta(1), -0.01, 0.01);
      delta(2) = std::clamp(delta(2), -0.01, 0.01);
      delta(3) = std::clamp(delta(3), -0.50, 0.50);
      delta(4) = std::clamp(delta(4), -max_z_iteration, max_z_iteration);
      const Rotation delta_rotation =
          (Eigen::AngleAxisd(delta(2), V3::UnitZ()) *
           Eigen::AngleAxisd(delta(1), V3::UnitY()) *
           Eigen::AngleAxisd(delta(0), V3::UnitX())).toRotationMatrix();
      Transform left_update = Transform::Identity();
      left_update.linear() = delta_rotation;
      left_update.translation() = anchor - delta_rotation * anchor +
          V3(delta(3) * normal_reference.x(),
             delta(3) * normal_reference.y(), delta(4));
      current = Transform(Eigen::Isometry3d(left_update * current));
      if (delta.head<4>().norm() < 1.0e-6 && std::abs(delta(4)) < 1.0e-5) break;
    }

    matches = associateRails(local, current);
    if (matches.size() < config_.rail_min_matches) return std::nullopt;
    double squared_error = 0.0;
    double planar_squared_error = 0.0;
    double baseline = 0.0;
    V3 first_center = V3::Zero();
    V3 correction_anchor = V3::Zero();
    bool have_first = false;
    const Eigen::Vector2d tangent_reference = referenceTangent(matches);
    const Eigen::Vector2d normal_reference(-tangent_reference.y(),
                                           tangent_reference.x());
    for (const RailMatch& match : matches) {
      const RailSample& observed = local[match.local];
      const RailSegmentProjection target_projection = railTarget(match);
      const V3 source_left = current * observed.left;
      const V3 source_right = current * observed.right;
      const V3& target_left = match.swap_sides ? target_projection.right
                                               : target_projection.left;
      const V3& target_right = match.swap_sides ? target_projection.left
                                                : target_projection.right;
      squared_error += std::pow(source_left.z() - target_left.z(), 2) +
                       std::pow(source_right.z() - target_right.z(), 2);
      Eigen::Vector2d segment_tangent =
          prior_rail_segments_[match.prior_segment].tangent.head<2>();
      if (segment_tangent.norm() > 1.0e-8) {
        segment_tangent.normalize();
        const Eigen::Vector2d segment_normal(-segment_tangent.y(),
                                             segment_tangent.x());
        const V3 source_center = 0.5 * (source_left + source_right);
        planar_squared_error += std::pow(segment_normal.dot(
            source_center.head<2>() - target_projection.center.head<2>()), 2);
      }
      const V3 center = 0.5 * (observed.left + observed.right);
      correction_anchor += center;
      if (!have_first) {
        first_center = center;
        have_first = true;
      }
      baseline = std::max(baseline, (center - first_center).head<2>().norm());
    }
    const double rms = std::sqrt(
        squared_error / static_cast<double>(2 * matches.size()));
    const double planar_rms = std::sqrt(
        planar_squared_error / static_cast<double>(matches.size()));
    correction_anchor /= static_cast<double>(matches.size());
    const double z_step = std::abs((current * correction_anchor).z() -
                                   (*seed * correction_anchor).z());
    const Rotation rotation_step = current.rotation() * seed->rotation().transpose();
    const double tilt_step = Eigen::AngleAxisd(rotation_step).angle() *
                             180.0 / 3.14159265358979323846;
    // The Kalman layer applies the tight tracking innovation gates and knows
    // how far it has travelled since the last accepted rail fix.  Here retain
    // only a broad 10-degree sanity bound so a valid fit can be handed over for
    // re-acquisition after a long rail-coverage gap.
    const double broad_tilt_gate =
        std::max(10.0, config_.rail_max_tilt_step_deg);
    if (baseline < config_.rail_min_baseline || rms > config_.rail_max_rms ||
        tilt_step > broad_tilt_gate) {
      LOG(W, "Rejected rail correction: " << matches.size() << " matches, baseline "
                                             << baseline << " m, vertical RMS " << rms
                                             << " m, step " << z_step << " m / "
                                             << tilt_step << " deg.");
      return std::nullopt;
    }
    const V3 seed_anchor_map = *seed * correction_anchor;
    const V3 fitted_anchor_map = current * correction_anchor;
    const double cross_track_step = std::abs(normal_reference.dot(
        (fitted_anchor_map - seed_anchor_map).head<2>()));
    const double yaw_step = std::abs(wrappedAngle(
        yawOf(current.rotation()) - yawOf(seed->rotation()))) *
        180.0 / 3.14159265358979323846;
    const bool planar_valid =
        planar_rms <= config_.rail_max_planar_rms &&
        cross_track_step <= config_.rail_max_cross_track_step &&
        yaw_step <= config_.rail_max_yaw_step_deg;
    if (!planar_valid) {
      LOG(W, "Rail vertical fit accepted without planar constraints: RMS "
                 << planar_rms << " m, cross-track/yaw step "
                 << cross_track_step << " m / " << yaw_step << " deg.");
    }
    PoleLocalizationCorrection correction;
    correction.T_map_odom = current;
    correction.stamp = stamp;
    correction.odom_distance_m = odom_distance_m;
    correction.correction_anchor_odom = correction_anchor;
    correction.matches = matches.size();
    correction.match_ratio = static_cast<double>(matches.size()) /
                             static_cast<double>(local.size());
    correction.baseline = baseline;
    correction.rms = rms;
    correction.planar_rms = planar_rms;
    correction.rail_tangent_map =
        V3(tangent_reference.x(), tangent_reference.y(), 0.0);
    correction.constrain_planar = false;
    correction.constrain_vertical = true;
    correction.constrain_cross_track = planar_valid;
    correction.constrain_yaw = planar_valid;
    correction.constrain_z = true;
    correction.constrain_roll = config_.rail_update_roll;
    correction.constrain_pitch = config_.rail_update_pitch;
    correction.valid = true;
    return correction;
  }

  void acceptRailCorrection(const PoleLocalizationCorrection& correction) {
    {
      std::lock_guard<std::mutex> lock(correction_mutex_);
      // Keep the extractor's pole-association seed free of rail-planar
      // ambiguity. The full 5-DoF observation is still delivered to the
      // localization graph below; internally we only absorb z/roll/pitch so
      // a parallel track cannot replace the configured/pole x/y/yaw seed.
      Transform planar_reference = correction.T_map_odom;
      if (localization_correction_.valid) {
        planar_reference = localization_correction_.T_map_odom;
      } else if (configured_seed_) {
        planar_reference = *configured_seed_;
      }
      const double rail_yaw = yawOf(correction.T_map_odom.rotation());
      const double reference_yaw = yawOf(planar_reference.rotation());
      const Rotation rail_roll_pitch =
          yawRotation(-rail_yaw) * correction.T_map_odom.rotation();
      const Rotation internal_rotation =
          yawRotation(reference_yaw) * rail_roll_pitch;
      const V3 reference_anchor =
          planar_reference * correction.correction_anchor_odom;
      const V3 rail_anchor =
          correction.T_map_odom * correction.correction_anchor_odom;
      V3 internal_anchor = reference_anchor;
      internal_anchor.z() = rail_anchor.z();
      localization_correction_ = correction;
      localization_correction_.T_map_odom = Transform(
          internal_rotation,
          internal_anchor - internal_rotation * correction.correction_anchor_odom);
      correction_history_.push_back(correction);
    }
    ++rail_localization_accepted_;
    PoleExtractor::CorrectionObserver observer;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      observer = correction_observer_;
    }
    if (observer) observer(correction);
    LOG(I, "Accepted rail correction: " << correction.matches
                                          << " paired samples, vertical RMS "
                                          << correction.rms << " m, planar RMS "
                                          << correction.planar_rms
                                          << " m, planar constraints "
                                          << (correction.constrain_cross_track &&
                                                      correction.constrain_yaw
                                                  ? "enabled"
                                                  : "disabled")
                                          << ".");
  }

  void processRails(uint64_t stamp, double odom_distance_m,
                    const std::vector<RailSample>& rails) {
    if (!config_.rail_enable || rails.empty()) return;
    if (!config_.localization_enable) {
      std::lock_guard<std::mutex> lock(results_mutex_);
      rail_samples_.insert(rail_samples_.end(), rails.begin(), rails.end());
      return;
    }
    if (!rail_localization_ready_) return;
    if (const auto correction = estimateRailCorrection(stamp, odom_distance_m, rails)) {
      acceptRailCorrection(*correction);
    }
  }

  void publishPriorMap(uint64_t stamp) {
    if (prior_map_published_ || prior_poles_.empty()) return;
    sensor_msgs::PointCloud2 message;
    pointCloudToMsg(priorPolesToCloud(prior_poles_), message);
    message.header.stamp.fromNSec(stamp);
    message.header.frame_id = config_.localization_map_frame;
    prior_map_pub_.publish(message);
    prior_map_published_ = true;
  }

  void acceptPoleCorrection(const PoleLocalizationCorrection& correction) {
    {
      std::lock_guard<std::mutex> lock(correction_mutex_);
      localization_correction_ = correction;
      correction_history_.push_back(correction);
    }
    ++localization_accepted_;
    nav_msgs::Odometry message;
    message.header.stamp.fromNSec(correction.stamp);
    message.header.frame_id = config_.localization_map_frame;
    message.child_frame_id = map_frame_;
    transformToMsg(correction.T_map_odom, message.pose.pose);
    correction_pub_.publish(message);

    PoleExtractor::CorrectionObserver observer;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      observer = correction_observer_;
    }
    if (observer) observer(correction);
    const V3 translation = correction.T_map_odom.translation();
    LOG(I, "Accepted pole correction: " << correction.matches << " matches ("
                                          << correction.match_ratio * 100.0 << "%), RMS "
                                          << correction.rms << " m, T_map_odom xy/yaw = ("
                                          << translation.x() << ", " << translation.y() << ", "
                                          << yawOf(correction.T_map_odom.rotation()) * 180.0 /
                                                 3.14159265358979323846
                                          << " deg).");
  }

  void localizePoles(uint64_t stamp, double odom_distance_m,
                     const std::vector<Detection>& detections) {
    if (!config_.localization_enable || !localization_ready_) return;
    recent_detection_frames_.push_back(detections);
    while (recent_detection_frames_.size() > config_.localization_window_keyframes) {
      recent_detection_frames_.pop_front();
    }
    publishPriorMap(stamp);
    // An empty keyframe adds no new evidence. Keeping old observations in the
    // rolling window is useful for the next non-empty keyframe, but must not
    // generate repeated corrections or make a pole-only filter overconfident.
    if (detections.empty()) return;
    if (const auto correction = estimatePoleCorrection(stamp, odom_distance_m)) {
      acceptPoleCorrection(*correction);
    }
  }

  void queueCurrentKeyframeLocked() {
    ExtractionTask task;
    task.index = next_keyframe_index_++;
    task.frames = current_frames_;
    const size_t overlap =
        std::min(config_.keyframe_overlap_frames, current_frames_.size());
    if (overlap == 0) {
      current_frames_.clear();
    } else {
      current_frames_.erase(current_frames_.begin(), current_frames_.end() - overlap);
    }
    distance_since_keyframe_ = 0.0;
    new_frames_since_keyframe_ = 0;
    if (!offline_ && queue_.size() >= config_.max_queue) {
      queue_.pop_front();
      ++dropped_keyframes_;
    }
    queue_.push_back(std::move(task));
  }

  void workerLoop() {
    while (true) {
      ExtractionTask task;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
        if (queue_.empty() && stop_) return;
        task = std::move(queue_.front());
        queue_.pop_front();
      }
      queue_cv_.notify_all();

      const auto start = std::chrono::steady_clock::now();
      const auto guided_poles = guidedRoiPoles(task);
      ExtractionResult result = extract(task, config_, guided_poles);
      const std::vector<RailSample> rails =
          extractRailVector(task, result.downsampled, config_);
      PoleCloud rail_cloud = extractRailPoints(task, rails, config_);
      integrateDetections(result.detections);
      const auto& anchor_frame = task.frames.back();
      PoleExtractor::KeyframeObserver keyframe_observer;
      {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        keyframe_observer = keyframe_observer_;
      }
      // Register X(k) before either observation callback can add a factor for
      // this timestamp.
      if (keyframe_observer) {
        keyframe_observer(anchor_frame->stamp, anchor_frame->pose,
                          anchor_frame->odom_distance_m);
      }
      localizePoles(anchor_frame->stamp, anchor_frame->odom_distance_m,
                    result.detections);
      processRails(anchor_frame->stamp, anchor_frame->odom_distance_m, rails);
      publish(anchor_frame->stamp, result);
      publishCloud(railsToCloud(rails), anchor_frame->stamp, rail_points_pub_);
      if (config_.localization_enable && config_.rail_enable &&
          !rail_prior_map_published_) {
        publishCloud(railsToCloud(prior_rails_), anchor_frame->stamp,
                     rail_map_pub_, config_.localization_map_frame);
        rail_prior_map_published_ = true;
      } else if (!config_.localization_enable && config_.rail_enable) {
        std::vector<RailSample> rail_snapshot;
        {
          std::lock_guard<std::mutex> lock(results_mutex_);
          rail_snapshot = rail_samples_;
        }
        publishCloud(railsToCloud(rail_snapshot), anchor_frame->stamp, rail_map_pub_);
      }
      {
        std::lock_guard<std::mutex> lock(results_mutex_);
        if (config_.save_keyframe_clouds) {
          pole_points_.insert(pole_points_.end(), result.refined.begin(), result.refined.end());
        }
        rail_points_.insert(rail_points_.end(), rail_cloud.begin(), rail_cloud.end());
      }
      ++processed_keyframes_;
      detection_count_.fetch_add(result.detections.size());
      const double seconds =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      LOG(I, "Pole keyframe " << task.index << ": " << task.frames.size() << " scans, "
                               << result.downsampled.size() << " ROI/downsampled points, "
                               << result.candidates.size() << " candidates, "
                               << result.detections.size() << " poles, " << rails.size()
                               << " rail samples, " << rail_cloud.size() << " measured rail points in " << std::fixed
                               << std::setprecision(3) << seconds << " s.");
    }
  }

  void integrateDetections(const std::vector<Detection>& detections) {
    std::lock_guard<std::mutex> lock(results_mutex_);
    detections_.insert(detections_.end(), detections.begin(), detections.end());
    const double merge_distance_sq =
        config_.landmark_merge_radius * config_.landmark_merge_radius;
    constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
    const double minimum_axis_dot =
        std::cos(config_.landmark_max_axis_angle_deg * kDegreesToRadians);
    std::unordered_set<size_t> assigned_landmarks;
    for (const Detection& detection : detections) {
      Landmark* match = nullptr;
      double best_distance_sq = merge_distance_sq;
      for (Landmark& landmark : landmarks_) {
        if (assigned_landmarks.find(landmark.id) != assigned_landmarks.end()) continue;
        const double comparison_z =
            0.5 * (landmark.axis_point.z() + detection.axis_point.z());
        const V3 landmark_point =
            pointOnAxisAtZ(landmark.axis_point, landmark.axis, comparison_z);
        const V3 detection_point =
            pointOnAxisAtZ(detection.axis_point, detection.axis, comparison_z);
        const double distance_sq =
            (landmark_point - detection_point).head<2>().squaredNorm();
        const double axis_dot = std::abs(landmark.axis.dot(detection.axis));
        const double radius_difference = std::abs(landmark.radius - detection.radius);
        if (distance_sq <= best_distance_sq && axis_dot >= minimum_axis_dot &&
            radius_difference <= config_.landmark_max_radius_difference) {
          match = &landmark;
          best_distance_sq = distance_sq;
        }
      }
      if (match == nullptr) {
        Landmark landmark;
        landmark.id = next_landmark_id_++;
        landmark.center = detection.center;
        landmark.axis = detection.axis;
        landmark.axis_point = detection.axis_point;
        landmark.base_z = detection.base_z;
        landmark.weight_sum = detection.fusion_weight;
        landmark.radius = detection.radius;
        landmark.fit_rms = detection.fit_rms;
        landmark.axial_min = detection.axial_min;
        landmark.axial_max = detection.axial_max;
        landmark.observations = 1;
        landmark.total_points = detection.points;
        landmarks_.push_back(landmark);
        assigned_landmarks.insert(landmarks_.back().id);
      } else {
        const double old_weight = match->weight_sum;
        const double detection_weight = detection.fusion_weight;
        const double total_weight = old_weight + detection_weight;
        const V3 old_lower = match->center + match->axial_min * match->axis;
        const V3 old_upper = match->center + match->axial_max * match->axis;
        const V3 detection_lower =
            detection.center + detection.axial_min * detection.axis;
        const V3 detection_upper =
            detection.center + detection.axial_max * detection.axis;
        V3 aligned_axis = detection.axis;
        if (match->axis.dot(aligned_axis) < 0.0) aligned_axis = -aligned_axis;
        const double reference_z = match->axis_point.z();
        const V3 observed_axis_point =
            pointOnAxisAtZ(detection.axis_point, aligned_axis, reference_z);
        match->axis_point =
            (old_weight * match->axis_point + detection_weight * observed_axis_point) /
            total_weight;
        match->axis =
            (old_weight * match->axis + detection_weight * aligned_axis).normalized();
        match->base_z =
            (old_weight * match->base_z + detection_weight * detection.base_z) /
            total_weight;
        match->center = pointOnAxisAtZ(match->axis_point, match->axis, match->base_z);
        match->radius =
            (old_weight * match->radius + detection_weight * detection.radius) /
            total_weight;
        match->fit_rms =
            (old_weight * match->fit_rms + detection_weight * detection.fit_rms) /
            total_weight;
        const V3 fused_lower =
            (old_weight * old_lower + detection_weight * detection_lower) / total_weight;
        const V3 fused_upper =
            (old_weight * old_upper + detection_weight * detection_upper) / total_weight;
        match->axial_min = (fused_lower - match->center).dot(match->axis);
        match->axial_max = (fused_upper - match->center).dot(match->axis);
        match->weight_sum = total_weight;
        ++match->observations;
        match->total_points += detection.points;
        assigned_landmarks.insert(match->id);
      }
    }
  }

  void publishCloud(const Pointcloud& cloud, uint64_t stamp,
                    const ros::Publisher& publisher,
                    const std::string& frame = std::string()) {
    if (!publisher) return;
    sensor_msgs::PointCloud2 message;
    pointCloudToMsg(cloud, message);
    message.header.stamp.fromNSec(stamp);
    message.header.frame_id = frame.empty() ? map_frame_ : frame;
    publisher.publish(message);
  }

  void publishMarkers(uint64_t stamp, const std::vector<Landmark>& landmarks) {
    if (!markers_pub_) return;
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker clear;
    clear.header.frame_id = map_frame_;
    clear.header.stamp.fromNSec(stamp);
    clear.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(clear);

    auto setCommon = [this, stamp](visualization_msgs::Marker& marker,
                                  const std::string& marker_namespace, int id, int type) {
      marker.header.frame_id = map_frame_;
      marker.header.stamp.fromNSec(stamp);
      marker.ns = marker_namespace;
      marker.id = id;
      marker.type = type;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
    };
    auto toPoint = [](const V3& point) {
      geometry_msgs::Point output;
      output.x = point.x();
      output.y = point.y();
      output.z = point.z();
      return output;
    };

    for (size_t index = 0; index < landmarks.size(); ++index) {
      const Landmark& landmark = landmarks[index];
      const int id = static_cast<int>(landmark.id);
      const double height = std::max(0.20, landmark.axial_max - landmark.axial_min);
      const V3 lower = landmark.center + landmark.axial_min * landmark.axis;
      const V3 upper = landmark.center + landmark.axial_max * landmark.axis;
      const V3 middle = 0.5 * (lower + upper);

      visualization_msgs::Marker cylinder;
      setCommon(cylinder, "pole_cylinder", id, visualization_msgs::Marker::CYLINDER);
      cylinder.pose.position = toPoint(middle);
      Eigen::Quaterniond orientation;
      orientation.setFromTwoVectors(V3::UnitZ(), landmark.axis);
      cylinder.pose.orientation.x = orientation.x();
      cylinder.pose.orientation.y = orientation.y();
      cylinder.pose.orientation.z = orientation.z();
      cylinder.pose.orientation.w = orientation.w();
      cylinder.scale.x = std::max(0.10, 2.0 * landmark.radius);
      cylinder.scale.y = cylinder.scale.x;
      cylinder.scale.z = height;
      cylinder.color.r = 0.10F;
      cylinder.color.g = 0.95F;
      cylinder.color.b = 0.25F;
      cylinder.color.a = 0.28F;
      array.markers.push_back(cylinder);

      visualization_msgs::Marker axis;
      setCommon(axis, "pole_axis", id, visualization_msgs::Marker::ARROW);
      axis.points.push_back(toPoint(lower));
      axis.points.push_back(toPoint(upper));
      axis.scale.x = 0.05;
      axis.scale.y = 0.13;
      axis.scale.z = 0.22;
      axis.color.r = 1.0F;
      axis.color.g = 0.82F;
      axis.color.b = 0.05F;
      axis.color.a = 1.0F;
      array.markers.push_back(axis);

      visualization_msgs::Marker base;
      setCommon(base, "pole_base", id, visualization_msgs::Marker::SPHERE);
      base.pose.position = toPoint(landmark.center);
      base.scale.x = 0.35;
      base.scale.y = 0.35;
      base.scale.z = 0.35;
      base.color.r = 1.0F;
      base.color.g = 0.20F;
      base.color.b = 0.05F;
      base.color.a = 1.0F;
      array.markers.push_back(base);

      visualization_msgs::Marker text_marker;
      setCommon(text_marker, "pole_id", id, visualization_msgs::Marker::TEXT_VIEW_FACING);
      text_marker.pose.position = toPoint(upper + V3(0.0, 0.0, 0.55));
      text_marker.scale.z = 0.48;
      text_marker.color.r = 1.0F;
      text_marker.color.g = 1.0F;
      text_marker.color.b = 1.0F;
      text_marker.color.a = 1.0F;
      std::ostringstream label;
      label << "Pole " << landmark.id << "  obs=" << landmark.observations << "\nr="
            << std::fixed << std::setprecision(2) << landmark.radius << " m  rms="
            << landmark.fit_rms;
      text_marker.text = label.str();
      array.markers.push_back(text_marker);
    }
    markers_pub_.publish(array);
  }

  void publish(uint64_t stamp, const ExtractionResult& result) {
    publishCloud(centersToCloud(result.detections), stamp, centers_pub_);
    publishCloud(toBievrCloud(result.refined), stamp, points_pub_);
    std::vector<Landmark> landmarks;
    {
      std::lock_guard<std::mutex> lock(results_mutex_);
      landmarks = landmarks_;
    }
    publishCloud(landmarksToCloud(landmarks), stamp, map_pub_);
    publishMarkers(stamp, landmarks);
  }

  PoleExtractionConfig config_;
  std::string map_frame_;
  bool offline_ = false;
  ros::Publisher centers_pub_;
  ros::Publisher points_pub_;
  ros::Publisher map_pub_;
  ros::Publisher prior_map_pub_;
  ros::Publisher correction_pub_;
  ros::Publisher markers_pub_;
  ros::Publisher rail_points_pub_;
  ros::Publisher rail_map_pub_;

  std::atomic<bool> accepting_{true};
  bool stop_ = false;
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::vector<ExtractionFramePtr> current_frames_;
  std::deque<ExtractionTask> queue_;
  size_t next_keyframe_index_ = 0;
  size_t new_frames_since_keyframe_ = 0;
  double distance_since_keyframe_ = 0.0;
  double odom_distance_m_ = 0.0;
  V3 last_frame_position_ = V3::Zero();
  bool has_last_frame_position_ = false;
  std::thread worker_;

  mutable std::mutex results_mutex_;
  PoleCloud pole_points_;
  PoleCloud rail_points_;
  std::vector<Detection> detections_;
  std::vector<Landmark> landmarks_;
  std::vector<RailSample> rail_samples_;
  size_t next_landmark_id_ = 0;

  bool localization_ready_ = false;
  bool mapping_trajectory_ready_ = true;
  std::vector<ReferencePose> mapping_trajectory_;
  bool prior_map_published_ = false;
  std::vector<PriorPole> prior_poles_;
  bool rail_localization_ready_ = true;
  bool rail_prior_map_published_ = false;
  std::vector<RailSample> prior_rails_;
  std::vector<RailSegment> prior_rail_segments_;
  std::optional<Transform> configured_seed_;
  std::deque<std::vector<Detection>> recent_detection_frames_;
  mutable std::mutex correction_mutex_;
  PoleLocalizationCorrection localization_correction_;
  std::vector<PoleLocalizationCorrection> correction_history_;
  mutable std::mutex callback_mutex_;
  PoleExtractor::CorrectionProvider correction_provider_;
  PoleExtractor::CorrectionObserver correction_observer_;
  PoleExtractor::KeyframeObserver keyframe_observer_;

  std::atomic<size_t> received_frames_{0};
  std::atomic<size_t> processed_keyframes_{0};
  std::atomic<size_t> dropped_keyframes_{0};
  std::atomic<size_t> rejected_trajectory_frames_{0};
  std::atomic<size_t> detection_count_{0};
  std::atomic<size_t> localization_attempts_{0};
  std::atomic<size_t> localization_accepted_{0};
  std::atomic<size_t> rail_localization_attempts_{0};
  std::atomic<size_t> rail_localization_accepted_{0};
};

PoleExtractor::PoleExtractor(ros::NodeHandle& nh, const std::shared_ptr<Pipeline>& pipeline,
                             PoleExtractionConfig config, const std::string& topic_prefix)
    : impl_(new Impl(&nh, pipeline, std::move(config), topic_prefix)) {}

PoleExtractor::PoleExtractor(PoleExtractionConfig config, const std::string& map_frame)
    : impl_(new Impl(nullptr, nullptr, std::move(config), "", map_frame)) {}

void PoleExtractor::addFrame(uint64_t stamp, const Transform& pose, const Pointcloud& cloud) {
  impl_->addFrame(stamp, pose, cloud);
}

PoleExtractor::~PoleExtractor() = default;

void PoleExtractor::finish() { impl_->finish(); }

bool PoleExtractor::saveResults(std::string* message) const { return impl_->saveResults(message); }

PoleExtractionStats PoleExtractor::stats() const { return impl_->stats(); }

void PoleExtractor::setCorrectionProvider(CorrectionProvider provider) {
  impl_->setCorrectionProvider(std::move(provider));
}

void PoleExtractor::setCorrectionObserver(CorrectionObserver observer) {
  impl_->setCorrectionObserver(std::move(observer));
}

void PoleExtractor::setKeyframeObserver(KeyframeObserver observer) {
  impl_->setKeyframeObserver(std::move(observer));
}

PoleLocalizationCorrection PoleExtractor::localizationCorrection() const {
  return impl_->localizationCorrection();
}

bool PoleExtractor::localizationReady() const { return impl_->localizationReady(); }

}  // namespace bievr
