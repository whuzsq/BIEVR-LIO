#include <bievr_lio/common.h>
#include <bievr_lio/log++.h>
#include <bievr_pgo/gnss_optimizer.h>
#include <bievr_lio/scan_archive.h>
#include <bievr_lio/utils.h>
#include <ros/ros.h>
#include <ros/master.h>
#include "bievr_lio_ros/pole_extractor.h"
#include <yaml-cpp/yaml.h>
#include <chrono>

#include <Eigen/Geometry>

#include <sys/wait.h>
#include <unistd.h>
#include <cerrno>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

struct Options {
  std::string scans;
  std::string lio_trajectory;
  std::string gnss_enu;
  std::string output_directory = "Log/gnss_optimization";
  std::string output_map;
  std::string poles_directory;
  std::string pole_params_file;
  bool skip_poles = false;
  double map_voxel_resolution_m = 0.15;
  double map_min_range_m = 1.0;
  double map_max_range_m = 100.0;
};

struct VoxelKey {
  int x = 0;
  int y = 0;
  int z = 0;
  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  size_t operator()(const VoxelKey& key) const {
    size_t hash = std::hash<int>{}(key.x);
    hash ^= std::hash<int>{}(key.y) + 0x9e3779b9u + (hash << 6) + (hash >> 2);
    hash ^= std::hash<int>{}(key.z) + 0x9e3779b9u + (hash << 6) + (hash >> 2);
    return hash;
  }
};

bool commandLineValue(const std::vector<std::string>& args, const std::string& key,
                      std::string& value) {
  for (size_t index = 0; index < args.size(); ++index) {
    if (args[index] == key && index + 1 < args.size()) {
      value = args[index + 1];
      return true;
    }
    const std::string prefix = key + "=";
    if (args[index].rfind(prefix, 0) == 0) {
      value = args[index].substr(prefix.size());
      return true;
    }
  }
  return false;
}

uint64_t secondsToNanoseconds(double seconds) {
  if (!std::isfinite(seconds) || seconds < 0.0) return 0;
  return static_cast<uint64_t>(std::llround(seconds * 1.0e9));
}

bool readTum(const std::string& path,
             std::vector<bievr::GnssTrajectoryOptimizer::StampedPose>& poses,
             std::string& message) {
  std::ifstream input(path);
  if (!input) {
    message = "Cannot open LIO TUM trajectory '" + path + "'.";
    return false;
  }
  poses.clear();
  std::string line;
  size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') continue;
    std::istringstream stream(line);
    double stamp = 0.0;
    bievr::V3 translation;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    if (!(stream >> stamp >> translation.x() >> translation.y() >> translation.z() >>
          qx >> qy >> qz >> qw)) {
      message = "Malformed TUM row " + std::to_string(line_number) + " in '" + path + "'.";
      return false;
    }
    bievr::Quaternion quaternion(qw, qx, qy, qz);
    if (!translation.allFinite() || !quaternion.coeffs().allFinite() ||
        quaternion.norm() < 1.0e-9) continue;
    quaternion.normalize();
    poses.push_back({secondsToNanoseconds(stamp), bievr::Transform(quaternion, translation)});
  }
  std::sort(poses.begin(), poses.end(), [](const auto& a, const auto& b) {
    return a.stamp < b.stamp;
  });
  poses.erase(std::unique(poses.begin(), poses.end(), [](const auto& a, const auto& b) {
                return a.stamp == b.stamp;
              }),
              poses.end());
  if (poses.size() < 2) {
    message = "LIO TUM trajectory contains fewer than two valid poses.";
    return false;
  }
  return true;
}

// Recorder format: metadata plus timestamp E N U varE varN varU.
bool readGnssEnu(
    const std::string& path,
    std::vector<bievr::GnssTrajectoryOptimizer::PositionMeasurement>& measurements,
    bievr::V3& origin_lla, std::string& message) {
  std::ifstream input(path);
  if (!input) {
    message = "Cannot open GNSS ENU file '" + path + "'.";
    return false;
  }
  measurements.clear();
  bool have_origin = false;
  std::string line;
  size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    std::istringstream stream(line);
    stream >> std::ws;
    if (stream.eof()) continue;
    if (stream.peek() == '#') {
      std::string marker, key;
      stream >> marker >> key;
      if (key != "origin_lla") continue;
      if (!(stream >> origin_lla.x() >> origin_lla.y() >> origin_lla.z()) ||
          !origin_lla.allFinite() || std::abs(origin_lla.x()) > 90.0 ||
          std::abs(origin_lla.y()) > 180.0 || have_origin) {
        message = "Invalid or repeated origin_lla in '" + path + "'.";
        return false;
      }
      have_origin = true;
      continue;
    }
    std::string timestamp, extra;
    ros::Time stamp;
    bievr::V3 position, variance;
    bool valid_stamp = false;
    if (stream >> timestamp >> position.x() >> position.y() >> position.z() >>
        variance.x() >> variance.y() >> variance.z()) {
      // ROS time parses seconds/nanoseconds separately to retain nanosecond precision.
      const auto dot = timestamp.find('.');
      const std::string seconds = timestamp.substr(0, dot);
      std::string fraction = dot == std::string::npos ? "" : timestamp.substr(dot + 1);
      if (!seconds.empty() && seconds.find_first_not_of("0123456789") == std::string::npos &&
          fraction.size() <= 9 && fraction.find_first_not_of("0123456789") == std::string::npos) {
        fraction.append(9 - fraction.size(), '0');
        try {
          const auto sec = std::stoull(seconds);
          if (sec <= std::numeric_limits<uint32_t>::max()) {
            stamp = ros::Time(static_cast<uint32_t>(sec), std::stoul(fraction));
            valid_stamp = true;
          }
        } catch (const std::exception&) {}
      }
    }
    if (!valid_stamp || !position.allFinite() || !variance.allFinite() ||
        (variance.array() < 0.0).any() || (stream >> extra)) {
      message = "Malformed GNSS ENU row " + std::to_string(line_number) + " in '" + path + "'.";
      return false;
    }
    measurements.push_back({stamp.toNSec(), position, variance});
  }
  if (input.bad() || !have_origin || measurements.size() < 3) {
    message = "GNSS ENU file requires # origin_lla lat lon alt and at least three valid measurements.";
    return false;
  }
  return true;
}

bool interpolatePose(const std::vector<bievr::GnssTrajectoryOptimizer::StampedPose>& poses,
                     uint64_t stamp, bievr::Transform& pose) {
  if (poses.size() < 2) return false;
  const auto upper = std::lower_bound(
      poses.begin(), poses.end(), stamp,
      [](const auto& value, uint64_t time) { return value.stamp < time; });
  if (upper != poses.end() && upper->stamp == stamp) {
    pose = upper->pose;
    return true;
  }
  // Existing TUM logs serialize epoch seconds through double (sub-microsecond rounding).
  if (upper == poses.begin()) {
    if (poses.front().stamp - stamp > 1000) return false;
    pose = poses.front().pose;
    return true;
  }
  if (upper == poses.end()) {
    if (stamp - poses.back().stamp > 1000) return false;
    pose = poses.back().pose;
    return true;
  }
  const auto& second = *upper;
  const auto& first = *(upper - 1);
  if (second.stamp <= first.stamp) return false;
  const double alpha = static_cast<double>(stamp - first.stamp) /
                       static_cast<double>(second.stamp - first.stamp);
  const bievr::Quaternion q = bievr::Quaternion(first.pose.rotation()).slerp(
      alpha, bievr::Quaternion(second.pose.rotation()));
  const bievr::V3 translation =
      (1.0 - alpha) * first.pose.translation() + alpha * second.pose.translation();
  pose = bievr::Transform(q.normalized(), translation);
  return true;
}

bool saveOptimizedPointcloudMap(
    const Options& options,
    const std::vector<bievr::GnssTrajectoryOptimizer::StampedPose>& optimized,
    std::string& message) {
  if (options.output_map.empty() || !std::isfinite(options.map_voxel_resolution_m) ||
      options.map_voxel_resolution_m <= 0.0 || !std::isfinite(options.map_min_range_m) ||
      options.map_min_range_m < 0.0 || !std::isfinite(options.map_max_range_m) ||
      options.map_max_range_m <= options.map_min_range_m) {
    message = "Invalid map output path, voxel resolution, or range filter.";
    return false;
  }
  std::unordered_map<VoxelKey, bievr::IntensityPoint, VoxelKeyHash> voxels;
  size_t scans = 0, skipped_scans = 0, input_points = 0;
  const double inverse_resolution = 1.0 / options.map_voxel_resolution_m;
  try {
    bievr::ScanArchiveReader archive(options.scans);
    uint64_t stamp;
    bievr::IntensityPointcloud cloud;
    while (archive.next(stamp, cloud)) {
      ++scans;
      bievr::Transform pose;
      if (!interpolatePose(optimized, stamp, pose)) {
        ++skipped_scans;
        continue;
      }
      for (size_t i = 0; i < cloud.size(); ++i) {
        const bievr::V3 body = cloud[i].head<3>();
        const double range = body.norm();
        if (!body.allFinite() || range < options.map_min_range_m ||
            range > options.map_max_range_m) continue;
        const bievr::V3 world = pose * body;
        const bievr::V3 cell = (world * inverse_resolution).array().floor();
        if (!cell.allFinite() || (cell.array() < std::numeric_limits<int>::min()).any() ||
            (cell.array() > std::numeric_limits<int>::max()).any()) {
          throw std::runtime_error("Map voxel index out of range.");
        }
        const VoxelKey key{static_cast<int>(cell.x()), static_cast<int>(cell.y()),
                           static_cast<int>(cell.z())};
        ++input_points;
        if (voxels.find(key) == voxels.end()) {
          bievr::IntensityPoint point;
          point << world, cloud[i][3];
          voxels.emplace(key, point);
        }
      }
    }
  } catch (const std::exception& error) {
    message = error.what();
    return false;
  }
  if (voxels.empty()) {
    message = "No archived points overlap the optimized trajectory and range filter.";
    return false;
  }
  bievr::IntensityPointcloud map;
  map.resize(voxels.size());
  size_t index = 0;
  for (const auto& entry : voxels) map[index++] = entry.second;
  if (!bievr::savePointcloudPCD(map, options.output_map)) {
    message = "Failed to save optimized pointcloud map '" + options.output_map + "'.";
    return false;
  }
  std::ostringstream stream;
  stream << "Saved optimized map to '" << options.output_map << "' (" << map.size()
         << " voxels from " << scans << " archived scans, " << input_points
         << " input points, " << skipped_scans << " scans outside trajectory).";
  message = stream.str();
  return true;
}

// Rebuild from body-frame measurements, avoiding timestamp guesses for merged
// PCDs and avoiding applying a second correction to already optimized results.
bool rebuildPoles(const Options& options,
                  const std::vector<bievr::GnssTrajectoryOptimizer::StampedPose>& optimized,
                  std::string& message) {
  namespace fs = std::filesystem;
  const fs::path directory(options.poles_directory);
  if (options.skip_poles) {
    message = "Skipping pole/rail rebuilding by request.";
    return true;
  }
  if (!fs::is_regular_file(directory / "landmarks.txt")) {
    message = "No existing pole map in '" + directory.string() + "'; skipping pole/rail rebuilding.";
    return true;
  }
  if (!fs::is_regular_file(options.scans)) {
    message = "Existing pole map requires scan archive '" + options.scans +
        "' for GNSS rebuilding. Use --skip-poles to leave it unchanged.";
    return false;
  }
  const std::string suffix = std::to_string(
      std::chrono::system_clock::now().time_since_epoch().count()) + "_" + std::to_string(getpid());
  const fs::path staging = directory.string() + ".gnss_pending_" + suffix;
  const fs::path backup = directory.string() + ".before_gnss_" + suffix;
  bool moved_original = false;
  try {
    if (!fs::create_directory(staging)) throw std::runtime_error("Cannot create pole staging directory.");
    fs::path params = options.pole_params_file;
    if (params.empty()) {
      params = directory / "extraction_config.yaml";
      if (!fs::is_regular_file(params)) {
        params = fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path() /
                 "config/params.yaml";
        LOG(W, "Existing pole map has no extraction_config.yaml; using '" << params.string()
               << "'. Use --pole_params_file to select its original extraction settings.");
      }
    }
    // Disable localization and any prior mapping trajectory before validation.
    const fs::path overlay_path = staging / "rebuild_overlay.yaml";
    YAML::Node overlay;
    overlay["pole_extraction"]["enable"] = true;
    overlay["pole_extraction"]["localization_enable"] = false;
    overlay["pole_extraction"]["mapping_trajectory_path"] = "";
    overlay["pole_extraction"]["output_directory"] = staging.string();
    overlay["rail_extraction"]["enable"] = fs::is_regular_file(directory / "rail_vector_map.txt");
    if (fs::is_regular_file(directory / "poles.pcd"))
      overlay["pole_extraction"]["save_keyframe_clouds"] = true;
    {
      std::ofstream file(overlay_path);
      file << overlay << "\n";
      file.close();
      if (!file) throw std::runtime_error("Cannot write pole rebuild settings.");
    }
    bievr::PoleExtractionConfig config;
    if (!bievr::loadPoleExtractionConfig({params.string(), overlay_path.string()}, config))
      throw std::runtime_error("Invalid pole extraction settings.");
    bievr::PoleExtractor extractor(config, "map");
    bievr::ScanArchiveReader archive(options.scans);
    uint64_t stamp;
    bievr::IntensityPointcloud scan;
    size_t accepted = 0, skipped = 0;
    while (archive.next(stamp, scan)) {
      bievr::Transform pose;
      if (!interpolatePose(optimized, stamp, pose)) { ++skipped; continue; }
      bievr::Pointcloud points;
      points.resize(scan.size());
      for (size_t i = 0; i < scan.size(); ++i) points.data().col(i) = scan[i].head<3>();
      extractor.addFrame(stamp, pose, points);
      ++accepted;
    }
    extractor.finish();
    const auto stats = extractor.stats();
    if (accepted == 0 || stats.processed_keyframes == 0 || stats.dropped_keyframes != 0)
      throw std::runtime_error("Pole rebuilding produced no keyframes or dropped input data.");
    auto has_records = [](const fs::path& path) {
      std::ifstream input(path);
      std::string line;
      std::getline(input, line); // table header
      while (std::getline(input, line)) {
        if (line.find_first_not_of(" \t\r") != std::string::npos) return true;
      }
      return false;
    };
    if ((stats.landmarks == 0 && has_records(directory / "landmarks.txt")) ||
        (config.rail_enable && stats.rail_samples == 0 &&
         has_records(directory / "rail_vector_map.txt"))) {
      throw std::runtime_error("Rebuilding would erase a nonempty pole/rail map; check extraction parameters.");
    }
    if (!extractor.saveResults(&message)) throw std::runtime_error(message);
    fs::remove(overlay_path);
    // Persist the final path rather than the temporary transaction directory.
    const fs::path snapshot_path = staging / "extraction_config.yaml";
    YAML::Node snapshot = YAML::LoadFile(snapshot_path.string());
    snapshot["pole_extraction"]["output_directory"] = directory.string();
    {
      std::ofstream file(snapshot_path);
      file << snapshot << "\n";
      file.close();
      if (!file) throw std::runtime_error("Cannot finalize extraction configuration.");
    }
    {
      YAML::Node provenance;
      provenance["coordinate_frame"] = "map_enu";
      provenance["method"] = "reextract_from_body_scans";
      provenance["optimized_trajectory"] = (fs::path(options.output_directory) / "gnss_optimized_tum.txt").string();
      provenance["scans"] = options.scans;
      provenance["accepted_scans"] = accepted;
      provenance["skipped_scans"] = skipped;
      provenance["original_backup"] = backup.string();
      std::ofstream file(staging / "gnss_optimization.yaml");
      file << provenance << "\n";
      file.close();
      if (!file) throw std::runtime_error("Cannot write pole optimization provenance.");
    }
    fs::rename(directory, backup);
    moved_original = true;
    fs::rename(staging, directory);
    moved_original = false;
    message = "Rebuilt GNSS-optimized poles/rails in '" + directory.string() +
        "' from " + std::to_string(accepted) + " scans (" + std::to_string(skipped) +
        " outside trajectory). Original results backed up to '" + backup.string() + "'.";
    return true;
  } catch (const std::exception& error) {
    std::error_code cleanup_error;
    if (moved_original) fs::rename(backup, directory, cleanup_error);
    message = "Failed to rebuild pole/rail map: " + std::string(error.what());
    if (cleanup_error) message += ". Original results remain at '" + backup.string() + "'.";
    fs::remove_all(staging, cleanup_error);
    return false;
  }
}

bool writeOutputs(
    const Options& options, const bievr::V3& origin_lla,
    const std::vector<bievr::GnssTrajectoryOptimizer::StampedPose>& raw,
    const std::vector<bievr::GnssTrajectoryOptimizer::StampedPose>& optimized,
    const bievr::GnssTrajectoryOptimizer::Config& config,
    const bievr::GnssTrajectoryOptimizer::Stats& stats, std::string& message) {
  namespace fs = std::filesystem;
  std::error_code error;
  fs::create_directories(options.output_directory, error);
  if (error) {
    message = "Cannot create output directory: " + error.message();
    return false;
  }
  const fs::path directory(options.output_directory);
  std::ofstream tum(directory / "gnss_optimized_tum.txt");
  std::ofstream obj(directory / "trajectory.obj");
  std::ofstream aligned(directory / "lio_aligned_enu_tum.txt");
  std::ofstream summary(directory / "summary.yaml");
  std::ofstream seed(directory / "initial_correction.yaml");
  if (!tum || !obj || !aligned || !summary || !seed) {
    message = "Cannot open one or more GNSS optimization output files.";
    return false;
  }
  obj << std::setprecision(15);
  tum << std::fixed << std::setprecision(9);
  for (const auto& stamped : optimized) {
    const bievr::V3& p = stamped.pose.translation();
    obj << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";
    const bievr::Quaternion q(stamped.pose.rotation());
    tum << static_cast<double>(stamped.stamp) * 1.0e-9 << " " << p.x() << " " << p.y()
        << " " << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
        << q.w() << "\n";
  }
  if (optimized.size() >= 2) {
    obj << "l";
    for (size_t i = 0; i < optimized.size(); ++i) obj << " " << i + 1;
    obj << "\n";
  }
  obj.close();
  aligned << std::fixed << std::setprecision(9);
  for (const auto& stamped : raw) {
    const bievr::Transform pose(stats.initial_T_enu_odom * stamped.pose);
    const bievr::V3 p = pose.translation();
    const bievr::Quaternion q(pose.rotation());
    aligned << static_cast<double>(stamped.stamp) * 1e-9 << " " << p.x() << " " << p.y()
            << " " << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  const bievr::Transform T_enu_odom =
      optimized.front().pose * raw.front().pose.inverse();
  const bievr::V3 translation = T_enu_odom.translation();
  const bievr::Quaternion correction_orientation(T_enu_odom.rotation());
  const double yaw_deg =
      std::atan2(T_enu_odom.rotation()(1, 0), T_enu_odom.rotation()(0, 0)) * 180.0 /
      3.14159265358979323846;
  seed << std::fixed << std::setprecision(9)
       << "# Pass this file as the last --params_file in the GNSS-free run.\n"
       << "pole_extraction:\n"
       << "  localization_use_initial_guess: true\n"
       << "  localization_initial_translation: [" << translation.x() << ", "
       << translation.y() << ", " << translation.z() << "]\n"
       << "  localization_initial_orientation_xyzw: [" << correction_orientation.x()
       << ", " << correction_orientation.y() << ", " << correction_orientation.z()
       << ", " << correction_orientation.w() << "]\n"
       << "  localization_initial_yaw_deg: " << yaw_deg << "\n";
  summary << std::fixed << std::setprecision(9)
          << "gnss_enu: \"" << fs::absolute(options.gnss_enu).string() << "\"\n"
          << "scans: \"" << options.scans << "\"\n"
          << "optimized_map: \"" << options.output_map << "\"\n"
          << "map_voxel_resolution_m: " << options.map_voxel_resolution_m << "\n"
          << "map_min_range_m: " << options.map_min_range_m << "\n"
          << "map_max_range_m: " << options.map_max_range_m << "\n"
          << "scan_frame: body\n"
          << "scans_already_deskewed: true\n"
          << "evaluation_max_time_gap_s: " << config.max_time_gap_s << "\n"
          << "lio_trajectory: \"" << options.lio_trajectory << "\"\n"
          << "origin_lla: [" << origin_lla.x() << ", " << origin_lla.y() << ", "
          << origin_lla.z() << "]\n"
          << "input_poses: " << stats.input_poses << "\n"
          << "control_poses: " << stats.control_poses << "\n"
          << "input_measurements: " << stats.input_measurements << "\n"
          << "used_measurements: " << stats.used_measurements << "\n"
          << "rejected_time: " << stats.rejected_time << "\n"
          << "rejected_outlier: " << stats.rejected_outlier << "\n"
          << "rejected_postfit: " << stats.rejected_postfit << "\n"
          << "initial_alignment_duration_s: " << config.initial_alignment_duration_s << "\n"
          << "initial_alignment_min_baseline_m: "
          << config.initial_alignment_min_baseline_m << "\n"
          << "initial_alignment_measurements: " << stats.initial_alignment_measurements << "\n"
          << "initial_alignment_start_offset_s: "
          << stats.initial_alignment_start_offset_s << "\n"
          << "initial_alignment_span_s: " << stats.initial_alignment_span_s << "\n"
          << "effective_initial_outlier_gate_m: "
          << stats.effective_initial_outlier_gate_m << "\n"
          << "initial_outlier_gate_m: " << config.initial_outlier_gate_m << "\n"
          << "initial_outlier_gate_scope: alignment_window\n"
          << "gnss_robust_kernel: \"" << config.gnss_robust_kernel << "\"\n"
          << "gnss_robust_delta: " << config.gnss_robust_delta << "\n"
          << "gnss_covariance_scale: " << config.gnss_covariance_scale << "\n"
          << "gnss_sigma_override_m: [" << config.gnss_sigma_override_m.x() << ", "
          << config.gnss_sigma_override_m.y() << ", "
          << config.gnss_sigma_override_m.z() << "]\n"
          << "gnss_lever_arm_body_m: [" << config.gnss_lever_arm_body_m.x() << ", "
          << config.gnss_lever_arm_body_m.y() << ", "
          << config.gnss_lever_arm_body_m.z() << "]\n"
          << "postfit_outlier_chi2: " << config.postfit_outlier_chi2 << "\n"
          << "odom_neighbor_span: " << config.odom_neighbor_span << "\n"
          << "control_pose_interval_s: " << config.control_pose_interval_s << "\n"
          << "initial_horizontal_rmse_m: " << stats.initial_horizontal_rmse_m << "\n"
          << "optimized_horizontal_rmse_m: " << stats.optimized_horizontal_rmse_m << "\n"
          << "initial_3d_rmse_m: " << stats.initial_3d_rmse_m << "\n"
          << "optimized_3d_rmse_m: " << stats.optimized_3d_rmse_m << "\n"
          << "initial_graph_error: " << stats.initial_error << "\n"
          << "final_graph_error: " << stats.final_error << "\n"
          << "seed_translation: [" << translation.x() << ", " << translation.y() << ", "
          << translation.z() << "]\n"
          << "seed_orientation_xyzw: [" << correction_orientation.x() << ", "
          << correction_orientation.y() << ", " << correction_orientation.z() << ", "
          << correction_orientation.w() << "]\n"
          << "seed_yaw_deg: " << yaw_deg << "\n";
  if (!tum || !obj || !aligned || !summary || !seed) {
    message = "Failed while writing GNSS optimization outputs.";
    return false;
  }
  message = "Saved GNSS-optimized trajectory and pole-localization seed to '" +
            options.output_directory + "'.";
  return true;
}

bool writeEvaluation(const std::string& output_directory, std::string& message) {
  // The plotting script is installed beside this executable, also in catkin devel space.
  const auto script = std::filesystem::canonical("/proc/self/exe").parent_path() /
                      "plot_gnss_optimization.py";
  const pid_t child = fork();
  if (child < 0) {
    message = "Cannot start GNSS evaluation process.";
    return false;
  }
  if (child == 0) {
    execlp("python3", "python3", script.c_str(), "--output_directory",
           output_directory.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }
  int status = 0;
  pid_t waited;
  do { waited = waitpid(child, &status, 0); } while (waited < 0 && errno == EINTR);
  if (waited < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    message = "GNSS evaluation/plotting failed; trajectory outputs were saved. "
              "Check the Python error above (numpy, matplotlib and yaml are required).";
    return false;
  }
  message = "Saved GNSS RMSE/MAX reports and XYZ trajectory/error plots to '" + output_directory + "'.";
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "bievr_gnss_optimize_node");
  Options options;
  bievr::GnssTrajectoryOptimizer::Config config;
  const std::vector<std::string> args(argv, argv + argc);
  for (const auto& arg : args) {
    if (arg == "--bag" || arg.rfind("--bag=", 0) == 0) {
      LOG(E, "--bag has been removed. Use --gnss_enu FILE and --scans FILE.");
      return 2;
    }
  }
  commandLineValue(args, "--scans", options.scans);
  commandLineValue(args, "--gnss_enu", options.gnss_enu);
  commandLineValue(args, "--lio_trajectory", options.lio_trajectory);
  commandLineValue(args, "--output_directory", options.output_directory);
  commandLineValue(args, "--output_map", options.output_map);
  commandLineValue(args, "--poles_directory", options.poles_directory);
  commandLineValue(args, "--pole_params_file", options.pole_params_file);
  options.skip_poles = std::find(args.begin(), args.end(), "--skip-poles") != args.end();
  bool skip_output_map =
      std::find(args.begin(), args.end(), "--skip-output-map") != args.end();
  std::string map_resolution;
  if (commandLineValue(args, "--map_voxel_resolution_m", map_resolution)) {
    try {
      options.map_voxel_resolution_m = std::stod(map_resolution);
    } catch (const std::exception&) {
      LOG(E, "Invalid --map_voxel_resolution_m value: " << map_resolution);
      return 2;
    }
  }
  // A roslaunch invocation has a master and can configure private parameters.
  // Direct offline CLI use intentionally does not require roscore.
  if ((options.gnss_enu.empty() || options.lio_trajectory.empty()) && ros::master::check()) {
    ros::NodeHandle private_node("~");
    private_node.param("skip_output_map", skip_output_map, skip_output_map);
    private_node.param("skip_poles", options.skip_poles, options.skip_poles);
    private_node.param("poles_directory", options.poles_directory, options.poles_directory);
    private_node.param("pole_params_file", options.pole_params_file, options.pole_params_file);
    private_node.param("scans", options.scans, options.scans);
    private_node.param("gnss_enu", options.gnss_enu, options.gnss_enu);
    private_node.param("lio_trajectory", options.lio_trajectory, options.lio_trajectory);
    private_node.param("output_directory", options.output_directory, options.output_directory);
    private_node.param("output_map", options.output_map, options.output_map);
    private_node.param("map_voxel_resolution_m", options.map_voxel_resolution_m,
                       options.map_voxel_resolution_m);
    private_node.param("map_min_range_m", options.map_min_range_m,
                       options.map_min_range_m);
    private_node.param("map_max_range_m", options.map_max_range_m,
                       options.map_max_range_m);
    private_node.param("max_time_gap_s", config.max_time_gap_s, config.max_time_gap_s);
    private_node.param("horizontal_sigma_floor_m", config.horizontal_sigma_floor_m,
                       config.horizontal_sigma_floor_m);
    private_node.param("vertical_sigma_floor_m", config.vertical_sigma_floor_m,
                       config.vertical_sigma_floor_m);
    private_node.param("gnss_covariance_scale", config.gnss_covariance_scale,
                       config.gnss_covariance_scale);
    private_node.param("gnss_robust_kernel", config.gnss_robust_kernel,
                       config.gnss_robust_kernel);
    private_node.param("gnss_robust_delta", config.gnss_robust_delta,
                       config.gnss_robust_delta);
    private_node.param("initial_alignment_duration_s", config.initial_alignment_duration_s,
                       config.initial_alignment_duration_s);
    private_node.param("initial_alignment_min_baseline_m",
                       config.initial_alignment_min_baseline_m,
                       config.initial_alignment_min_baseline_m);
    private_node.param("initial_outlier_gate_m", config.initial_outlier_gate_m,
                       config.initial_outlier_gate_m);
    private_node.param("postfit_outlier_chi2", config.postfit_outlier_chi2,
                       config.postfit_outlier_chi2);
    private_node.param("odom_rotation_sigma_deg_per_sqrt_m",
                       config.odom_rotation_sigma_deg_per_sqrt_m,
                       config.odom_rotation_sigma_deg_per_sqrt_m);
    private_node.param("odom_translation_sigma_m_per_sqrt_m",
                       config.odom_translation_sigma_m_per_sqrt_m,
                       config.odom_translation_sigma_m_per_sqrt_m);
    private_node.param("minimum_edge_distance_m", config.minimum_edge_distance_m,
                       config.minimum_edge_distance_m);
    private_node.param("odom_neighbor_span", config.odom_neighbor_span,
                       config.odom_neighbor_span);
    private_node.param("control_pose_interval_s", config.control_pose_interval_s,
                       config.control_pose_interval_s);
    private_node.param("max_iterations", config.max_iterations, config.max_iterations);
    std::vector<double> lever_arm;
    if (private_node.getParam("gnss_lever_arm_body_m", lever_arm) &&
        lever_arm.size() == 3) {
      config.gnss_lever_arm_body_m =
          bievr::V3(lever_arm[0], lever_arm[1], lever_arm[2]);
    }
    std::vector<double> sigma_override;
    if (private_node.getParam("gnss_sigma_override_m", sigma_override) &&
        sigma_override.size() == 3) {
      config.gnss_sigma_override_m =
          bievr::V3(sigma_override[0], sigma_override[1], sigma_override[2]);
    }
  }
  if (options.gnss_enu.empty() || options.lio_trajectory.empty()) {
    LOG(E, "Usage: gnss_optimize --gnss_enu FILE --lio_trajectory TRAJECTORY "
           "[--scans FILE] [--output_directory DIR] [--output_map MAP.pcd] "
           "[--skip-output-map] [--poles_directory DIR] [--pole_params_file YAML] [--skip-poles]");
    return 2;
  }
  if (options.poles_directory.empty()) {
    options.poles_directory = (std::filesystem::path(options.lio_trajectory).parent_path() / "poles").string();
  }
  const bool write_map = !skip_output_map;
  if (options.scans.empty()) {
    options.scans = (std::filesystem::path(options.gnss_enu).parent_path() / "scans.bin").string();
  }
  if (skip_output_map) options.output_map.clear();
  if (write_map && options.output_map.empty()) {
    options.output_map = (std::filesystem::path(options.output_directory) /
                          "gnss_optimized_map.pcd").string();
  }
  std::error_code directory_error;
  std::filesystem::create_directories(options.output_directory, directory_error);
  if (directory_error) {
    LOG(E, "Cannot create output directory: " << directory_error.message());
    return 1;
  }
  if (write_map && !std::filesystem::is_regular_file(options.scans)) {
    LOG(E, "Missing scan archive '" << options.scans
           << "'. Record a new run, or use --skip-output-map for trajectory only.");
    return 1;
  }

  std::string message;
  std::vector<bievr::GnssTrajectoryOptimizer::StampedPose> lio;
  if (!readTum(options.lio_trajectory, lio, message)) {
    LOG(E, message);
    return 1;
  }
  std::vector<bievr::GnssTrajectoryOptimizer::PositionMeasurement> gnss;
  bievr::V3 origin_lla;
  if (!readGnssEnu(options.gnss_enu, gnss, origin_lla, message)) {
    LOG(E, message);
    return 1;
  }
  bievr::GnssTrajectoryOptimizer optimizer(config);
  bievr::GnssTrajectoryOptimizer::Stats stats;
  std::vector<bievr::GnssTrajectoryOptimizer::StampedPose> optimized;
  if (!optimizer.optimize(lio, gnss, optimized, stats, &message)) {
    LOG(E, message);
    return 1;
  }
  LOG(I, message);
  if (write_map) {
    if (!saveOptimizedPointcloudMap(options, optimized, message)) {
      LOG(E, message);
      return 1;
    }
    LOG(I, message);
  } else {
    LOG(I, "Skipping GNSS-optimized dense pointcloud map by request.");
  }
  if (!writeOutputs(options, origin_lla, lio, optimized, config, stats, message)) {
    LOG(E, message);
    return 1;
  }
  LOG(I, message);
  if (!rebuildPoles(options, optimized, message)) {
    LOG(E, message);
    return 1;
  }
  LOG(I, message);
  if (!writeEvaluation(options.output_directory, message)) {
    LOG(E, message);
    return 1;
  }
  LOG(I, message);
  return 0;
}
