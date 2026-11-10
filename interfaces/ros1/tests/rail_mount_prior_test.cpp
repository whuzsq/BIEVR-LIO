#include "bievr_lio_ros/pole_extractor.h"
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
struct Observation { double station; bievr::V3 center; };
std::vector<Observation> run(const std::filesystem::path& directory, bool prior,
                             bool real_rails, bool distractor, double height,
                             const bievr::Rotation& rotation, bool reverse) {
  bievr::PoleExtractionConfig config;
  config.enable = true;
  config.rail_enable = true;
  config.rail_mount_prior_enable = prior;
  config.frames_per_keyframe = 12;
  config.min_frames_per_keyframe = 12;
  config.keyframe_overlap_frames = 0;
  config.min_partial_frames = 2;
  config.keyframe_distance = 0.0;
  config.max_queue = 1;
  config.input_voxel_size = 0.02;
  config.output_directory = directory.string();
  config.save_keyframe_clouds = false;
  bievr::PoleExtractor extractor(config);
  for (size_t frame = 0; frame < 12; ++frame) {
    std::vector<bievr::V3> measured;
    auto add_pair = [&](double center_y, double background_depth) {
      for (int station = -40; station <= 40; ++station) {
        for (double side : {-0.5, 0.5}) {
          const double y = center_y + side * config.rail_nominal_gauge;
          measured.emplace_back(station * 0.1, y, -height);
          for (int offset : {-3, -2, 2, 3})
            measured.emplace_back(station * 0.1, y + offset * 0.05, -height - background_depth);
        }
      }
    };
    if (real_rails) add_pair(config.rail_mount_center_y, 0.10);
    if (distractor) add_pair(config.rail_mount_center_y + 1.0, 0.35);
    bievr::Pointcloud cloud;
    cloud.resize(measured.size());
    for (size_t i = 0; i < measured.size(); ++i) cloud.data().col(i) = measured[i];
    const bievr::V3 translation = rotation * bievr::V3((reverse ? -1.0 : 1.0) * frame, 0, 0);
    extractor.addFrame(1000000000ULL + frame * 100000000ULL,
                       bievr::Transform(rotation, translation), cloud);
  }
  extractor.finish();
  std::string message;
  require(extractor.saveResults(&message), message.c_str());
  require(extractor.stats().dropped_keyframes == 0, "Offline extraction dropped frames");
  std::ifstream file(directory / "rail_vector_map.txt");
  std::string line;
  std::getline(file, line);
  std::vector<Observation> observations;
  while (std::getline(file, line)) {
    std::istringstream stream(line);
    size_t segment, index;
    double stamp, station;
    bievr::V3 left, right;
    require(bool(stream >> segment >> index >> stamp >> station >> left.x() >> left.y() >>
                 left.z() >> right.x() >> right.y() >> right.z()), "Malformed output rail sample");
    observations.push_back({station, 0.5 * (left + right)});
  }
  return observations;
}

double center_y(const std::vector<Observation>& samples, const bievr::Rotation& rotation,
                bool reverse, bool height_check = true) {
  require(samples.size() >= 12, "Expected a continuous detected rail segment");
  bievr::V3 direction = rotation * bievr::V3(reverse ? -1.0 : 1.0, 0, 0);
  direction /= direction.head<2>().norm();
  std::vector<double> values;
  for (const auto& sample : samples) {
    const bievr::V3 body = rotation.transpose() * (sample.center - sample.station * direction);
    values.push_back(body.y());
    if (height_check) require(std::abs(body.z() + 4.135) < 0.08, "Height prior did not follow body attitude");
  }
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "rail_mount_prior_test", ros::init_options::AnonymousName);
  const auto directory = std::filesystem::temp_directory_path() /
      ("bievr-mount-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    const bievr::Rotation identity = bievr::Rotation::Identity();
    const auto old = run(directory / "disabled", false, true, true, 4.135, identity, false);
    require(std::abs(center_y(old, identity, false) - 1.49) < .10,
            "Fixture must reproduce a stronger but displaced distractor pair");
    const auto guided = run(directory / "enabled", true, true, true, 4.135, identity, false);
    require(std::abs(center_y(guided, identity, false) - .49) < .08,
            "Installation prior failed to select the train's own track");
    require(run(directory / "distractor_only", true, false, true, 4.135, identity, false).empty(),
            "Prior invented rails or accepted an out-of-band distractor");
    require(run(directory / "wrong_height", true, true, false, 4.65, identity, false).empty(),
            "Prior accepted rails outside the mounting-height tolerance");
    const bievr::Rotation tilted = (Eigen::AngleAxisd(.5, bievr::V3::UnitZ()) *
        Eigen::AngleAxisd(.04, bievr::V3::UnitY()) *
        Eigen::AngleAxisd(.10, bievr::V3::UnitX())).toRotationMatrix();
    for (bool reverse : {false, true}) {
      const auto samples = run(directory / (reverse ? "reverse" : "tilted"), true,
                               true, false, 4.135, tilted, reverse);
      require(std::abs(center_y(samples, tilted, reverse) - .49) < .08,
              "Body prior changed sides under yaw/tilt/reverse travel");
    }
    std::filesystem::remove_all(directory);
    std::cout << "PASS: own-track selection, distractor/height rejection, attitude and reverse travel\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << " (fixtures at " << directory << ")\n";
    return 1;
  }
}
