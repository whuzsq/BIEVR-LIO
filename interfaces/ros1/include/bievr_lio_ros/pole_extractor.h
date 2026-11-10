#ifndef BIEVR_LIO_ROS_POLE_EXTRACTOR_H_
#define BIEVR_LIO_ROS_POLE_EXTRACTOR_H_

#include <bievr_lio/pipeline.h>
#include <ros/ros.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace bievr {

// Online counterpart of scripts/coarse_extraction.py. Consecutive LIO frames are
// accumulated in the world frame and processed as one pole-extraction keyframe.
struct PoleExtractionConfig {
  bool enable = false;

  // Keyframe construction and background work queue.
  // A keyframe is emitted after traveling keyframe_distance, or when the
  // maximum frame count is reached. A small overlap avoids boundary misses.
  size_t frames_per_keyframe = 50;
  size_t min_frames_per_keyframe = 20;
  double keyframe_distance = 30.0;
  size_t keyframe_overlap_frames = 5;
  bool process_partial_keyframe = true;
  size_t min_partial_frames = 10;
  size_t max_queue = 3;
  double input_voxel_size = 0.10;

  // Optional mapping-only pose source. A GNSS/PGO trajectory can be replayed
  // so accumulated scans and saved poles use its world frame. Query-time pole
  // localization deliberately continues to use only live LIO poses.
  std::string mapping_trajectory_path;
  double mapping_trajectory_max_time_gap_s = 0.20;

  // Track-relative region of interest.
  bool use_roi = true;
  double roi_y_min = -10.0;
  double roi_y_max = 10.0;
  double roi_z_min = -5.0;
  double roi_z_max = 10.0;

  // 2-D grid and vertical occupancy tests.
  double grid_resolution = 0.5;
  int min_points_per_cell = 60;
  double min_height_span = 8.0;
  double layer_resolution = 0.2;
  int min_continuous_layers = 40;
  int max_component_cells = 8;

  // Euclidean refinement. This corresponds to CLUSTERING_METHOD=EUCLIDEAN in
  // the original script.
  double cluster_tolerance = 1.0;
  double max_xy_span = 1.0;
  int min_cluster_points = 50;
  int max_cluster_points = 5000;

  // SE-LIO-inspired cylinder validation after coarse candidate extraction.
  bool cylinder_fit_enable = true;
  int cylinder_ransac_iterations = 128;
  double cylinder_inlier_threshold = 0.10;
  double cylinder_min_inlier_ratio = 0.35;
  double cylinder_max_rms = 0.20;
  double cylinder_min_radius = 0.08;
  double cylinder_max_radius = 0.80;
  double cylinder_max_axis_tilt_deg = 5.0;
  double cylinder_min_angular_coverage = 0.25;
  int cylinder_max_piecewise_depth = 0;
  int cylinder_min_segment_points = 80;

  // The fitted axis is intersected with trajectory_z + this offset.
  double center_z_offset = -5.0;

  // Merge repeated observations using position, cylinder axis and radius.
  double landmark_merge_radius = 1.0;
  double landmark_max_axis_angle_deg = 5.0;
  double landmark_max_radius_difference = 0.30;

  std::string output_directory = "Log/poles";
  std::string extraction_config_yaml; // Effective YAML snapshot for offline rebuilding.
  // Legacy config name: save all extracted pole points together in poles.pcd.
  bool save_keyframe_clouds = true;

  // Match a rolling local pole map against landmarks saved by a mapping run.
  // The result is an SE(2) refinement of T_map_odom; z/roll/pitch are retained
  // from the supplied initial correction.
  bool localization_enable = false;
  std::string localization_map_path;
  std::string localization_map_frame = "map";
  size_t localization_window_keyframes = 7;
  double localization_match_distance = 4.0;
  // Extend the extraction ROI only around predicted prior axes. Return runs
  // can see the same poles from a neighbouring track outside the normal ROI.
  bool localization_guided_roi = true;
  double localization_guided_roi_radius = 5.0;
  double localization_guided_roi_max_range = 40.0;
  double localization_radius_difference = 0.30;
  double localization_axis_angle_deg = 5.0;
  int localization_min_matches = 3;
  double localization_min_match_ratio = 0.60;
  double localization_min_baseline = 10.0;
  double localization_max_rms = 0.50;
  double localization_huber_delta = 0.50;
  double localization_max_translation_step = 5.0;
  double localization_max_yaw_step_deg = 5.0;
  bool localization_use_initial_guess = false;
  V3 localization_initial_translation = V3::Zero();
  double localization_initial_yaw_deg = 0.0;
  bool localization_initial_orientation_enable = false;
  Quaternion localization_initial_orientation = Quaternion::Identity();

  // Rail-vector extraction reuses exactly the same accumulated keyframes.
  // Each accepted cross section contributes a paired left/right rail-top
  // control point. Localization connects adjacent control points into 3-D
  // line segments and projects observations continuously onto those segments;
  // the saved text format remains backward compatible.
  bool rail_enable = false;
  // Installation prior in IMU/body coordinates (+Y follows that frame).
  // Generic users opt in through YAML; absent keys preserve old map snapshots.
  bool rail_mount_prior_enable = false;
  double rail_mount_height = 4.135;
  double rail_mount_center_y = 0.49;
  double rail_mount_lateral_tolerance = 0.25;
  double rail_mount_height_tolerance = 0.20;
  double rail_mount_prior_weight = 0.15;
  double rail_station_resolution = 0.40;
  double rail_lateral_resolution = 0.05;
  double rail_lateral_min = -3.0;
  double rail_lateral_max = 3.0;
  double rail_relative_z_min = -5.30;
  double rail_relative_z_max = -3.40;
  double rail_nominal_gauge = 1.435;
  double rail_min_gauge = 1.20;
  double rail_max_gauge = 1.80;
  int rail_min_cell_points = 1;
  double rail_min_prominence = 0.035;
  double rail_max_lateral_jump = 0.30;
  double rail_max_vertical_jump = 0.15;
  double rail_max_cross_level = 0.20;
  size_t rail_min_segment_samples = 12;
  // Full-resolution measured points near the detected rail head.
  double rail_cloud_half_width = 0.10;
  double rail_cloud_depth = 0.18;
  double rail_cloud_above = 0.05;
  std::string rail_map_path;
  double rail_match_distance = 2.0;
  double rail_max_tangent_angle_deg = 12.0;
  size_t rail_min_matches = 10;
  double rail_min_baseline = 8.0;
  double rail_huber_delta = 0.08;
  double rail_max_rms = 0.12;
  double rail_max_planar_rms = 0.30;
  double rail_max_cross_track_step = 1.0;
  double rail_max_yaw_step_deg = 2.0;
  double rail_max_z_step = 1.0;
  double rail_max_tilt_step_deg = 2.0;
  bool rail_update_roll = true;
  bool rail_update_pitch = true;
};

struct PoleExtractionStats {
  size_t received_frames = 0;
  size_t processed_keyframes = 0;
  size_t dropped_keyframes = 0;
  size_t rejected_trajectory_frames = 0;
  size_t detections = 0;
  size_t landmarks = 0;
  size_t localization_attempts = 0;
  size_t localization_accepted = 0;
  size_t rail_samples = 0;
  size_t rail_localization_attempts = 0;
  size_t rail_localization_accepted = 0;
};

struct PoleLocalizationCorrection {
  Transform T_map_odom = Transform::Identity();
  uint64_t stamp = 0;
  // Cumulative planar LIO travel at stamp. This lets the localization filter
  // propagate covariance in sensor-time order instead of worker wall time.
  double odom_distance_m = 0.0;
  // Odom-frame pivot at which a planar correction was observed. Comparing
  // map<-odom translations directly is not invariant to yaw: far from the
  // odom origin a small yaw change can alter tx/ty by several metres while
  // moving the vehicle only centimetres.
  V3 correction_anchor_odom = V3::Zero();
  size_t matches = 0;
  double match_ratio = 0.0;
  double baseline = 0.0;
  double rms = 0.0;
  // For a rail observation, the planar RMS is the centerline point-to-line
  // residual. rail_tangent_map defines the deliberately unobservable
  // along-track translation direction.
  double planar_rms = 0.0;
  V3 rail_tangent_map = V3::UnitX();
  // Pole corrections constrain x/y/yaw. Rail corrections constrain
  // z/roll/pitch. Keeping the masks explicit prevents a straight rail from
  // injecting an unobservable along-track displacement.
  bool constrain_planar = true;
  bool constrain_vertical = false;
  bool constrain_cross_track = false;
  bool constrain_yaw = false;
  bool constrain_z = false;
  bool constrain_roll = false;
  bool constrain_pitch = false;
  bool valid = false;
};

// Reads pole_extraction: from the same layered YAML files as the LIO config.
bool loadPoleExtractionConfig(const std::vector<std::string>& yaml_paths,
                              PoleExtractionConfig& config);

class PoleExtractor {
 public:
  PoleExtractor(ros::NodeHandle& nh, const std::shared_ptr<Pipeline>& pipeline,
                PoleExtractionConfig config, const std::string& topic_prefix);
  // Offline replay: no ROS publishers/master, bounded queue without dropped frames.
  explicit PoleExtractor(PoleExtractionConfig config, const std::string& map_frame = "map");
  void addFrame(uint64_t stamp, const Transform& pose, const Pointcloud& cloud);
  ~PoleExtractor();

  PoleExtractor(const PoleExtractor&) = delete;
  PoleExtractor& operator=(const PoleExtractor&) = delete;

  // Stops accepting frames, optionally queues the final partial keyframe, and
  // drains the worker. Safe to call more than once.
  void finish();

  // Writes landmarks.txt, detections.txt, landmarks.pcd and summary.yaml.
  bool saveResults(std::string* message = nullptr) const;

  PoleExtractionStats stats() const;

  // Pole-only localization refines its previous filtered correction or a
  // configured/manual seed. Dense-map localization is deliberately separate.
  using CorrectionProvider = std::function<std::optional<Transform>()>;
  using CorrectionObserver = std::function<void(const PoleLocalizationCorrection&)>;
  using KeyframeObserver =
      std::function<void(uint64_t, const Transform&, double)>;
  void setCorrectionProvider(CorrectionProvider provider);
  void setCorrectionObserver(CorrectionObserver observer);
  void setKeyframeObserver(KeyframeObserver observer);
  PoleLocalizationCorrection localizationCorrection() const;
  bool localizationReady() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bievr

#endif  // BIEVR_LIO_ROS_POLE_EXTRACTOR_H_
