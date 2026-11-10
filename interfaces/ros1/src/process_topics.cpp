#include <bievr_lio/common.h>
#include <bievr_lio/log++.h>
#include <bievr_lio/synchronizer.h>
#include <tbb/global_control.h>
#include <tbb/task_arena.h>
#include <topic_tools/shape_shifter.h>

#include <chrono>
#include <thread>

#include "bievr_lio/config_loader.h"
#include "bievr_lio_ros/localization.h"
#include "bievr_lio_ros/loop_closure.h"
#include "bievr_lio_ros/pole_extractor.h"
#include "bievr_lio_ros/pole_localization.h"
#include "bievr_lio_ros/publisher.h"
#include "bievr_lio_ros/save_map_service.h"
#include "bievr_ros_common/conversions.h"
#ifdef BIEVR_WITH_LIVOX
#include <livox_ros_driver/CustomMsg.h>
#endif
#ifdef BIEVR_WITH_LIVOX2
#include <livox_ros_driver2/CustomMsg.h>
#endif

int main(int argc, char** argv) {
  srand(1);
  // ros::init strips ROS-specific arguments (remappings, __name:=, etc.) from
  // argv in place, leaving our config-file flags for loadConfigFromArgs.
  ros::init(argc, argv, "bievr_lio_topic_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  bievr::Config config;
  if (!bievr::loadConfigFromArgs({argv, argv + argc}, config)) {
    LOG(E, "Failed to load config.");
    return -1;
  }

  // Cap TBB parallelism for the whole process (0 = TBB default, i.e. all cores).
  const int n_threads =
      config.max_num_threads > 0 ? config.max_num_threads : tbb::this_task_arena::max_concurrency();
  tbb::global_control tbb_control(tbb::global_control::max_allowed_parallelism, n_threads);
  LOG(I, config.max_num_threads > 0, "TBB parallelism limited to " << n_threads << " threads.");

  std::shared_ptr<bievr::Pipeline> pipeline =
      std::make_shared<bievr::Pipeline>(config.pipeline_config);
  bievr::Synchronizer synchronizer(pipeline);
  bievr::Publisher lio_pub(nh, pipeline, "bievr_lio");
  bievr::SaveMapService save_map_service(private_nh, pipeline);

  bievr::PoleExtractionConfig pole_config;
  if (!bievr::loadPoleExtractionConfig(config.yaml_paths, pole_config)) return -1;
  bievr::LoopClosureConfig loop_config;
  if (!bievr::loadLoopClosureConfig(config.yaml_paths, loop_config)) return -1;
  bievr::LocalizationConfig localization_config;
  if (!bievr::loadLocalizationConfig(config.yaml_paths, localization_config)) return -1;
  bievr::PoleLocalizationConfig pole_localization_config;
  if (!bievr::loadPoleLocalizationConfig(config.yaml_paths, pole_localization_config)) return -1;

  const bool dense_localization_mode = localization_config.enable;
  const bool pole_localization_mode = pole_localization_config.enable;
  if (dense_localization_mode && pole_localization_mode) {
    LOG(E, "Relocalization modes are mutually exclusive: enable either dense-map "
           "localization or pole-map localization, not both.");
    return -1;
  }
  if ((dense_localization_mode || pole_localization_mode) && loop_config.enable) {
    LOG(E, "loop_closure cannot run together with either relocalization mode.");
    return -1;
  }
  if (dense_localization_mode && pole_config.enable) {
    LOG(E, "Dense-map relocalization is dense-only; set pole_extraction.enable to false.");
    return -1;
  }
  if (pole_config.localization_enable && !pole_localization_mode) {
    LOG(E, "pole_extraction.localization_enable is only valid in pole-map "
           "relocalization mode.");
    return -1;
  }
  if (pole_localization_mode &&
      (!pole_config.enable || !pole_config.localization_enable)) {
    LOG(E, "Pole-map relocalization requires pole_extraction.enable and "
           "pole_extraction.localization_enable.");
    return -1;
  }
  if (pole_localization_mode &&
      pole_localization_config.map_frame != pole_config.localization_map_frame) {
    LOG(E, "pole_localization.map_frame must match "
           "pole_extraction.localization_map_frame.");
    return -1;
  }
  if (dense_localization_mode) {
    LOG(I, "Relocalization mode: DENSE_MAP_ONLY (GICP, pole modules disabled).");
  } else if (pole_localization_mode) {
    LOG(I, "Relocalization mode: POLE_MAP_ONLY (no dense map or GICP).");
  }

  std::unique_ptr<bievr::PoleExtractor> pole_extractor;
  if (pole_config.enable) {
    pole_extractor.reset(
        new bievr::PoleExtractor(nh, pipeline, pole_config, "bievr_lio"));
    if (!pole_extractor->localizationReady()) return -1;
  }
  std::unique_ptr<bievr::LoopClosure> loop_closure;
  if (loop_config.enable) {
    loop_closure.reset(new bievr::LoopClosure(nh, private_nh, pipeline, loop_config, "bievr_lio"));
  }
  std::unique_ptr<bievr::Localization> localization;
  if (dense_localization_mode) {
    localization.reset(
        new bievr::Localization(nh, pipeline, localization_config, "bievr_lio"));
    std::string message;
    if (!localization->start(&message)) {
      LOG(E, "Failed to start localization: " << message);
      return -1;
    }
  }
  std::unique_ptr<bievr::PoleLocalization> pole_localization;
  if (pole_localization_mode) {
    pole_localization.reset(new bievr::PoleLocalization(
        nh, pipeline, pole_localization_config, "bievr_lio"));
    pole_extractor->setCorrectionProvider(
        [&pole_localization]() { return pole_localization->correction(); });
    pole_extractor->setKeyframeObserver(
        [&pole_localization](uint64_t stamp, const bievr::Transform& pose,
                             double odom_distance_m) {
          pole_localization->addKeyframe(stamp, pose, odom_distance_m);
        });
    pole_extractor->setCorrectionObserver(
        [&pole_localization](const bievr::PoleLocalizationCorrection& correction) {
          pole_localization->addPoleCorrection(correction);
        });
  }

  ros::Subscriber pointcloud_sub = nh.subscribe<topic_tools::ShapeShifter>(
      config.topic_config.pointcloud_topic, 100000,
      [&](const topic_tools::ShapeShifter::ConstPtr& msg) {
        // Detect the message type by name
        std::string msg_type = msg->getDataType();
        LOG_FIRST(I, 1, "First message type received: " << msg_type);

        if (msg_type == "sensor_msgs/PointCloud2") {
          // Deserialize into sensor_msgs::PointCloud2
          sensor_msgs::PointCloud2::ConstPtr pc_msg = msg->instantiate<sensor_msgs::PointCloud2>();
          if (pc_msg) {
            bievr::StampedIntensityPointcloud pointcloud;
            bievr::msgToPointcloud(*pc_msg, pointcloud);
            synchronizer.addPointcloud(pointcloud);
          } else {
            LOG(E, "Failed to convert ShapeShifter to PointCloud2");
          }
        }
#ifdef BIEVR_WITH_LIVOX
        else if (msg_type == "livox_ros_driver/CustomMsg") {
          // Deserialize into livox_ros_driver::CustomMsg
          livox_ros_driver::CustomMsg::ConstPtr livox_msg =
              msg->instantiate<livox_ros_driver::CustomMsg>();
          if (livox_msg) {
            bievr::StampedIntensityPointcloud pointcloud;
            bievr::msgToPointcloud(*livox_msg, pointcloud);
            synchronizer.addPointcloud(pointcloud);
          } else {
            LOG(E, "Failed to convert ShapeShifter to livox_ros_driver CustomMsg");
          }
        }
#endif
#ifdef BIEVR_WITH_LIVOX2
        else if (msg_type == "livox_ros_driver2/CustomMsg") {
          // Deserialize into livox_ros_driver2::CustomMsg
          livox_ros_driver2::CustomMsg::ConstPtr livox2_msg =
              msg->instantiate<livox_ros_driver2::CustomMsg>();
          if (livox2_msg) {
            bievr::StampedIntensityPointcloud pointcloud;
            bievr::msgToPointcloud(*livox2_msg, pointcloud);
            synchronizer.addPointcloud(pointcloud);
          } else {
            LOG(E, "Failed to convert ShapeShifter to livox_ros_driver2 CustomMsg");
          }
        }
#endif
        else {
          LOG(W, "Received unsupported message type");
        }
      });

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(
      config.topic_config.imu_topic, 100000, [&](const sensor_msgs::Imu::ConstPtr& msg) {
        bievr::ImuMeasurement imu;
        bievr::msgToImuMeasurement(*msg, imu);
        synchronizer.addImu(imu);
      });

  // Keep LiDAR and IMU callbacks serialized. Loop detection, optimization and
  // localization ICP have their own worker threads inside the core modules.
  ros::spin();

  int exit_code = 0;
  if (pole_extractor) {
    pole_extractor->finish();
    std::string message;
    if (!pole_extractor->saveResults(&message)) {
      LOG(E, message);
      exit_code = -1;
    } else {
      LOG(I, message);
    }
    const auto stats = pole_extractor->stats();
    LOG(I, "Pole extraction summary: " << stats.processed_keyframes << " keyframes, "
                                        << stats.detections << " detections, " << stats.landmarks
                                        << " merged landmarks, " << stats.dropped_keyframes
                                        << " dropped keyframes.");
    if (pole_config.localization_enable) {
      LOG(I, "Pole localization summary: " << stats.localization_accepted << "/"
                                             << stats.localization_attempts
                                             << " corrections accepted.");
    }
    if (pole_config.rail_enable) {
      LOG(I, "Rail-vector summary: " << stats.rail_samples << " samples, "
                                      << stats.rail_localization_accepted << "/"
                                      << stats.rail_localization_attempts
                                      << " vertical corrections accepted.");
    }
  }
  // Relocalization saves its corrected trajectory through the localizer below.
  // Raw LIO trajectory/map outputs are only needed for mapping runs.
  if (!dense_localization_mode && !pole_localization_mode && !pipeline->saveResults()) {
    LOG(E, "One or more final outputs could not be saved.");
    exit_code = -1;
  }

  if (loop_closure && !loop_closure->bundlePath().empty()) {
    // Ctrl+C makes ros::ok() false, but the loop-closure worker may still have
    // queued keyframes. Give it a bounded interval to finish before snapshotting.
    int idle_polls = 0;
    int total_polls = 0;
    while (idle_polls < 10 && total_polls++ < 100) {
      idle_polls = loop_closure->idle() ? idle_polls + 1 : 0;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const auto stats = loop_closure->stats();
    LOG(I, "Loop closure summary: " << stats.num_keyframes << " keyframes, "
                                     << stats.num_loops << " accepted loops, "
                                     << stats.num_rejected << " rejected candidates, "
                                     << stats.dropped_frames << " dropped keyframes.");
    std::string message;
    if (!loop_closure->saveBundle(&message)) {
      LOG(E, "Failed to save loop-closed map bundle: " << message);
      exit_code = -1;
    } else {
      LOG(I, message);
    }
  }

  if (localization) {
    std::string message;
    if (!localization->saveTrajectory(&message)) {
      LOG(E, message);
      exit_code = -1;
    } else {
      LOG(I, message);
    }
    const auto stats = localization->stats();
    LOG(I, "Localization summary: " << stats.accepted << "/" << stats.attempts
                                     << " GICP matches accepted.");
  }
  if (pole_localization) {
    std::string message;
    if (!pole_localization->saveResults(&message)) {
      LOG(E, message);
      exit_code = -1;
    } else {
      LOG(I, message);
    }
    const auto stats = pole_localization->stats();
    LOG(I, "Pole-only localization summary: " << stats.accepted << "/"
                                                << stats.measurements
                                                << " filtered updates accepted, "
                                                << stats.localized_frames
                                                << " localized LIO frames.");
  }

  return exit_code;
}
