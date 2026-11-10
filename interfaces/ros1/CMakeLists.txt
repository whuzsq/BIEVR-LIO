cmake_minimum_required(VERSION 3.16)
project(bievr_lio_ros LANGUAGES CXX)

# Declare the C++ standard via CMake's mechanism (not a raw -std flag): linking
# Ceres propagates a cxx_std_17 compile feature, and a raw -std=c++2a would be
# overridden by the -std=gnu++17 CMake appends for that feature. Setting the
# standard makes CMake take the max (C++20). GCC still needs -fconcepts.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fconcepts -fdiagnostics-color=always")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  nav_msgs
  tf
  tf_conversions
  rosbag
  pcl_conversions
  std_srvs
)
# GeographicLib distributions do not all ship a CMake package configuration.
find_path(GEOGRAPHICLIB_INCLUDE_DIR GeographicLib/LocalCartesian.hpp)
find_library(GEOGRAPHICLIB_LIBRARY NAMES GeographicLib)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GeographicLib REQUIRED_VARS
  GEOGRAPHICLIB_INCLUDE_DIR GEOGRAPHICLIB_LIBRARY
)
if(NOT GeographicLib_FOUND)
  message(FATAL_ERROR "GeographicLib development files are required (libgeographiclib-dev).")
endif()
find_package(TBB REQUIRED)
find_package(yaml-cpp REQUIRED)  # shared config_loader.h parses plain YAML files
# bievr_lio (core) and bievr_ros_common (shared header-only glue) are plain-CMake
# packages (not catkin), so they are not pulled in via ${catkin_LIBRARIES}; find
# and link their exported targets explicitly.
find_package(bievr_lio REQUIRED)
find_package(bievr_ros_common REQUIRED)
find_package(bievr_pgo REQUIRED)
find_package(bievr_localization REQUIRED)

# Livox support is optional: the Livox CustomMsg branches are only compiled when
# the corresponding driver is on the system. Users without the drivers can still
# build and run with sensor_msgs/PointCloud2 inputs.
#   livox_ros_driver  (gen1) -> BIEVR_WITH_LIVOX
#   livox_ros_driver2 (gen2) -> BIEVR_WITH_LIVOX2
find_package(livox_ros_driver QUIET)
find_package(livox_ros_driver2 QUIET)
if(livox_ros_driver_FOUND)
  message(STATUS "bievr_lio_ros: livox_ros_driver found, enabling Livox (gen1) support.")
else()
  message(STATUS "bievr_lio_ros: livox_ros_driver NOT found, building without Livox (gen1) support.")
endif()
if(livox_ros_driver2_FOUND)
  message(STATUS "bievr_lio_ros: livox_ros_driver2 found, enabling Livox (gen2) support.")
else()
  message(STATUS "bievr_lio_ros: livox_ros_driver2 NOT found, building without Livox (gen2) support.")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bievr_pole_extractor
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    std_msgs
    nav_msgs
    tf
    tf_conversions
    rosbag
    pcl_conversions
    std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(bievr_pole_extractor
  src/pole_extractor.cpp
  src/pole_localization.cpp
)
add_dependencies(bievr_pole_extractor ${catkin_EXPORTED_TARGETS})
target_link_libraries(bievr_pole_extractor
  ${catkin_LIBRARIES}
  bievr_lio::bievr_lio
  bievr_ros_common::bievr_ros_common
  bievr_pgo::bievr_pgo
  ${YAML_CPP_LIBRARIES}
)

add_executable(process_topics src/process_topics.cpp)
add_executable(process_bag src/process_bag.cpp)
add_executable(gnss_optimize src/gnss_optimize.cpp)
add_executable(gnss_recorder src/gnss_recorder.cpp)
target_include_directories(gnss_recorder PRIVATE ${GEOGRAPHICLIB_INCLUDE_DIR})
target_link_libraries(gnss_recorder ${GEOGRAPHICLIB_LIBRARY})
add_dependencies(gnss_recorder ${catkin_EXPORTED_TARGETS})
target_link_libraries(gnss_recorder ${catkin_LIBRARIES})
# In an isolated workspace a stale merged devel/include can precede the freshly
# exported plain-CMake package. Prefer this repository's matching public header
# for the GNSS executable; the linked implementation still comes from the
# exported bievr_pgo target.
target_include_directories(gnss_optimize BEFORE PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/pgo/include
)

add_dependencies(gnss_optimize ${catkin_EXPORTED_TARGETS})
target_link_libraries(gnss_optimize
  bievr_pole_extractor
  ${YAML_CPP_LIBRARIES}
  ${catkin_LIBRARIES}
  bievr_lio::bievr_lio
  bievr_ros_common::bievr_ros_common
  bievr_pgo::bievr_pgo
)
if(UNIX AND NOT APPLE)
  target_link_options(gnss_optimize PRIVATE "-Wl,--disable-new-dtags")
  set_target_properties(gnss_optimize PROPERTIES INSTALL_RPATH "$ORIGIN/..")
endif()

foreach(target process_topics process_bag)
  # Ensure message headers from catkin dependencies exist first.
  add_dependencies(${target} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${target}
    ${catkin_LIBRARIES}
    bievr_lio::bievr_lio
    bievr_ros_common::bievr_ros_common
    bievr_pgo::bievr_pgo
    bievr_localization::bievr_localization
    bievr_pole_extractor
    ${YAML_CPP_LIBRARIES}
    TBB::tbb
  )
  if(UNIX AND NOT APPLE)
    # The isolated workspace contains several plain-CMake libraries. Catkin's
    # top-level setup only exports the ROS package's lib directory, while these
    # are transitive dependencies of pgo/localization. Use old-style DT_RPATH
    # (transitive) for the build tree, and find the co-installed libraries one
    # directory above the node after installation.
    target_link_options(${target} PRIVATE "-Wl,--disable-new-dtags")
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN/..")
  endif()
  # Optional Livox drivers: define the guard macros and pull in headers/libs
  # only when found (livox_ros_driver = gen1, livox_ros_driver2 = gen2).
  if(livox_ros_driver_FOUND)
    target_compile_definitions(${target} PRIVATE BIEVR_WITH_LIVOX)
    target_include_directories(${target} PRIVATE ${livox_ros_driver_INCLUDE_DIRS})
    target_link_libraries(${target} ${livox_ros_driver_LIBRARIES})
  endif()
  if(livox_ros_driver2_FOUND)
    target_compile_definitions(${target} PRIVATE BIEVR_WITH_LIVOX2)
    target_include_directories(${target} PRIVATE ${livox_ros_driver2_INCLUDE_DIRS})
    target_link_libraries(${target} ${livox_ros_driver2_LIBRARIES})
  endif()
endforeach()

install(TARGETS bievr_pole_extractor process_bag process_topics gnss_optimize gnss_recorder
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS
  ../../scripts/plot_gnss_trajectory_osm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(rail_mount_prior_test tests/rail_mount_prior_test.cpp)
  target_link_libraries(rail_mount_prior_test bievr_pole_extractor ${catkin_LIBRARIES})
  if(UNIX AND NOT APPLE)
    target_link_options(rail_mount_prior_test PRIVATE "-Wl,--disable-new-dtags")
  endif()
  add_test(NAME rail_mount_prior COMMAND rail_mount_prior_test)
  add_executable(scan_archive_test tests/scan_archive_test.cpp)
  target_link_libraries(scan_archive_test bievr_lio::bievr_lio)
  add_test(NAME scan_archive_roundtrip COMMAND scan_archive_test)
  add_test(NAME gnss_poles_rebuild
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/gnss_poles_rebuild_test.py $<TARGET_FILE:gnss_optimize>
  )
  add_test(NAME gnss_evaluation_report
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/gnss_evaluation_test.py
  )
endif()

catkin_install_python(PROGRAMS
  ../../scripts/plot_gnss_optimization.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
