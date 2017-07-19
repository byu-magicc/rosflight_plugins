cmake_minimum_required(VERSION 2.8.3)
project(rosflight_plugins)

set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  roscpp
  std_srvs
  tf
  rosflight_msgs
  rosflight_utils
  sensor_msgs
  std_msgs
  tf2_ros
  nav_msgs
  gazebo_ros
  gazebo_plugins
)

find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  CATKIN_DEPENDS geometry_msgs roscpp std_srvs tf gazebo_plugins gazebo_ros tf2_ros std_msgs nav_msgs sensor_msgs
  DEPENDS eigen gazebo rosflight_msgs SDF roscpp gazebo_ros
)

include_directories(
  include ${catkin_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(magnetometer_plugin
  src/magnetometer.cpp
include/rosflight_plugins/magnetometer.h)
target_link_libraries(magnetometer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(magnetometer_plugin ${catkin_EXPORTED_TARGETS})

add_library(odometry_plugin
  src/odometry_plugin.cpp
  include/rosflight_plugins/odometry_plugin.h)
target_link_libraries(odometry_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(odometry_plugin ${catkin_EXPORTED_TARGETS})

add_library(imu_plugin
  src/imu_plugin.cpp
  include/rosflight_plugins/imu_plugin.h)
target_link_libraries(imu_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(imu_plugin ${catkin_EXPORTED_TARGETS})

add_library(barometer_plugin
  src/barometer_plugin.cpp
  include/rosflight_plugins/barometer_plugin.h)
target_link_libraries(barometer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(barometer_plugin ${catkin_EXPORTED_TARGETS} rosflight_msgs_generate_messages_cpp)

add_library(airspeed_plugin
  src/airspeed_plugin.cpp
  include/rosflight_plugins/airspeed_plugin.h)
target_link_libraries(airspeed_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(airspeed_plugin ${catkin_EXPORTED_TARGETS} rosflight_msgs_generate_messages_cpp)

add_library(GPS_plugin
  src/GPS_plugin.cpp
  include/rosflight_plugins/GPS_plugin.h)
target_link_libraries(GPS_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(GPS_plugin ${catkin_EXPORTED_TARGETS} rosflight_msgs_generate_messages_cpp)


install(
  TARGETS
    odometry_plugin
    imu_plugin
    barometer_plugin
    GPS_plugin
    airspeed_plugin
    magnetometer_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

