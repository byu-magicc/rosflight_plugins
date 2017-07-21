cmake_minimum_required(VERSION 2.8.3)
project(rosflight_plugins)

set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "-fopenmp")

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  rosflight_msgs
  rosflight_utils
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  gazebo_ros
  gazebo_plugins
)

find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp tf rosflight_msgs rosflight_utils std_msgs nav_msgs geometry_msgs sensor_msgs std_srvs gazebo_ros gazebo_plugins
  DEPENDS EIGEN3 GAZEBO
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(magnetometer_plugin
  src/magnetometer.cpp)
target_link_libraries(magnetometer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(magnetometer_plugin ${catkin_EXPORTED_TARGETS})

add_library(odometry_plugin
  src/odometry_plugin.cpp)
target_link_libraries(odometry_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(odometry_plugin ${catkin_EXPORTED_TARGETS})

add_library(imu_plugin
  src/imu_plugin.cpp)
target_link_libraries(imu_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(imu_plugin ${catkin_EXPORTED_TARGETS})

add_library(barometer_plugin
  src/barometer_plugin.cpp)
target_link_libraries(barometer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(barometer_plugin ${catkin_EXPORTED_TARGETS})

add_library(airspeed_plugin
  src/airspeed_plugin.cpp)
target_link_libraries(airspeed_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(airspeed_plugin ${catkin_EXPORTED_TARGETS})

add_library(GPS_plugin
  src/GPS_plugin.cpp)
target_link_libraries(GPS_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(GPS_plugin ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

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

