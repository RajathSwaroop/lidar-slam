cmake_minimum_required(VERSION 2.8.3)
project(Lidar-slam)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -o3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  geometry_msgs
  nav_msgs
)

set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
)

include_directories(
  "/usr/include/eigen3"
  src
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(lidar_slam
  src/ProcessPointCloud.cpp
  src/Mapper.cpp
  src/main.cpp
)

target_link_libraries(lidar_slam
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS lidar_slam
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

