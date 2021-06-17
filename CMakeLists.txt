cmake_minimum_required(VERSION 2.8.3)
project(corner_detector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


#set(PCL_DIR "/usr/lib/aarch64-linux-gnu/cmake/pcl/")
# set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  visualization_msgs
  laser_geometry
  pcl_ros
  mrpt_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES corner_detector
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/corner_detector.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(Detector src/Detector.cpp src/VectorUtils.cpp src/LineClustering.cpp src/Visualization.cpp src/RobotState.cpp 
                        src/GlobalFeatureMap.cpp src/PointMatching.cpp src/LineMatching.cpp src/AccumulateFeatures.cpp)

target_link_libraries(Detector
  ${catkin_LIBRARIES}
)

add_executable(TfBroadcaster src/TfBroadcaster.cpp)

target_link_libraries(TfBroadcaster
  ${catkin_LIBRARIES}
)

