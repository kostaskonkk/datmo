cmake_minimum_required(VERSION 2.8.3)
project(datmo)
add_compile_options(-std=c++11)
# list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)# required for YCM
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  nav_msgs
  sensor_msgs
  visualization_msgs
  cmake_modules REQUIRED
  message_generation
)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)

#MultiThreading Library
OPTION (USE_OpenMP "Use OpenMP" OFF)
IF(USE_OpenMP)
  FIND_PACKAGE(OpenMP)
  IF(OPENMP_FOUND)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  ENDIF()
ENDIF()

add_message_files(
  FILES
  Track.msg
  TrackArray.msg
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(datmo src/main.cpp src/l_shape_tracker.cpp src/kalman-cpp/kalman.cpp src/datmo.cpp src/cluster.cpp)
add_dependencies(datmo ${catkin_EXPORTED_TARGETS} datmo_generate_messages_cpp)
target_link_libraries(datmo ${catkin_LIBRARIES})
