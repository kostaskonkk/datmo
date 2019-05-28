cmake_minimum_required(VERSION 2.8.3)
project(datmo)
add_compile_options(-std=c++11)

# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES datmo
# #  CATKIN_DEPENDS roscpp std_msgs
# #  DEPENDS system_lib
# )

# http://wiki.ros.org/eigen
# list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  sensor_msgs
  cmake_modules REQUIRED
#   message_generation
)
find_package(Eigen REQUIRED)

# add_message_files(
#    FILES
#    Encoder.msg
#    ECU.msg
# )
#  generate_messages(
#    DEPENDENCIES
#    std_msgs
#  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_tracking
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)



#add_executable(frames_sim src/frames_sim.cpp)
#target_link_libraries(frames_sim ${catkin_LIBRARIES})
#add_executable(frames_dsv src/frames_dsv.cpp)
#target_link_libraries(frames_dsv ${catkin_LIBRARIES})

add_executable(datmo src/trajectory.cpp  src/main.cpp src/kalman-cpp/kalman.cpp src/datmo.cpp src/cluster.cpp)
add_dependencies(datmo ${catkin_EXPORTED_TARGETS})
target_link_libraries(datmo ${catkin_LIBRARIES})
