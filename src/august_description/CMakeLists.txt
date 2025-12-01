cmake_minimum_required(VERSION 2.8.3)
project(august_description)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)   # ← UNCOMMENTED THIS (you had it commented)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
)

## ← ADD THESE 3 LINES ONLY (for jsoncpp + websocketpp + threads)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fusion2urdf
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}          # ← ADDED
)

# ← YOUR ORIGINAL NODES 100% UNCHANGED
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

# ← ONLY THIS LINE CHANGED – added jsoncpp + pthread
add_executable(drone_mixer src/drone_mixer.cpp)
target_link_libraries(drone_mixer 
  ${catkin_LIBRARIES} 
  ${JSONCPP_LIBRARIES} 
  ${CMAKE_THREAD_LIBS_INIT}   # pthread for std::thread
)

add_executable(test_imu src/test_imu.cpp)
target_link_libraries(test_imu ${catkin_LIBRARIES})