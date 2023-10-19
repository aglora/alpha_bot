cmake_minimum_required(VERSION 3.0.2)
project(alpha_bot_odometry)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


 add_executable(ground_truth_translation_node src/ground_truth_translation.cpp)
 target_link_libraries(ground_truth_translation_node ${catkin_LIBRARIES})
 add_executable(alpha_bot_odometry_node src/alpha_bot_odometry_node.cpp)
 target_link_libraries(alpha_bot_odometry_node ${catkin_LIBRARIES})
 

