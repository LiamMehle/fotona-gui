cmake_minimum_required(VERSION 3.0.2)
project(librviz_test)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)
add_executable(sender "src/sender.cpp")
target_link_libraries(sender ${catkin_LIBRARIES})
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
