cmake_minimum_required(VERSION 3.0.2)
project(fotona_gui CXX)
find_package(catkin REQUIRED COMPONENTS
roscpp
rviz
)
catkin_package()
set(BIN_NAME "fotona_gui_node")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1 -march=skylake")
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/OGRE
  ${Boost_INCLUDE_DIR}
  ${Eigen_INCLUDE_DIRS}
  ${rviz_INCLUDE_DIRS}
  )
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  find_package(Qt5 REQUIRED COMPONENTS Core)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
if(CMAKE_VERSION VERSION_LESS "3.7.0")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Widgets REQUIRED)
add_executable(${BIN_NAME}
src/main.cpp
src/main_window.cpp
src/main_window_constructor.cpp
src/rviz_active_management.cpp
src/mesh_generator.cpp
# src/depth_map_transformer.cpp
)
target_compile_features(${BIN_NAME} PRIVATE
cxx_std_17
cxx_template_template_parameters
cxx_auto_type
cxx_decltype
cxx_defaulted_functions
cxx_defaulted_move_initializers
cxx_delegating_constructors
cxx_deleted_functions
cxx_explicit_conversions
cxx_lambdas
cxx_noexcept
cxx_nullptr
cxx_rvalue_references
cxx_trailing_return_types
cxx_generic_lambdas
cxx_lambda_init_captures
)
target_link_libraries(${BIN_NAME} PRIVATE
${catkin_LIBRARIES}
Qt5::Widgets
${OGRE_LIBRARIES}
  ${QT_LIBRARIES}
)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${BIN_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()
