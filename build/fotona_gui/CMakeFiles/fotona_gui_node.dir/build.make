# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/liam/ros/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/liam/ros/catkin_ws/build

# Include any dependencies generated for this target.
include fotona_gui/CMakeFiles/fotona_gui_node.dir/depend.make

# Include the progress variables for this target.
include fotona_gui/CMakeFiles/fotona_gui_node.dir/progress.make

# Include the compile flags for this target's objects.
include fotona_gui/CMakeFiles/fotona_gui_node.dir/flags.make

fotona_gui/CMakeFiles/fotona_gui_node.dir/src/main.cpp.o: fotona_gui/CMakeFiles/fotona_gui_node.dir/flags.make
fotona_gui/CMakeFiles/fotona_gui_node.dir/src/main.cpp.o: /home/liam/ros/catkin_ws/src/fotona_gui/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/liam/ros/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object fotona_gui/CMakeFiles/fotona_gui_node.dir/src/main.cpp.o"
	cd /home/liam/ros/catkin_ws/build/fotona_gui && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fotona_gui_node.dir/src/main.cpp.o -c /home/liam/ros/catkin_ws/src/fotona_gui/src/main.cpp

fotona_gui/CMakeFiles/fotona_gui_node.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fotona_gui_node.dir/src/main.cpp.i"
	cd /home/liam/ros/catkin_ws/build/fotona_gui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/liam/ros/catkin_ws/src/fotona_gui/src/main.cpp > CMakeFiles/fotona_gui_node.dir/src/main.cpp.i

fotona_gui/CMakeFiles/fotona_gui_node.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fotona_gui_node.dir/src/main.cpp.s"
	cd /home/liam/ros/catkin_ws/build/fotona_gui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/liam/ros/catkin_ws/src/fotona_gui/src/main.cpp -o CMakeFiles/fotona_gui_node.dir/src/main.cpp.s

# Object files for target fotona_gui_node
fotona_gui_node_OBJECTS = \
"CMakeFiles/fotona_gui_node.dir/src/main.cpp.o"

# External object files for target fotona_gui_node
fotona_gui_node_EXTERNAL_OBJECTS =

/home/liam/ros/catkin_ws/devel/lib/fotona_gui/fotona_gui_node: fotona_gui/CMakeFiles/fotona_gui_node.dir/src/main.cpp.o
/home/liam/ros/catkin_ws/devel/lib/fotona_gui/fotona_gui_node: fotona_gui/CMakeFiles/fotona_gui_node.dir/build.make
/home/liam/ros/catkin_ws/devel/lib/fotona_gui/fotona_gui_node: fotona_gui/CMakeFiles/fotona_gui_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/liam/ros/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/liam/ros/catkin_ws/devel/lib/fotona_gui/fotona_gui_node"
	cd /home/liam/ros/catkin_ws/build/fotona_gui && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fotona_gui_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
fotona_gui/CMakeFiles/fotona_gui_node.dir/build: /home/liam/ros/catkin_ws/devel/lib/fotona_gui/fotona_gui_node

.PHONY : fotona_gui/CMakeFiles/fotona_gui_node.dir/build

fotona_gui/CMakeFiles/fotona_gui_node.dir/clean:
	cd /home/liam/ros/catkin_ws/build/fotona_gui && $(CMAKE_COMMAND) -P CMakeFiles/fotona_gui_node.dir/cmake_clean.cmake
.PHONY : fotona_gui/CMakeFiles/fotona_gui_node.dir/clean

fotona_gui/CMakeFiles/fotona_gui_node.dir/depend:
	cd /home/liam/ros/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/liam/ros/catkin_ws/src /home/liam/ros/catkin_ws/src/fotona_gui /home/liam/ros/catkin_ws/build /home/liam/ros/catkin_ws/build/fotona_gui /home/liam/ros/catkin_ws/build/fotona_gui/CMakeFiles/fotona_gui_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : fotona_gui/CMakeFiles/fotona_gui_node.dir/depend

