#!/bin/sh
set -xe
catkin build
source devel/setup.bash
~/git/optview2/opt-viewer.py --output-dir opt-viewer --source-dir src ./build/fotona_gui/CMakeFiles/fotona_gui_node.dir/src -j12
# rosrun fotona_gui fotona_gui_node
wait
