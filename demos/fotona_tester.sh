set -xe
catkin build radix &
killall roscore &
killall fotona_gui &
killall rosbag &

wait
clear

python3 ~/ros/new_catkin3/src/tester/src/main.py &
roslaunch fotona_gui gui.launch
# -q quiet
# -i immediate
# -l -k
# --topics pico_flexx/points