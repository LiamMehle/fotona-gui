set -xe
catkin build radix &
killall roscore &
killall fotona_gui &
killall rosbag &

wait
clear

rosbag play '/mnt/c/Users/Liam/Desktop/actual documents/gui_2023-03-10-15-03-23.bag' -s 40 -q -l --topics /pico_flexx/points > /dev/null &
roslaunch fotona_gui gui.launch
# -q quiet
# -i immediate
# -l -k
# --topics pico_flexx/points