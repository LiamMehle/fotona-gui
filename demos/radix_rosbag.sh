set -xe
catkin build radix &
killall roscore &
killall radix_node &
killall rosbag &

wait
clear

roscore > /dev/null &
rosbag play '/mnt/c/Users/Liam/Desktop/actual documents/gui_2023-03-10-15-03-23.bag' -s 40 -q -l --topics /pico_flexx/points > /dev/null &
~/ros/new_catkin3/src/radix/bin/radix_node
# -q quiet
# -i immediate
# -l -k
# --topics pico_flexx/points