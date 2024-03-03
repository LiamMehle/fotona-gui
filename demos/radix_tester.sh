set -xe
catkin build radix &
killall roscore &
killall radix_node &
killall rosbag &

wait
clear

roscore > /dev/null &
python3 ~/ros/new_catkin3/src/tester/src/main.py &
~/ros/new_catkin3/src/radix/bin/radix_node
# -q quiet
# -i immediate
# -l -k
# --topics pico_flexx/points