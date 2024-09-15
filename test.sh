#rostopic list
#chmod +x test.sh
rosparam set /use_sim_time true
rosbag play --clock test2.bag
rosparam set /use_sim_time false