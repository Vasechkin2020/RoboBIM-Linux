#rostopic list
#chmod +x test.sh
rosparam set /use_sim_time true
rosbag play --clock -l src/pb/bag/lidar175.bag
rosparam set /use_sim_time false