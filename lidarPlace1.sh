#rostopic list
#chmod +x test.sh
rosparam set /use_sim_time true
#rosbag play --clock -l src/pb/bag/lidarOnPlace1.bag
rosbag play --clock -l src/pb/bag/lidarOnPlaceGood.bag
rosparam set /use_sim_time false