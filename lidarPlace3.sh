#rostopic list
#chmod +x test.sh
rosparam set /use_sim_time true
#rosbag play --clock -l src/pb/bag/lidarZigZag3.bag

#rosbag play --clock -l -s 65 -u 15 src/pb/bag/lidarZigZag3.bag
rosbag play --clock src/pb/bag/lidarZigZag3.bag

rosparam set /use_sim_time false