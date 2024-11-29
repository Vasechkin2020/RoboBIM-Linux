#rostopic list
#chmod +x test.sh
rosparam set /use_sim_time true
rosbag play --clock -l lidarOnPlace1.bag 
rosparam set /use_sim_time false