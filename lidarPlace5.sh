#rostopic list
#chmod +x /home/pi/RoboBIM-Linux/lidarPlace5.sh
rosparam set /use_sim_time true
rosbag play --clock -l src/pb/bag/lidar3m-1m.bag
rosparam set /use_sim_time false