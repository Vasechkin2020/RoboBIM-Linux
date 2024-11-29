#rostopic list
#chmod +x /home/pi/RoboBIM-Linux/go2.ch 
rosparam set /use_sim_time true
rosbag play --clock /home/pi/RoboBIM-Linux/go2-25Hz.bag 
rosparam set /use_sim_time false