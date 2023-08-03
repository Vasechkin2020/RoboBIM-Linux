Hey, i had the same problem and this is my solution:

clone the csm-Repo in your catkin_ws: 
git clonesource devel/setup.bash
move in to the repo cd csm
move back cd ..
solve dependencies: rosdep install --from-paths src --ignore-src -r -y
move into csm cd csm
Use the install instructions from their manual:
cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/local .
make
sudo make install
Move back cd ..
Build catkin_make
I hope, that will solve your problem!

Hey, try my metod
1 cd ~/catkin_ws/src
2 git clone https://github.com/AndreaCensi/csm
3 cd ..
4 catkin_make_isolated
5 cd ~/catkin_ws/src
6 git clone https://github.com/ccny-ros-pkg/scan_tools.git
7 cd ..
8 catkin_make_isolated
9 source devel/setup.bash

1. This package has been tested well in Ubuntu 16.04 with ROS Kinetic.
    https://github.com/nkuwenjian/laser_scan_matcher

2. If you want to use it, you must install csm first:
    $ sudo apt-get install ros-kinetic-csm
    
3. Clone the repo to your workspace and complie it
    $ cd ~/catkin_ws/src/
    $ git clone https://github.com/nkuwenjian/laser_scan_matcher.git
    $ cd ..
    $ catkin_make
    $ source devel/setup.bash

4. Run offline rosbag
    $ roslaunch laser_scan_matcher demo.launch
    $ rosbag play <rosbagfile> --clock