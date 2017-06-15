# ROS_RatSLAM_with_viso2
ROS version of Open RatSLAM with odometry from Viso2

This package is targeting ROS kinetic and Ubuntu 16.04 Xenial



#Install ROS
1. In "Software & Updates" under 'Other Software'-tab: Select "Canonical Parners (not the source code one)"

2. Update
sudo apt-get update
sudo apt-get upgrade

3. Set up sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

4. Set up keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

5. Install ROS
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full

(find packages: apt-cache search ros-kinetic)

6. Initialize rosdep
sudo rosdep init
rosdep update

7. Setup environment (Using Bash shell)
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
ALT (If using Z shell)
echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
ALT (Only setup single environment/terminal session)
source /opt/ros/kinetic/setup.bash


#Create catkin workspace
mkdir -p ratslam_viso2_ws/src
cd ratslam_viso2_ws/src/
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash


#Install RatSLAM (https://github.com/davidmball/ratslam/blob/wiki/RatSLAMROS.md)
##Prerequisites (opencv2, topological_nav_msgs and 3D graphics library Irrlicht)
sudo apt-get install git
sudo apt-get install libopencv-dev
sudo apt-get install libirrlicht-dev


cd src
git clone https://github.com/davidmball/ratslam.git ratslam_ros
cd ratslam_ros
git checkout ratslam_ros
cd ../..
catkin_make (HAD TO CHANGE CMakeLists.txt in src/ratslam_ros/ "OpenCV 2.4.9")


#Install Viso2
cd src
git clone https://github.com/srv/viso2.git viso2
cd viso2
git checkout viso2
cd ../..
catkin_make
source devel/setup.bash
