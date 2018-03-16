# Terrapin-point
Final project repository for CSC 790 'Advanced Robotics'  

## Instructions  
Firstly, you will need to install [ROS](http://wiki.ros.org/lunar/Installation)  

You may install the following additional packages through apt:  

```
sudo apt-get install ros-lunar-joy ros-lunar-rgbd-launch ros-lunar-rosserial-arduino ros-lunar-rosserial-python ros-lunar-rosserial-server ros-lunar-rosserial-client ros-lunar-rosserial-msgs ros-lunar-amcl ros-lunar-map-server ros-lunar-move-base ros-lunar-urdf ros-lunar-xacro ros-lunar-compressed-image-transport ros-lunar-rqt-image-view
ros-lunar-gmapping ros-lunar-navigation ros-lunar-interactive-markers
```

Our project relies on the Turtlebot3 packages by [ROBOTIS]().  
clone them into your local catkin workspace:  
`git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git`
`git clone https://github.com/ROBOTIS-GIT/turtlebot3.git`
`git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git`  

Build the workspace and source the setup.bash file.  

Our project uses the Turtlebot3 Waffle, so export the following environment variable:  
`export TURTLEBOT3_MODEL=waffle`  


In order to run this project's ROS node that processes depth image data,  
you will first need to make sure the a few prerequisite packages are running via roslaunch.  
`roslaunch turtlebot3_fake turtlebot3_fake.launch`  
`roslaunch turtlebot3_gazebo_ros turtlebot3_house.launch`  

After you have these two processes running you may run any turtlebot module  
that you could run with a physical device!  

To view the depth image data:  
rqt_image_view  

Finally, you may run our node:  
`rosrun terrapin_detection detection.py`  
