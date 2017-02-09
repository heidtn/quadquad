# quadquad

This is a ROS based controller for a quadrupedal robot intended to run on a Raspberry Pi.  There is a gait generator, monocular visual odometry, sparse SLAM (in progress), and a simulator for the robot itself.

The robot will eventually have integrated elements of machine learning systems.  The gait and path planning will be influenced via different ML algorithms.

# requirements

I used ubuntu Mate which can be installed directly on to a raspberry pi.  From there install ROS kinetic using 'sudo apt-get install ros-kinetic-desktop-full'

This downloads ROS and all dependencies necessary for this project.

# running

The simulator can be run by calling 'roslaunch quadquad_gazebo basicworld.launch'

The gait controller can be run by calling 'python /path/to/gait_controller.py' 

The vision odometry (egomotion) and SLAM can be run by calling 'rosrun quadquad_vision quadquad_vision_node'


# why?

The goal of this project is to integrate machine learning with conventional robotics.  The quadruped platform seemed to be a very interesting way to accomplish this and allows a steady trend of integration.

# future plans

The robot does not currently have a depth sensor, but a sharp infrared depth sensor will be added soon to better create an occupancy grid for navigation.


