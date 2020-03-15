# turtle-sensing
This repo contains the source code for my EEC106B Robotics and Manipulation final project.

## Overview
Working alongside David Wang, our project aimed to create a hybrid autonomous/tele-operated system that alleviates the probability of vehicle collisions with stationary and dynamic obstacles. We used Kobuki Turtlebots to perform real-time obstacle detection with the Kinect sensors built-in infrared laser projector. We performed Vector Field Histogram analysis on interpolated and smoothed laser scan data, continuously monitoring the environment for objects in the Kinect’s field of view. When no obstacles are in the way, the Turtlebot can be freely driven by the user. When an obstacle is detected and determined to be in the path of the Turtlebot, our software immediately switches to autonomous control to avoid collisions, reducing the chances of impact and injury.
1. [Presentation and demo](https://docs.google.com/presentation/d/15HJ5kUDF1NXPR6jYo17pOHJCR9Qhuvq8K2VfPPu_B6Q/edit#slide=id.g39c297cd23_0_22)


## Setup Instructions
This code is based on the ROS platform. For more information, please visit [here](https://www.ros.org/) for documentation. NOTE: This project was built in Ubuntu 14.04 on ROS Indigo and ROS Kinetic for the Turtlebot

#### On Your Local Machine:
1. Clone this repo.
2. Make sure you have ROS is installed and configured. Ensure catkin_tools is installed, which should be the case by default.
3. Navigate to this repo on your local machine to the top level ROS workspace that contains the `/src` directory. Run `catkin_make` in your terminal at this level - catkin_tools should build your pacakges and create two folders: `/devel` and `/build`.
4. Remember to source `devel/setup.bash` so ROS can find the path to the **Turtlebot_Collision_Avoidance** package.

#### On your Turtlebot:
1. Re-compile your package if uncompiled. To do this, navigate to the ROS workspace containing the **Turtlebot_Collision_Avoidance** package and run `catkin_make`.
2. ssh into your turtlebot, and ource `devel/setup.bash` inside the **Turtlebot_Collision_Avoidance** package on the turtlebot
2. Check that the **Turtlebot_Collision_Avoidance** package can be found by ROS by doing a `rospack find Turtlebot_Collision_Avoidance`. This should return the path of the package. If not, return to step 1 and try again.
3. run `roslaunch turltebot_collision_avoidance custom.launch`
4. You should see a lot of bringup messages. Make sure there are no fatal errors. If there are, it's likely certain packages are not installed.
5. You should be able to drive your turtlebot in teleoperated mode.

#### After completing previous steps:
1. On our local machine, run `main.py` inside the `/scripts` directory in the **Turtlebot_Collision_Avoidance** package. This should load two plots.
