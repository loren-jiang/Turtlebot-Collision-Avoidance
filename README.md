# Turtlebot_Collision_Avoidance
This repo contains the source code for my EEC106B Robotics and Manipulation final project.

## Setup Instructions
This code is based on the ROS platform. For more information, please visit [here](ros.org) for documentation. NOTE: This project was built in Ubuntu 14.04 on ROS Indigo and ROS Kinetic for the Turtlebot

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
