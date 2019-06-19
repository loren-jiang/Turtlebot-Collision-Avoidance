#!/usr/bin/env python

#imports go here
import rospy
from geometry_msgs.msg import Twist
from utils import *
import tf
import tf.transformations as tfs
import numpy as np 
import scipy as scp 
import math
# import teleop_custom as tele
import dynamic_reconfigure.client as dyn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 
from lidarClass import Lidar
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2 #may want to use point cloud...not sure yet

from std_msgs.msg import Empty
from time import time

# global variables go here
ROS_RATE = 10 # frequency Hz?

## INITIALIZE TOPICS TO PUBLISH TO
cmd_vel_auto = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10) # autonomous
# cmd_vel_auto = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10) # autonomous
# cmd_vel_auto = rospy.Publisher('/cmd_vel_mux/output', Twist, queue_size=10) # autonomous

# cmd_vel_tele = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10) # teleop
# set up the odometry reset publisher
reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)


## INITIALIZE RECONFIGURABLE NODES
# rospy.init_node('yaml_reconfig')
tb_mode_client = dyn.Client('/cmd_vel_mux') # setup client instance of node to reconfigure


TARGET_SPEED = 1.0 # m/s

# NUMPY PARAMETERS
np.set_printoptions(threshold='nan')


#"VFH" algorithm
def compute_cmd(state):
    ret_twist = 0
    return ret_twist

def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n


def main():
    global cmd_vel, ROS_RATE
    rospy.init_node('scan_to_cmd', anonymous=False)
    rospy.loginfo("To stop TurtleBot CTRL + C")
    rospy.on_shutdown(shutdown)

    # reset odometry (these messages take a few iterations to get through)
    timer = time()
    while time() - timer < 0.25:
        reset_odom.publish(Empty())

    #new lidar object
    lidar = Lidar()

    # setting up the transform listener to find turtlebot position
    listener = tf.TransformListener()
    from_frame = 'odom'
    to_frame = 'base_link'
    listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(5.0))
    broadcaster = tf.TransformBroadcaster()

    # this is so that each loop of the while loop takes the same amount of time.  The controller works better
    # if you have this here
    rate = rospy.Rate(ROS_RATE) 

    #direction from pixel to turtlebot pos vector
    beta = 0
    
    #constants chosen such that a - b*d_max = 0
    a = 10
    b = 1
    d_max = 10.0

    # certainty value
    c = 1

    while not rospy.is_shutdown():
        # getting the position of the turtlebot
        robot_pos, robot_rot = listener.lookupTransform(from_frame, to_frame, listener.getLatestCommonTime(from_frame, to_frame))
    
        # 3x1 array, representing (x,y,theta) of robot starting state
        robot_state = np.array([robot_pos[0], robot_pos[1], robot_rot[2]])

        #define goal state as in front of the current state
        goal_dist = np.array([1.5, 0])
        robot_rot = tf.transformations.euler_from_quaternion(robot_rot) #should be from -pi to pi as euler
        rot2d = rotation2d(robot_rot[2]) #just care about yaw
        goal_from_world = np.dot(rot2d, goal_dist)
        # need to double check if this is the correct transformation
        goal_state = np.array([goal_from_world[0] + robot_state[0], goal_from_world[1] + robot_state[1], robot_rot[2]])
      
        #IF WE WANT POINT CLOUD2
        # if lidar.xyz_generator is not None:
        #   for point in lidar.xyz_generator:
        #       cloud_pt = np.array(point) #[x y z], we also dont need z probably
        #       beta = math.atan2(cloud_pt[1]-robot_state[1], cloud_pt[0]-robot_state[0])

        #       d = np.linalg.norm(cloud_pt) #distance from turtlebot center to cloud point
                
        #       mag = c*c * (a - b*d) #magnitude of pixel to turtlebot pos vector
        #       print(d)
        if not lidar.ranges:
            continue
        else:
            # clean data optimized:
            angles = np.arange(lidar.angle_min, lidar.angle_max, lidar.angle_increment)

            # linear interpolation to replace nan values in laser scan
            y = np.asarray(lidar.ranges)
            nans, x = nan_helper(y)
            y[nans] = np.interp(x(nans), x(~nans), y[~nans])

            # combining interpolated data with angles
            combined = zip(y, angles)
            clean_data = filter(lambda da: not np.isnan(da[0]), combined)
            inverted_data = np.array(list(map(lambda x: tuple((1/x[0]**2, x[1])), clean_data))) #inveting distances^2

            # print("-------------------- LIDAR INVERTED RANGES --------------------------")
            # print("-------------------------------------------------------")
            # print(inverted_data)
            # # print(list(clean_data))
            # print("-------------------------------------------------------")
            # print("------------------ END LIDAR INVERTED RANGES ----------------------")
            # #clean_data = list(map(lambda x:(a - b*x[0],x[1]), clean_data)) 
            # print("-------------------------------------------------------")
            # print("ROBOT STATE: " + str(robot_state))
            # print("-------------------------------------------------------")
            # print("GOAL STATE: " +str(goal_state))
            # for (dist, ang) in clean_data:

            # move_cmd = compute_cmd(robot_state)

            obstacles = list(filter(lambda data: data[0] > 2, inverted_data)) # considers any data point with inverted depth greater than .5 an obstacle

            print("OBSTACLES: " + str(obstacles))

        # TODO: Add a case if all inverted data is nan 
        if len(obstacles) > 10:
            auto_params = {'yaml_cfg_file': '/opt/ros/kinetic/share/turtlebot_bringup/param/auto_mux.yaml', 'yaml_cfg_data': '', 'groups': {'groups': {}, 'state': True, 'yaml_cfg_data': '', 'name': 'Default', 'parent': 0, 'parameters': {}, 'type': '', 'id': 0, 'yaml_cfg_file': '/opt/ros/kinetic/share/turtlebot_bringup/param/mux.yaml'}}
            tb_mode_client.update_configuration(auto_params)
            print("AUTONOMOUS")

            # THIS IS WHERE ALGORITHM GOES - STOPPING FOR NOW
            twist_message = Twist()
            twist_message.linear.x = 0
            twist_message.angular.z = 0
            cmd_vel_auto.publish(twist_message)
        else:
            teleop_params = {'yaml_cfg_file': '/opt/ros/kinetic/share/turtlebot_bringup/param/mux.yaml', 'yaml_cfg_data': '', 'groups': {'groups': {}, 'state': True, 'yaml_cfg_data': '', 'name': 'Default', 'parent': 0, 'parameters': {}, 'type': '', 'id': 0, 'yaml_cfg_file': '/opt/ros/kinetic/share/turtlebot_bringup/param/mux.yaml'}}
            tb_mode_client.update_configuration(teleop_params)
            print("TELEOP")

        # END LOOP 
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")

            
        rate.sleep()    

def shutdown():
    global cmd_vel
    rospy.loginfo("Stopping TurtleBot")
    cmd_vel_auto.publish(Twist())
    # cmd_vel_tele.publish(Twist())
    rospy.sleep(1)

if __name__ == '__main__':
    main()

rospy.loginfo("scan_to_cmd node terminated.")
