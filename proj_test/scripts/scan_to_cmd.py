#!/usr/bin/env python

#math tools
from utils import *
import numpy as np 
import scipy 
import math

#ROS stuff
import rospy
import tf
import tf.transformations as tfs
from std_msgs.msg import Empty
from time import time
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan 
from lidarClass import Lidar
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2 #may want to use point cloud...not sure yet
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *

#plotting tools
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

# global var
valley_mag_thresh = 0 #NEED TO SET THESE
valley_ind_thresh = 0

robot_state = np.zeros(3) #global turtlebot state
goal_state = np.zeros(3) #global goal state
target_speed = np.array(.2) # m/s?
ROS_RATE = 10 # frequency Hz?

#initializations
rospy.init_node('scan_to_cmd', anonymous=False)
rospy.loginfo("To stop TurtleBot CTRL + C")
# setting up the transform listener to find turtlebot position
from_frame = 'odom'
to_frame = 'base_link'
listener = tf.TransformListener()
listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(5.0))
broadcaster = tf.TransformBroadcaster()
# this is so that each loop of the while loop takes the same amount of time.  The controller works better
# if you have this here
rate = rospy.Rate(ROS_RATE) 

cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10) #autonomous

def nan_helper(y):
    """Helper to handle indices and logical indices of NaNs.

    Input:
        - y, 1d numpy array with possible NaNs
    Output:
        - nans, logical indices of NaNs
        - index, a function, with signature indices= index(logical_indices),
          to convert logical indices of NaNs to 'equivalent' indices
    Example:
        >>> # linear interpolation of NaNs
        >>> nans, x= nan_helper(y)
        >>> y[nans]= np.interp(x(nans), x(~nans), y[~nans])
    """
    return np.isnan(y), lambda z: z.nonzero()[0]

def euclidean_dist(pos1, pos2):
    """Euclidean distance between current pose and the goal."""
    return np.linalg.norm(pos1 - pos2)

# finds the possible valleys to take based on threshold values and closest to goal
def find_valleys(depths):
    return 0
#modified"VFH" algorithm
#data --> np array of 1/depth^2 and angle as tuple
#state --> robot state
def compute_goal(state, data):
    #valley_inds = scipy.signal.find_peaks_cwt(-data[:][0]) #need to include neg sign to be able to find "peaks"
    #print(-data[:,0])

    valley_inds - find_valleys(-data[:,0]) #need to include neg sign to be able to find "peaks"

    valley_inds = np.array([1, 40, 50]) #testing purposes
    #print(valley_inds)
    headings =  data[valley_inds][:,1] #possible headings to choose from
    final_goal = np.zeros(3) 
    goal_dist = np.array([0.05, 0]) #goal dist in robot frame [meters]
    for heading in headings:
        rot2d = rotation2d(heading) #just care about yaw
        goal_from_world = np.dot(rot2d, goal_dist)
        # need to double check if this is the correct transformation
        next_goal = np.array([goal_from_world[0] + state[0], goal_from_world[1] + state[1], state[2]])
        if (euclidean_dist(goal_state[:2], next_goal[:2]) < euclidean_dist(goal_state[:2], final_goal[:2])):
            final_goal = next_goal

    # goal = MoveBaseGoal()
    # goal.target_pose.header.frame_id = 'base_link'
    # goal.target_pose.header.stamp = rospy.Time.now()
    # goal_pt = Point(final_goal[0], final_goal[1], 0)
    # np_quat = tf.transformations.quaternion_from_euler(0, 0, final_goal[2])
    # goal_quat = Quaternion(np_quat[0], np_quat[1], np_quat[2], np_quat[3])
    # goal.target_pose.pose = Pose(goal_pt, goal_quat)
    # print(goal)
    return final_goal #desired goal pose!

#compute desired twist using a simple P controller...for now
def compute_twist_and_move(final_state):
    global cmd_vel, robot_state
    twist = Twist()

    #gains which NEED TO BE TUNED
    kp_lin_vel = .75
    kp_ang_vel = 2.5
    distance_tolerance = 0.025
    final_state = np.array(([.5,.5,0]))
    while (euclidean_dist(robot_state[:2], final_state[:2]) >= distance_tolerance):
        robot_state = get_robot_state()
        print("ROBOT STATE:" + str(robot_state))
        print("FINAL STATE:" + str(final_state))
        steer_ang = math.atan2(final_state[1] - robot_state[1], final_state[0] - robot_state[0])
        error = final_state - robot_state
        twist.linear.x = kp_lin_vel * (error[0])
        twist.angular.z = kp_ang_vel * (steer_ang - robot_state[2])
        cmd_vel.publish(twist)
        rate.sleep()
    
def get_robot_state():
    # getting the position of the turtlebot
    global robot_state
    robot_pos, robot_rot = listener.lookupTransform(from_frame, to_frame, listener.getLatestCommonTime(from_frame, to_frame))
    #broadcaster.sendTransform(robot_pos, robot_rot, listener.getLatestCommonTime(from_frame, to_frame), to_frame, from_frame) #this seems to mess things up...not sure why
    robot_rot = tf.transformations.euler_from_quaternion(robot_rot) #should be from -pi to pi as euler
    # 3x1 array, representing (x,y,theta) of robot starting state
    robot_state = np.array([robot_pos[0], robot_pos[1], robot_rot[2]])
    return robot_state

def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n


def main():
    global cmd_vel, ROS_RATE, goal_state, robot_state, from_frame, to_frame
    
    #shutdown
    rospy.on_shutdown(shutdown)

    # set up the odometry reset publisher
    reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

    # reset odometry (these messages take a few iterations to get through)
    timer = time()
    while time() - timer < 0.4:
        reset_odom.publish(Empty())

    # move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # rospy.loginfo("Wait for the action server to come up")
    # # Allow up to 5 seconds for the action server to come up
    # move_base.wait_for_server(rospy.Duration(5))

    #new lidar object
    lidar = Lidar()

    #direction from pixel to turtlebot pos vector
    beta = 0
    
    #constants chosen such that a - b*d_max = 0
    a = 10
    b = 1
    d_max = 10.0

    # certainty value
    c = 1

    #data visualization
    plt.ion() ## Note this correction
    fig=plt.figure()

    while not rospy.is_shutdown():
        # 3x1 array, representing (x,y,theta) of robot starting state
        robot_state = get_robot_state()

        #define goal state as in front of the current state
        goal_dist = np.array([1.5, 0])
        rot2d = rotation2d(robot_state[2]) 
        goal_from_world = np.dot(rot2d, goal_dist)
        # need to double check if this is the correct transformation
        goal_state = np.array([goal_from_world[0] + robot_state[0], goal_from_world[1] + robot_state[1], robot_state[2]])
        #IF WE WANT POINT CLOUD2
        # if lidar.xyz_generator is not None:
        #   for point in lidar.xyz_generator:
        #       cloud_pt = np.array(point) #[x y z], we also dont need z probably
        #       beta = math.atan2(cloud_pt[1]-robot_state[1], cloud_pt[0]-robot_state[0])

        #       d = np.linalg.norm(cloud_pt) #distance from turtlebot center to cloud point
                
        #       mag = c*c * (a - b*d) #magnitude of pixel to turtlebot pos vector
        #       print(d)

        if not lidar.ranges or all(np.isnan(lidar.ranges)):
            continue
        else:
            #clean the scan data and compute angle
            # clean data optimized:
            angles = np.arange(lidar.angle_min, lidar.angle_max, lidar.angle_increment)

            #linear interpolation nan values in laser scan
            lidar_copy = np.asarray(lidar.ranges)
            nans, nonzero_filter = nan_helper(lidar_copy)
            
            lidar_copy[nans] = np.interp(nonzero_filter(nans), nonzero_filter(~nans), lidar_copy[~nans])

            combined = np.vstack((lidar_copy, angles)).T # shape  = (640, 2)
            tmp = 1./(combined[:,0] *combined[:,0] )
            combined[:,0] = tmp

            goal_state = compute_goal(robot_state, combined) #find the goal (best "valley" to go to)
            #move_base.send_goal(move_goal)

            compute_twist_and_move(goal_state)
            
            plt.plot(angles, 1./(lidar_copy*lidar_copy))
            plt.draw()
            plt.pause(0.0001) 
            plt.clf()
        #rospy.sleep(0.05)
        
        rate.sleep()    

def shutdown():
    global cmd_vel
    rospy.loginfo("Stopping TurtleBot")
    cmd_vel.publish(Twist())
    rospy.sleep(1)

if __name__ == '__main__':
    main()

rospy.loginfo("scan_to_cmd node terminated.")
