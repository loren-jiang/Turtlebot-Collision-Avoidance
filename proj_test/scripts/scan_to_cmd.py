#!/usr/bin/env python

#imports go here
import rospy
from geometry_msgs.msg import Twist
from utils import *
import tf
import tf.transformations as tfs
import numpy as np 
import scipy 
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 
from lidarClass import Lidar
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2 #may want to use point cloud...not sure yet
import matplotlib.pyplot as plt
from std_msgs.msg import Empty
from time import time

# global variables go here



ROS_RATE = 10 # frequency Hz?
cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10) #autonomous
# set up the odometry reset publisher
reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

valley_mag_thresh = 0 #NEED TO SET THESE
valley_ind_thresh = 0

goal_state = np.zeros(3) #global goal state
target_speed = np.array(.2) # m/s?


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

#"VFH" algorithm
#data --> np array of 1/depth^2 and angle as tuple
#state --> robot state
def compute_cmd(state, data):
    #valley_inds = scipy.signal.find_peaks_cwt(-data[:][0]) #need to include neg sign to be able to find "peaks"
    #print(-data[:,0])
    valley_inds = np.array([1, 40, 50]) #testing purposes
    #print(valley_inds)
    headings =  data[valley_inds][:,1] #possible headings to choose from
    print(headings)
    final_goal = np.zeros(3) 
    goal_dist = np.array([0.05, 0]) #goal dist in robot frame [meters]
    for heading in headings:
        rot2d = rotation2d(heading) #just care about yaw
        goal_from_world = np.dot(rot2d, goal_dist)
        # need to double check if this is the correct transformation
        next_goal = np.array([goal_from_world[0] + state[0], goal_from_world[1] + state[1], state[2]])

        if (np.linalg.norm(goal_state[:2] - next_goal[:2]) < np.linalg.norm(goal_state[:2] - final_goal[:2])):
            final_goal = next_goal

    ret_twist = Twist()

    # Need to figure out some control law to find appropriate twist
    ret_twist.linear.x = .1
    ret_twist.angular.z = final_goal[2]
    print(ret_twist)
    return ret_twist

def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n


def main():
    global cmd_vel, ROS_RATE, goal_state
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

    #data visualization
    plt.ion() ## Note this correction
    fig=plt.figure()

    while not rospy.is_shutdown():
        # getting the position of the turtlebot
        robot_pos, robot_rot = listener.lookupTransform(from_frame, to_frame, listener.getLatestCommonTime(from_frame, to_frame))

        #define goal state as in front of the current state
        goal_dist = np.array([1.5, 0])
        robot_rot = tf.transformations.euler_from_quaternion(robot_rot) #should be from -pi to pi as euler
        # 3x1 array, representing (x,y,theta) of robot starting state
        robot_state = np.array([robot_pos[0], robot_pos[1], robot_rot[2]])

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

            move_cmd = compute_cmd(robot_state, combined) #computes twist message to cmd_vel
            # twist_message = Twist()
            # twist_message.linear.x = 0.2
            # twist_message.angular.z = 0
            cmd_vel.publish(move_cmd)
            plt.plot(angles, 1./(lidar_copy*lidar_copy))
            plt.draw()
            plt.pause(0.0001) 
        #rospy.sleep(0.05)
        plt.clf()
        rate.sleep()    

def shutdown():
    global cmd_vel
    rospy.loginfo("Stopping TurtleBot")
    cmd_vel.publish(Twist())
    rospy.sleep(1)

if __name__ == '__main__':
    main()

rospy.loginfo("scan_to_cmd node terminated.")
