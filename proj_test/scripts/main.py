#!/usr/bin/env python

#math tools
from utils import *
import numpy as np 
np.set_printoptions(threshold='nan')
import scipy.signal as sig
import math

#ROS stuff
import rospy
import tf
import tf.transformations as tfs
from std_msgs.msg import Empty
from time import time
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan 
import laserscan
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2 #may want to use point cloud...not sure yet
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

#plotting tools
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

# global var
valley_mag_thresh = 0.4 #NEED TO SET THESE
valley_ind_thresh = 20.0 * 0.00171110546216 #num pixels * angle increment

filt_scan_flattened = np.zeros(1280)
robot_state = np.zeros(3) #global turtlebot state
goal_state = np.zeros(3) #global goal state
target_speed = np.array(.2) # m/s?
#butterworth for smoothing
b_filt, a_filt = sig.butter(4, 0.03, analog=False)

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
rate_ctrl_loop = rospy.Rate(1000) 
rate_main_loop = rospy.Rate(10000000)

cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10) #autonomous
prev_twist = Twist()
prev_goal_state = np.zeros(3)

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
#returns the index of the valleys
def find_valleys(data):
    cropped_data = data[data[:,0] < valley_mag_thresh]
    k = 0
    valley_angles = []
    valley_widths = []
    cum_sum = 0
    ct = 0
    while (k < len(cropped_data) - 1):
        cum_sum += cropped_data[k][1]
        ct += 1
        if abs(cropped_data[k][1] - cropped_data[k+1][1]) > valley_ind_thresh or k == len(cropped_data) - 2: # if there is a big jump in the angles > valley_ind_thresh
            cum_avg = cum_sum / ct
            valley_angles.append(cum_avg)
            valley_widths.append(ct)
            cum_sum = 0
            ct = 0
        k += 1
    return valley_angles, valley_widths

#modified"VFH" algorithm
#data --> np array of 1/depth^2 and angle as tuple
#state --> robot state
def compute_goal(state, data):
    #valley_inds = scipy.signal.find_peaks_cwt(-data[:][0]) #need to include neg sign to be able to find "peaks"
    #print(-data[:,0])

    valley_angles, valley_widths = find_valleys(data) #need to include neg sign to be able to find "peaks"
    print("-----------------------------valley_angles--------------------------------")
    print(valley_angles)
    print("\n")

    # print("-----------------------------valley_widths--------------------------------")
    # print(valley_widths)
    # print("\n")
    final_goal = np.zeros(3) 
    goal_dist = np.array([0.1, 0]) #goal dist in robot frame [meters]


    #PICKING WIDEST VALLEY
    k = 0
    max_width_ind = 0
    max_width = 0
    while(k<len(valley_widths)):
        if max_width < valley_widths[k]:
            max_width = valley_widths[k]
            max_width_ind = k
        k +=1
    if valley_angles:
        heading = valley_angles[max_width_ind]
    else:
        heading = 0
    print("------------------------------heading------------------------------------")    
    print(heading)
    print("\n")
    rot2d = rotation2d(heading) #just care about yaw
    goal_from_world = np.dot(rot2d, goal_dist)
    # need to double check if this is the correct transformation
    final_goal = np.array([goal_from_world[0] + state[0], goal_from_world[1] + state[1], state[2]])

    # PICKIGN VALLEY NEAREST TO AHEAD GOAL...NEED TO FIX IF STATEMENT FOR NEXT GOAL!!!
    # for heading in valley_angles:
    #     rot2d = rotation2d(heading) #just care about yaw
    #     goal_from_world = np.dot(rot2d, goal_dist)
    #     # need to double check if this is the correct transformation
    #     next_goal = np.array([goal_from_world[0] + state[0], goal_from_world[1] + state[1], state[2]])
    #     if (euclidean_dist(goal_state[:2], next_goal[:2]) < euclidean_dist(goal_state[:2], final_goal[:2])):
    #         final_goal = next_goal

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
    kp_lin_vel = 1.5
    kp_ang_vel = 6
    distance_tolerance = 0.03 #[meters]

    while (euclidean_dist(robot_state[:2], final_state[:2]) >= distance_tolerance):
        robot_state = get_robot_state()
        print("ROBOT STATE:" + str(robot_state))
        print("FINAL STATE:" + str(final_state))
        steer_ang = math.atan2(final_state[1] - robot_state[1], final_state[0] - robot_state[0])
        error = final_state - robot_state
        twist.linear.x = kp_lin_vel * (error[0])
        twist.angular.z = kp_ang_vel * (steer_ang - robot_state[2])
        cmd_vel.publish(twist)
        rate_ctrl_loop.sleep() 
    
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

def callback(msg):
    global filt_scan_flattened
    if msg.data.any():
        filt_scan_flattened = msg.data

def main():
    global cmd_vel, ROS_RATE, goal_state, robot_state, from_frame, to_frame, prev_goal_state
    
    #shutdown
    rospy.on_shutdown(shutdown)

    #create new laser_scan class
    laser_scan = laserscan.LaserScan()

    # set up the odometry reset publisher
    reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    # reset odometry (these messages take a few iterations to get through)
    timer = time()
    while time() - timer < 0.5:
        reset_odom.publish(Empty())

    # subscribed to filtered laser scan data
    #scan_sub = rospy.Subscriber('/filtered_laserscan', numpy_msg(Floats), callback)


    # move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # rospy.loginfo("Wait for the action server to come up")
    # # Allow up to 5 seconds for the action server to come up
    # move_base.wait_for_server(rospy.Duration(5))

    
    #data visualization
    plt.ion() ## Note this correction
    fig=plt.figure()

    while not rospy.is_shutdown():
        # 3x1 array, representing (x,y,theta) of robot starting state
        robot_state = get_robot_state()
        # ranges = filt_scan_flattened[0:640]
        # angles = filt_scan_flattened[640:]
        # combined = np.column_stack((ranges, angles))
        
        combined = laser_scan.combined_data
        #define goal state as in front of the current state
        goal_dist = np.array([1.5, 0])
        rot2d = rotation2d(robot_state[2]) 
        goal_from_world = np.dot(rot2d, goal_dist)

        ideal_goal_state = np.array([goal_from_world[0] + robot_state[0], goal_from_world[1] + robot_state[1], robot_state[2]])
        #IF WE WANT POINT CLOUD2
        # if lidar.xyz_generator is not None:
        #   for point in lidar.xyz_generator:
        #       cloud_pt = np.array(point) #[x y z], we also dont need z probably
        #       beta = math.atan2(cloud_pt[1]-robot_state[1], cloud_pt[0]-robot_state[0])

        #       d = np.linalg.norm(cloud_pt) #distance from turtlebot center to cloud point
                
        #       mag = c*c * (a - b*d) #magnitude of pixel to turtlebot pos vector
        #       print(d)
        if not combined[:,0].any():
            print("BAD DATA")
            continue
        else:
            goal_state = compute_goal(robot_state, combined) #find the goal (best "valley" to go to)
            #move_base.send_goal(move_goal)
            #if euclidean_dist(prev_goal_state[:2],goal_state[:2]) > .25:
            if abs(prev_goal_state[2]-goal_state[2]) > .6:
                goal_state = prev_goal_state

            # print("-----------------------robot_state--------------------------------")
            # print(robot_state)
            # print("\n")
            # print("-----------------------goal_state--------------------------------")
            # print(goal_state)
            # print("\n")

            compute_twist_and_move(goal_state)
            
            prev_goal_state = goal_state
        #     plt.plot(combined[:,1], combined[:,0])
        #     plt.title("Laserscan data, interpolated and smoothed")
        #     plt.xlabel("Angle [radians]")
        #     plt.ylabel("Depth metric "  + r"$[1 / depth^2]$")
        #     plt.draw()
        #     plt.pause(0.0001) 
        #     plt.clf()
        # rospy.sleep(0.1)
        # print(time())
        rate_main_loop.sleep()    

def shutdown():
    global cmd_vel
    rospy.loginfo("Stopping TurtleBot")
    cmd_vel.publish(Twist())
    rospy.sleep(1)

if __name__ == '__main__':
    main()

rospy.loginfo("scan_to_cmd node terminated.")
