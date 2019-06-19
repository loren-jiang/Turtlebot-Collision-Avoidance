#!/usr/bin/env python

#imports go here
import rospy
import numpy as np
import sensor_msgs.msg 
from sensor_msgs.msg import LaserScan 
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2 #may want to use point cloud...not sure yet
import scipy.signal as sig
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import matplotlib.pyplot as plt
from utils import nan_helper
import dynamic_reconfigure.client as dyn

tb_mode_client = dyn.Client('/cmd_vel_mux') # setup client instance of node to reconfigure


class LaserScan():
    def __init__(self, scan_topic="/scan"):
        #rospy.init_node('LaserScan')
        #self.scan_pub = rospy.Publisher('/filtered_laserscan', numpy_msg(Floats), queue_size=10)
        self.scan_sub = rospy.Subscriber(scan_topic, sensor_msgs.msg.LaserScan, self.on_scan, queue_size = 1)
        self.laser_projector = LaserProjection()
        self.in_msg = None
        self.angle_min = -.547
        self.angle_max = .547
        self.angle_increment = 0.00171110546216
        self.angles = np.arange(self.angle_min, self.angle_max, self.angle_increment)
        self.out_msg = np.array([])
        self.b_filt, self.a_filt = sig.butter(4, 0.03, analog=False)
        self.combined_data = np.zeros((640,2))

    def publish(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.scan_pub.publish(self.out_msg)
        r.sleep()

    # def nan_helper(self, y):
    #     return np.isnan(y), lambda z: z.nonzero()[0]

    def on_scan(self, msg):
        msg.range_max = 10 #[meter] 6 seems to  be decent
        msg.range_min = 0.5 
        self.in_msg = msg
        #do the data processing
        #linear interpolation nan values in laser scan
        a = np.array([1.2, 2.3], dtype = np.float32)
        laserscan_copy = np.asarray(msg.ranges, dtype=np.float32)
        # print("LASER COPY: " + str(laserscan_copy))
        nans, nonzero_filter = nan_helper(laserscan_copy)
        # print("NANS: " + str(nans))
        try:
            laserscan_copy[nans] = np.interp(nonzero_filter(nans), nonzero_filter(~nans), laserscan_copy[~nans])
        except Exception as e:
            print(e)
            print("WALLLLLL")
            teleop_params = {'yaml_cfg_file': '/opt/ros/kinetic/share/turtlebot_bringup/param/mux.yaml', 'yaml_cfg_data': '', 'groups': {'groups': {}, 'state': True, 'yaml_cfg_data': '', 'name': 'Default', 'parent': 0, 'parameters': {}, 'type': '', 'id': 0, 'yaml_cfg_file': '/opt/ros/kinetic/share/turtlebot_bringup/param/mux.yaml'}}
            tb_mode_client.update_configuration(teleop_params)
            return
        combined = np.vstack((laserscan_copy, self.angles)).T # shape  = (640, 2)
        tmp = sig.filtfilt(self.b_filt,self.a_filt,1./(combined[:,0] *combined[:,0] ))
        #tmp = 1./(combined[:,0] *combined[:,0] ) #no fitlering
        combined[:,0] = tmp
        self.combined_data = combined
        #self.out_msg = combined.T.flatten().astype(np.float32)
        #self.out_msg = a
        # print(len(self.out_msg))
        # print(type(a))
        # print(type(self.out_msg[0]))

        # real-time plotting
        # plt.plot(self.angles, combined[:,0])
        # plt.title("Laserscan data, interpolated and smoothed")
        # plt.xlabel("Angle [radians]")
        # plt.ylabel("Depth metric "  + r"$[1 / depth^2]$")
        # plt.draw()
        # plt.pause(0.0001) 
        # plt.clf()
    

#if __name__ == '__main__':
    # new_laser = LaserScan()
    # new_laser.publish()


        # print(self.angle_increment) # default is 0.00171110546216
        #IF WE WANT POINT CLOUD2
        #rospy.loginfo("Got scan, projecting")
        #self.distances = self.laser_projector.projectLaser(scan, channel_options = 0x04)
        #self.cloud = self.laser_projector.projectLaser(scan)

        # print self.cloud
        #rospy.loginfo("Printed cloud")
        #gen = pc2.read_points(self.cloud, skip_nans=True, field_names=("x", "y", "z"))
        #gen = pc2.read_points(self.distances, skip_nans=True, field_names=("distances"))
        #self.xyz_generator = gen
