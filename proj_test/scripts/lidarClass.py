#!/usr/bin/env python

#imports go here
import rospy
import sensor_msgs.msg 
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2 #may want to use point cloud...not sure yet

class Lidar():
    def __init__(self, scan_topic="/scan"):
        self.scan_sub = rospy.Subscriber(scan_topic, sensor_msgs.msg.LaserScan, self.on_scan)
        self.laser_projector = LaserProjection()
        self.msg = None
        self.distances = None
        self.cloud = None
        self.scans = None
        self.xyz_generator = None
        self.angle_min = -.547
        self.angle_max = .547
        self.angle_increment = 0.00171110546216
        self.ranges = None
       

    def on_scan(self, msg):
        msg.range_max = 10 #[meter] 6 seems to  be decent
        msg.range_min = 0.5 
        self.msg = msg
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment

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
