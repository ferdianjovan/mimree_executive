#! /usr/bin/env python
import laser_geometry.laser_geometry as lg
import rospy
from sensor_msgs.msg import PointCloud2, PointCloud,LaserScan
from sensor_msgs import point_cloud2
print("INITIATED ScanToPointCloud2")

import sys

import time
class ScanToPointCloud2:
    """
    Given a message containing a position x,y,z the drone will move to that position.
    I have decided to tie

    """

    def __init__(self):
        self.lp = lg.LaserProjection()
        print(sys.version)
        rospy.init_node('slam_cloud2')


        self.scan_sub = rospy.Subscriber('/hector/scanningLidar/laser_scan', LaserScan,
                                         self.convert_and_publish)  # make node
        self.pc2_pub = rospy.Publisher('/PointCloud2', PointCloud2, queue_size=1)
        # self.scan_sub2 = rospy.Subscriber('/sonar_height', LaserScan, self.convert_and_publish)  # make node

    def convert_and_publish(self, msg):
        self.publish_pc2(self.lp.projectLaser(msg))

    def process_kinect(self, pc2):
        print("recieved: ", type(pc2))
        self.publish_pc2(pc2)

    def publish_pc2(self, pc2):
        print(type(pc2))

        self.pc2_pub.publish(pc2)


if __name__ == '__main__':
    ScanToPointCloud2()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
