#!/usr/bin/env python
import time

import numpy as np
import ros_numpy
import rospy
import smach
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import PointCloud2
from smach import State
from state_outcomes import WT_ODOM_ORIENTATION_FOUND, ERROR
from std_msgs.msg import Float64

from .utils import get_k_means_clusters


class IdentifyTurbineOdomOrientation(smach.State):
    """
    Flies the drone to the closest wind turbine at a given altitude.
    Tests if the tower is within range of the lidar sensors.
    """

    def __init__(self, keys, uav_namespace):
        print("INITITATED FIND ODOM ORIENTATION")
        State.__init__(self, outcomes=[WT_ODOM_ORIENTATION_FOUND, ERROR], input_keys=keys,
                       output_keys=keys)
        self.cmd_vel_pub = rospy.Publisher('/' + uav_namespace + '/mavros/setpoint_velocity/cmd_vel', TwistStamped)
        self.heading = 0
        self.comp_sub = rospy.Subscriber('/' + uav_namespace + '/mavros/global_position/compass_hdg', Float64,
                                         self.update_heading)
        self.octomap_data = PointCloud2()
        self.octomap_data_last_change = time.time()
        self.pc2 = PointCloud2()

        self.octo_pc2_sub = rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, self.update_octo_pc2)
        self.pc2_sub = rospy.Subscriber('/PointCloud2', PointCloud2, self.update_pc2)

        pass

    def update_octo_pc2(self, msg):
        # print("UPDATING PC2: ", msg)
        if not msg.data == self.octomap_data.data:
            self.octomap_data_last_change = time.time()
        self.octomap_data = msg

    def update_heading(self, msg):
        self.heading = msg.data

    def update_pc2(self, msg):
        # print(len(msg.points))
        self.pc2 = msg

    def heading_within_range(self, heading, thresh):
        return not (self.heading < (heading - thresh) % 360 or self.heading > (heading + thresh) % 360)

    def scan_until_detect_lost(self, twist):
        print("SCANNING")
        # time since last change in octomap data > 1s.

        while time.time() - self.octomap_data_last_change < 1:
            self.cmd_vel_pub.publish(twist)
            self.scan_feature_extraction()
            time.sleep(0.1)
        print("END FOUND")

    def scan_until_detect(self, twist):
        print("WAITING TO DETECT")

        while time.time() - self.octomap_data_last_change > 1:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        print("DETECTED")

    def scan_wt(self):
        print("SCANNING WT")
        twist = TwistStamped()
        twist.twist.angular.z = -0.1
        twist.twist.angular.x = 0
        twist.twist.angular.y = 0
        self.scan_until_detect(twist)
        self.scan_until_detect_lost(twist)
        max_ang1 = self.heading
        print("FOUND EDGE1: ", max_ang1)
        twist.twist.angular.z = 0.1
        self.scan_until_detect(twist)
        self.scan_until_detect_lost(twist)
        max_ang2 = self.heading
        print("FOUND EDGE2: ", max_ang2)
        print(max_ang1, max_ang2)
        # print(self.octomap_data.data)

        # send a spin message

    def pose_array_to_np_array(self, poses):
        arr = []
        for p in poses:
            print(p)
            x, y, z = p.position.x, p.position.y, p.position.z
            arr.append([x, y, z])

        return np.array(arr)

    def analyze_pointcloud(self):
        print("analyzing point cloud: ")
        # print(self.octomap_data)

        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.octomap_data)

        get_k_means_clusters(xyz_array, 4)

    def scan_feature_extraction(self):
        """
        returns the prediction of what is currently being scanned with ranges.
        :param np_arr:
        :return:
        """
        # we only care about the z axis, we assume the wing is not vertical as that would be impossible to land on.
        # possible things to return are: two blades with the coordinates for each, one blade or the pylon.
        # assume wing mas width is 10m
        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.pc2)
        print("LENGTH: ", len(xyz_array))
        print(xyz_array)

        # print(xyz_array)
        h_vals = xyz_array[:, 0] * -1
        d_vals = xyz_array[:, 1]
        features = []
        h_c = -1000
        d_c = -1000
        # print(h_vals)
        print("------------------=========================")
        for index, h in enumerate(h_vals):
            d = d_vals[index]
            if abs(h - h_c) > 5 or abs(d - d_c) > 3:
                features.append(index)
            h_c = h
            d_c = d
            # print(h,d)
        print("features: ", features)
        return features
        # print(len(xyz_array))

    def execute(self, userdata):
        time.sleep(2)
        print("EXECUTING IDENTIFY TURBINE ODOM ;D")

        self.scan_wt()
        self.analyze_pointcloud()
        if True:
            return WT_ODOM_ORIENTATION_FOUND
        return ERROR

        pass

    def nav_to_est_wt_pos(self, est_pos):
        # ex_point = GeoPoseStamped()
        # self.nav_pub.publish(est_pos)
        pass

    def detect_turbine(self, msg):
        # print('DETECTING TURBINE: ', msg)
        pass
