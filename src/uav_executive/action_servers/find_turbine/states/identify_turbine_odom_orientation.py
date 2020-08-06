#!/usr/bin/env python
import time

import rospy
import smach
from geometry_msgs.msg import TwistStamped
from smach import State
from state_outcomes import WT_ODOM_ORIENTATION_FOUND, ERROR
from std_msgs.msg import Float64


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
        pass

    def update_heading(self, msg):
        self.heading = msg.data

    def heading_within_range(self, heading, thresh):
        return not (self.heading < (heading - thresh) % 360 or self.heading > (heading + thresh) % 360)

    def spin_drone(self):
        print("spinning drone")
        twist = TwistStamped()
        twist.twist.angular.z = -0.1
        twist.twist.angular.x = 0
        twist.twist.angular.y = 0
        heading_start = self.heading
        has_passed = False
        print('STARTING HEADING:', heading_start)
        while (not has_passed) or not (self.heading_within_range(heading_start, 5)):
            print(self.heading, (not has_passed), not (self.heading_within_range(heading_start, 5)))
            if not has_passed and not self.heading_within_range(heading_start, 5):
                print("PASSED THRESHOLD")
                has_passed = True
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        print('ENDED: ', self.heading, (not has_passed), not (self.heading_within_range(heading_start, 5)))

        # send a spin message

    def execute(self, userdata):
        time.sleep(2)
        print("EXECUTING IDENTIFY TURBINE ODOM ;D")
        # self.nav_pub = rospy.Publisher(
        #     '/%s/mavros/setpoint_position/global' % userdata.namespace,
        #     PoseStamped,
        #     queue_size=10)
        #
        # rospy.Subscriber('/edge_wt_detector', Range, self.detect_turbine, queue_size=10)
        # self.nav_to_est_wt_pos(userdata.estimated_turbine_position)
        #
        # # navigate to within x meters of estimated position
        # # check for lidar values at same height
        # # return self.outcomes[0]
        self.spin_drone()
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
