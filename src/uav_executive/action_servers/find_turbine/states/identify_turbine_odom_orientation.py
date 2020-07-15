#!/usr/bin/env python
import rospy
import smach

from state_outcomes import WT_ODOM_ORIENTATION_FOUND, ERROR


class IdentifyTurbineOdomOrientation(smach.State):
    """
    Flies the drone to the closest wind turbine at a given altitude.
    Tests if the tower is within range of the lidar sensors.
    """

    def __init__(self):
        self._outcomes = [WT_ODOM_ORIENTATION_FOUND, ERROR]
        pass

    def execute(self, userdata):
        print(self._outcomes)
        print(userdata)
        rospy.loginfo('Executing state IdentifyTurbineOdomOrientation')
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
