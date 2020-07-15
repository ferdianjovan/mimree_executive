#!/usr/bin/env python
import rospy
import smach

from state_outcomes import WT_ODOM_POSITION_FOUND, ERROR


class IdentifyTurbineOdomPosition(smach.State):
    """
    Flies the drone to the closest wind turbine at a given altitude.
    Tests if the tower is within range of the lidar sensors.
    """

    def __init__(self):
        self._outcomes = [WT_ODOM_POSITION_FOUND,ERROR]

        pass

    def execute(self, userdata):
        rospy.loginfo('Executing state IdentifyTurbineOdomPosition')
        print(userdata)
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
            return WT_ODOM_POSITION_FOUND
        return ERROR

        pass
