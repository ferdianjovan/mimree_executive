#!/usr/bin/env python
import rospy
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import Range
from smach import State
from state_outcomes import WT_FOUND, WT_NOT_FOUND, ERROR


class FlyToEstimatedTurbinePosition(State):
    """
    Flies the drone to the closest wind turbine at a given altitude.
    Tests if the tower is within range of the lidar sensors.
    """

    def __init__(self, keys):
        State.__init__(self, outcomes=[WT_FOUND, WT_NOT_FOUND, ERROR], input_keys=keys,
                       output_keys=keys)
        # subscribe to estimated position
        # init publisher to navigation (gps)
        # subscribe to lidar sensors
        # Publisher
        pass

    def execute(self, userdata):
        print("\n\n\n", " EXECUTING FLY TO ESTIMATED TURBINE POSITION: ", userdata.keys())
        print("\n\n", userdata)
        data = userdata
        self.nav_pub = rospy.Publisher(
            '/%s/mavros/setpoint_position/global' % data['uav_namespace'],
            GeoPoseStamped,
            queue_size=10)

        rospy.Subscriber('/edge_wt_detector', Range, self.detect_turbine, queue_size=10)
        self.nav_to_est_wt_pos(data.turbine_estimated_position)
        if True:
            return WT_FOUND
        elif True:
            return WT_NOT_FOUND
        else:
            return ERROR

    def nav_to_est_wt_pos(self, est_pos):
        print(est_pos)
        ex_point = GeoPoseStamped()
        ex_point.pose.position = est_pos.pose.position
        ex_point.pose.orientation = est_pos.pose.orientation
        self.nav_pub.publish(est_pos)

    def detect_turbine(self, msg):
        print('DETECTING TURBINE: ', msg)
