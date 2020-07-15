#!/usr/bin/env python
import rospy
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from smach import State
from state_outcomes import WT_FOUND, WT_NOT_FOUND, ERROR


class FlyToEstimatedTurbinePosition(State):
    """
    Flies the drone to the closest wind turbine at a given altitude.
    Tests if the tower is within range of the lidar sensors.
    """

    def __init__(self):
        State.__init__(self, outcomes=[WT_FOUND, WT_NOT_FOUND, ERROR], input_keys=['ftetp_input'],
                       output_keys=['ftetp_output'])
        # subscribe to estimated position
        # init publisher to navigation (gps)
        # subscribe to lidar sensors
        # Publisher
        pass

    def execute(self, userdata):
        print("\n\nEXECUTING FLY TO ESTIMATED TURBINE POSITION")
        print(userdata['ftetp_input'])
        data = userdata['ftetp_input']
        self.nav_pub = rospy.Publisher(
            '/%s/mavros/setpoint_position/global' % data.namespace,
            PoseStamped,
            queue_size=10)

        rospy.Subscriber('/edge_wt_detector', Range, self.detect_turbine, queue_size=10)
        self.nav_to_est_wt_pos(data.estimated_turbine_position)
        if True:
            return WT_FOUND
        elif True:
            return WT_NOT_FOUND
        else:
            return ERROR

    def nav_to_est_wt_pos(self, est_pos):
        ex_point = GeoPoseStamped()
        self.nav_pub.publish(est_pos)

    def detect_turbine(self, msg):
        print('DETECTING TURBINE: ', msg)
