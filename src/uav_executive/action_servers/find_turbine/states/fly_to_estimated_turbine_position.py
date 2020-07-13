#!/usr/bin/env python
import rospy
import smach
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import Range


class FlyToEstimatedTurbinePosition(smach.State):
    """
    Flies the drone to the closest wind turbine at a given altitude.
    Tests if the tower is within range of the lidar sensors.
    """

    def __init__(self, outcomes=['near_turbine', 'no_turbine_detected']):
        # subscribe to estimated position
        # init publisher to navigation (gps)
        # subscribe to lidar sensors
        # Publisher
        pass

    def execute(self, userdata):
        rospy.loginfo('Executing state FlyToEstimatedTurbinePosition')
        print(userdata)
        self.nav_pub = rospy.Publisher(
            '/%s/mavros/setpoint_position/global' % userdata.namespace,
            PoseStamped,
            queue_size=10)

        rospy.Subscriber('/edge_wt_detector', Range, self.detect_turbine, queue_size=10)
        self.nav_to_est_wt_pos(userdata.estimated_turbine_position)

        # navigate to within x meters of estimated position
        # check for lidar values at same height
        # return self.outcomes[0]
        pass

    def nav_to_est_wt_pos(self, est_pos):
        ex_point = GeoPoseStamped()
        self.nav_pub.publish(est_pos)

    def detect_turbine(self, msg):
        print('DETECTING TURBINE: ', msg)
