#!/usr/bin/env python

import roslib
import smach
from states.identify_turbine_odom_orientation import IdentifyTurbineOdomOrientation
from states.identify_turbine_odom_position import IdentifyTurbineOdomPosition
from states.fly_to_estimated_turbine_position import FlyToEstimatedTurbinePosition

# roslib.load_manifest('my_pkg_name')
import rospy
import actionlib
from mimree_executive.msg import FindTurbineOdomAction


class FindTurbineOdomServer:
    """
    The Turbine Odom is defined as follows:
    position: outermost center of the three wings
    orientation: the way the turbine is facing
    This is using gps positioning.
    """

    def __init__(self):
        self.server = actionlib.SimpleActionServer('find_turbine_odom', FindTurbineOdomAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        # A SMACH implementation of the following tasks:
        rospy.loginfo("EXECUTING FIND TURBINE SERVER: ",goal)
        print('EXEC Find Turbine')
        print(goal)
        self.namespace = goal.namespace
        self.turbine_estimated_position=goal.turbine_estimated_position
        sm = smach.StateMachine(outcomes=['no_turbine_detected', 'turbine_odom_identified'])
        with sm:
            # drone flight to estimated position of turbine

            smach.StateMachine.add('FlyToEstimatedTurbinePosition', FlyToEstimatedTurbinePosition(),
                                   transitions={
                                       'near_turbine': 'IdentifyTurbineOdomOrientation'})
            # drone identification of turbine orientation (main column is in front of or behind wings & face)

            smach.StateMachine.add('IdentifyTurbineOdomOrientation', IdentifyTurbineOdomOrientation(),
                                   transitions={'turbine_odom_identified': 'IdentifyTurbineOdomPosition'})
            # drone identification of the pose for the turbine center
            smach.StateMachine.add('IdentifyTurbineOdomPosition', IdentifyTurbineOdomPosition(),
                                   transitions={})
        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('mission_find_turbine_odom_server')
    server = FindTurbineOdomServer()
    rospy.spin()
