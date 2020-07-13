#!/usr/bin/env python

import roslib

roslib.load_manifest('my_pkg_name')
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
        self.server = actionlib.SimpleActionServer('do_dishes', FindTurbineOdomAction, self.execute, False)
        self.server.start()


    def execute(self, goal):
        # A SMACH implementation of the following tasks:
        # drone flight to estimated position of turbine
        # drone identification of turbine orientation (main column is in front of or behind wings)
        # whilst ensuring that a safe distance is kept.
        # drone identification of the pose for the turbine center
        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('find_turbine_odom_server')
    server = FindTurbineOdomServer()
    rospy.spin()
