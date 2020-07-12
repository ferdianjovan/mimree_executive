#!/usr/bin/env python

import roslib

roslib.load_manifest('my_pkg_name')
import rospy
import actionlib
from mimree_executive.msg import FindTurbineOdomAction


class DoDishesServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('do_dishes', FindTurbineOdomAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        
        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('find_turbine_odom_server')
    server = DoDishesServer()
    rospy.spin()
