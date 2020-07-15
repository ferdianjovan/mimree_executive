#!/usr/bin/env python


import actionlib
import rospy
from mimree_executive.msg import FindTurbineOdomGoal, FindTurbineOdomAction
from geographic_msgs.msg import GeoPoseStamped
# print("FINISHED IMPORTS")
minute = 60
if __name__ == '__main__':
    print("INITIATING")
    rospy.init_node('find_turbine_odom_client')
    client = actionlib.SimpleActionClient('find_turbine_odom', FindTurbineOdomAction)
    client.wait_for_server()
    print("SERVER CONNECTED")
    goal = FindTurbineOdomGoal()
    goal.turbine_estimated_position=GeoPoseStamped()
    goal.uav_namespace='hector'
    print("GOAL CREATED: ",goal)
    client.send_goal(goal)
    print("GOAL SENT")
    client.wait_for_result(rospy.Duration(3 * minute))
    print("RESULT RECIEVED")
