#!/usr/bin/env python


import actionlib
import rospy
from geographic_msgs.msg import GeoPoseStamped
from mimree_executive.msg import FindTurbineOdomGoal, FindTurbineOdomAction

# print("FINISHED IMPORTS")
minute = 60
if __name__ == '__main__':
    print("INITIATING")
    rospy.init_node('find_turbine_odom_client')
    client = actionlib.SimpleActionClient('find_turbine_odom', FindTurbineOdomAction)
    client.wait_for_server()
    print("SERVER CONNECTED")
    goal = FindTurbineOdomGoal()
    goal.turbine_estimated_position = GeoPoseStamped()
    # Test coordinates, flies near turbine
    # latitude: 43.7993805
    # longitude: 28.5936549
    # altitude: 90.0
    goal.turbine_estimated_position.pose.position.latitude = 43.7993805
    goal.turbine_estimated_position.pose.position.longitude = 28.5936549
    goal.turbine_estimated_position.pose.position.altitude = 90
    goal.turbine_estimated_position.pose.orientation.z = 0.70710678
    goal.turbine_estimated_position.header.frame_id='map'
    goal.turbine_estimated_position.header.stamp.secs=8070
    goal.turbine_estimated_position.header.stamp.nsecs=0
    goal.turbine_estimated_position.header.seq=1

    goal.uav_namespace = 'hector'
    print("GOAL CREATED: ", goal)
    client.send_goal(goal)
    print("GOAL SENT")
    client.wait_for_result(rospy.Duration(3 * minute))
    print("RESULT RECIEVED")
