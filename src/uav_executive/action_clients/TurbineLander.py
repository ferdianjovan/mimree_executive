# ! /usr/bin/env python
import roslib

roslib.load_manifest('my_pkg_name')
import rospy
import actionlib
from mimree_executive.msg import FindTurbineOdomGoal, FindTurbineOdomAction


minute = 60
if __name__ == '__main__':
    rospy.init_node('find_turbine_odom_client')
    client = actionlib.SimpleActionClient('find_turbine_odom', FindTurbineOdomAction)
    client.wait_for_server()
    goal = FindTurbineOdomGoal()
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(3*minute))
