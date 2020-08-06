# !/usr/bin/env python
import time

import rospy
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseArray
from smach import State
from state_outcomes import WT_FOUND, WT_NOT_FOUND, ERROR


class FlyToEstimatedTurbinePosition(State):
    """
    Flies the drone to the closest wind turbine at a given altitude.
    Tests if the tower is within range of the lidar sensors.
    """

    def __init__(self, keys, uav_namespace):
        State.__init__(self, outcomes=[WT_FOUND, WT_NOT_FOUND, ERROR], input_keys=keys,
                       output_keys=keys)
        # subscribe to estimated position
        # init publisher to navigation (gps)
        # subscribe to lidar sensors
        # Publisher
        print("INITIATED FLY TO EST POS")
        uav_topic_name = '/%s/mavros/setpoint_position/global' % uav_namespace

        self.nav_pub = rospy.Publisher(
            uav_topic_name,
            GeoPoseStamped,
            queue_size=10)

        rospy.Subscriber('/edge_wt_detector', PoseArray, self.detect_turbine, queue_size=10)
        # rospy.Subscriber('/hector/' + uav_namespace + '/global_position/global', GeoPoseStamped, self.update_odom)
        pass

    def update_odom(self, msg):
        print(type(msg))
        self.odom = msg

    def execute(self, userdata):
        print("====================================================")
        self.await_connections()
        print("\n\n\n", " EXECUTING FLY TO ESTIMATED TURBINE POSITION: ", userdata.keys())
        print("\n\n", userdata)
        data = userdata
        # rospy.init_node(data.uav_namespace+'_fly_to_estimated_turbine_position')

        self.nav_to_est_wt_pos(data.turbine_estimated_position)
        print("AWAITING FOR ARIVAL AT DEST: ")
        # print(wait_for_action_completion(100, data.turbine_estimated_position, 1, 0.1, self.odom))
        # for now just return the drone has arrived after x seconds
        time.sleep(20)
        if True:
            return WT_FOUND
        elif True:
            return WT_NOT_FOUND
        else:
            return ERROR

    def await_connections(self):
        while (self.nav_pub.get_num_connections() == 0):
            time.sleep(0.2)

    def nav_to_est_wt_pos(self, est_pos):
        print("NAVIGATING TO: ")
        print(est_pos)
        print(self.nav_pub)
        print(self.nav_pub.name)
        print(self.nav_pub.get_num_connections())
        print(self.nav_pub.reg_type)
        print(self.nav_pub.impl)
        print(self.nav_pub.data_class)
        self.nav_pub.publish(est_pos)
        print("NAVIGATION MESSAGE SENT")

    def detect_turbine(self, msg):
        # print('DETECTING TURBINE: ')
        pass
