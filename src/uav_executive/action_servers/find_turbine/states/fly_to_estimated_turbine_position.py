#!/usr/bin/env python
import time

import numpy as np
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
        rospy.Subscriber('/hector/' + uav_namespace + '/global_position/global', GeoPoseStamped, self.update_odom)
        pass

    def update_odom(self, msg):
        print(type(msg))
        self.odom = msg

    def wait_for_action_completion(self, timeout, target_pose, pos_err, or_err,
                                   pose_stream):
        """

        :param timeout: allowed time to reach target pose
        :param target_pose: the pose to reach
        :param pos_err: acceptable tolerance to assume pose has been reached
        :param or_err: acceptable tolerance to assume pose has been reached
        :param pose_stream: the constantly updated pose value
        :return:
        """
        attributes = ['altitude', 'longitude', 'latitude', 'x', 'y', 'z', 'w']
        time_elapsed = time.time()
        t_pos, t_or = target_pose.pose.position, target_pose.pose.orientation
        t_pos_ranges = map(lambda a: [t_pos.__getattribute__(a) - pos_err, t_pos.__getattribute__(a) + pos_err],
                           attributes[:3])
        t_or_ranges = map(lambda a: [t_or.__getattribute__(a) - or_err, t_or.__getattribute__(a) + or_err],
                          attributes[3:])
        t_ranges = t_pos_ranges + t_or_ranges
        prev_c_ranges = None
        while timeout > 0:
            timeout -= time.time() - time_elapsed
            time_elapsed = time.time()
            # both positional and orientational values must be within target+-error to be accepted
            c_pos = pose_stream.pose.position
            c_or = pose_stream.pose.orientation
            c_pos_ranges = map(lambda a: c_pos.__getattribute__(a), attributes[:3])
            c_or_ranges = map(lambda a: c_or.__getattribute__(a), attributes[3:])
            c_ranges = c_pos_ranges + c_or_ranges
            if prev_c_ranges == c_ranges:
                print("awaiting update")
                time.sleep(0.2)
                continue
            prev_c_ranges = c_ranges
            if np.all(map((lambda a, b: b[0] < a < b[1]), c_ranges, t_ranges)):
                return (True, "POSE_REACHED")
        return (False, "OUT_OF_TIME")

    def execute(self, userdata):
        print("====================================================")
        time.sleep(10)
        print("\n\n\n", " EXECUTING FLY TO ESTIMATED TURBINE POSITION: ", userdata.keys())
        print("\n\n", userdata)
        data = userdata
        # rospy.init_node(data.uav_namespace+'_fly_to_estimated_turbine_position')

        self.nav_to_est_wt_pos(data.turbine_estimated_position)
        print("AWAITING FOR ARIVAL AT DEST: ")
        print(self.wait_for_action_completion(100, data.turbine_estimated_position, 1, 0.1, self.odom))
        if True:
            return WT_FOUND
        elif True:
            return WT_NOT_FOUND
        else:
            return ERROR

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
#!/usr/bin/env python
import time

import numpy as np
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
        rospy.Subscriber('/hector/' + uav_namespace + '/global_position/global', GeoPoseStamped, self.update_odom)
        pass

    def update_odom(self, msg):
        print(type(msg))
        self.odom = msg

    def wait_for_action_completion(self, timeout, target_pose, pos_err, or_err,
                                   pose_stream):
        """

        :param timeout: allowed time to reach target pose
        :param target_pose: the pose to reach
        :param pos_err: acceptable tolerance to assume pose has been reached
        :param or_err: acceptable tolerance to assume pose has been reached
        :param pose_stream: the constantly updated pose value
        :return:
        """
        attributes = ['altitude', 'longitude', 'latitude', 'x', 'y', 'z', 'w']
        time_elapsed = time.time()
        t_pos, t_or = target_pose.pose.position, target_pose.pose.orientation
        t_pos_ranges = map(lambda a: [t_pos.__getattribute__(a) - pos_err, t_pos.__getattribute__(a) + pos_err],
                           attributes[:3])
        t_or_ranges = map(lambda a: [t_or.__getattribute__(a) - or_err, t_or.__getattribute__(a) + or_err],
                          attributes[3:])
        t_ranges = t_pos_ranges + t_or_ranges
        prev_c_ranges = None
        while timeout > 0:
            timeout -= time.time() - time_elapsed
            time_elapsed = time.time()
            # both positional and orientational values must be within target+-error to be accepted
            c_pos = pose_stream.pose.position
            c_or = pose_stream.pose.orientation
            c_pos_ranges = map(lambda a: c_pos.__getattribute__(a), attributes[:3])
            c_or_ranges = map(lambda a: c_or.__getattribute__(a), attributes[3:])
            c_ranges = c_pos_ranges + c_or_ranges
            if prev_c_ranges == c_ranges:
                print("awaiting update")
                time.sleep(0.2)
                continue
            prev_c_ranges = c_ranges
            if np.all(map((lambda a, b: b[0] < a < b[1]), c_ranges, t_ranges)):
                return (True, "POSE_REACHED")
        return (False, "OUT_OF_TIME")

    def execute(self, userdata):
        print("====================================================")
        time.sleep(10)
        print("\n\n\n", " EXECUTING FLY TO ESTIMATED TURBINE POSITION: ", userdata.keys())
        print("\n\n", userdata)
        data = userdata
        # rospy.init_node(data.uav_namespace+'_fly_to_estimated_turbine_position')

        self.nav_to_est_wt_pos(data.turbine_estimated_position)
        print("AWAITING FOR ARIVAL AT DEST: ")
        print(self.wait_for_action_completion(100, data.turbine_estimated_position, 1, 0.1, self.odom))
        if True:
            return WT_FOUND
        elif True:
            return WT_NOT_FOUND
        else:
            return ERROR

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
