#!/usr/bin/env python

import actionlib
# roslib.load_manifest('my_pkg_name')
import rospy
import smach
from mimree_executive.msg import FindTurbineOdomAction
from states.fly_to_estimated_turbine_position import FlyToEstimatedTurbinePosition
from states.identify_turbine_odom_orientation import IdentifyTurbineOdomOrientation
from states.identify_turbine_odom_position import IdentifyTurbineOdomPosition
from states.state_outcomes import ERROR
from states.state_outcomes import WT_FOUND
from states.state_outcomes import WT_NOT_FOUND
from states.state_outcomes import WT_ODOM_IDENTIFIED
from states.state_outcomes import WT_ODOM_ORIENTATION_FOUND
from states.state_outcomes import WT_ODOM_POSITION_FOUND


class FindTurbineOdomServer:
    """
    The Turbine Odom is defined as follows:
    position: outermost center of the three wings
    orientation: the way the turbine is facing
    This is using gps positioning.
    """

    def __init__(self):
        print("STARTING FIND TURBINE ODOM SERVER")
        self.server = actionlib.SimpleActionServer('find_turbine_odom', FindTurbineOdomAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        # A SMACH implementation of the following tasks:
        # rospy.loginfo("EXECUTING FIND TURBINE SERVER: "+ str(goal))
        print('EXEC Find Turbine')
        sm = smach.StateMachine(outcomes=[WT_NOT_FOUND, WT_ODOM_IDENTIFIED, ERROR],
                                input_keys=['sm_input'],
                                output_keys=['sm_output']
        )
        sm.userdata = self.msg_to_dict(goal)

        with sm:
            # drone flight to estimated position of turbine

            smach.StateMachine.add('FlyToEstimatedTurbinePosition', FlyToEstimatedTurbinePosition(),
                                   transitions={
                                       ERROR: ERROR,
                                       WT_NOT_FOUND: WT_NOT_FOUND,
                                       WT_FOUND: 'IdentifyTurbineOdomOrientation'},
                                   remapping={
                                       'ftetp_input':'sm_input',
                                       'ftetp_output':'sm_output'
                                   }
                                  )
            # drone identification of turbine orientation (main column is in front of or behind wings & face)

            smach.StateMachine.add('IdentifyTurbineOdomOrientation', IdentifyTurbineOdomOrientation(),
                                   transitions={WT_ODOM_ORIENTATION_FOUND: 'IdentifyTurbineOdomPosition',
                                                ERROR: ERROR,
                                                })
            # drone identification of the pose for the turbine center
            smach.StateMachine.add('IdentifyTurbineOdomPosition', IdentifyTurbineOdomPosition(),
                                   transitions={WT_ODOM_POSITION_FOUND: WT_ODOM_IDENTIFIED,
                                                ERROR: ERROR,
                                                })
            print(sm.userdata)
            print(type(sm.userdata))
            sm.execute()

            self.server.set_succeeded()

    def msg_to_dict(self, msg):
        ret={}
        for sl in msg.__slots__:
            ret[sl]=msg.__getattribute__(sl)
        return ret


if __name__ == '__main__':
    rospy.init_node('mission_find_turbine_odom_server')
    server = FindTurbineOdomServer()
    rospy.spin()
