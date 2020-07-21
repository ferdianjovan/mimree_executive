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
                                input_keys=goal.__slots__,
                                output_keys=goal.__slots__
                                )
        userdata=self.msg_to_dict(goal)
        sm.userdata._data=userdata
        with sm:
            # drone flight to estimated position oBoucf turbinehttps://2.bp.blogspot.com/uW4v5wLM_virixoivjgVVOL4VYZKA6rFUrQXHzQvEpiRj17TmtrZPWrsO9Jqs7rlpFWoW_Fhmx6dZvPdjLOMCcUuFdh-vhoFUwoRWELhD5ARB4HVruuzET9vczt9QvYYczaa4IRSWg=s0?title=ODEuMjUxLjEzMy4yNDU=001-003___1593434756.png

            smach.StateMachine.add('FlyToEstimatedTurbinePosition', FlyToEstimatedTurbinePosition(goal.__slots__,userdata['uav_namespace']),
                                   transitions={
                                       ERROR: ERROR,
                                       WT_NOT_FOUND: WT_NOT_FOUND,
                                       WT_FOUND: 'IdentifyTurbineOdomOrientation'},
                                   remapping=dict(zip(goal.__slots__, goal.__slots__))
                                   )
            # drone identification of turbine orientation (main column is in front of or behind wings & face)https://2.bp.blogspot.com/W2LcnA8adgre7euW-M6aiOFPZSwg0gQOeaQkJSGAdrssZFx75B0-dVsAcuPTe-ArcN2XRSg8W_q0G8fj7GPxhHvoWl_0aGVBFlab9BguYU9JVuH9x_RbgkdKtxONRU5h-QqTP3ru_g=s0?title=ODEuMjUxLjEzMy4yNDU=007-001___1594811036.png

            smach.StateMachine.add('IdentifyTurbineOdomOrientation', IdentifyTurbineOdomOrientation(),
                                   transitions={WT_ODOM_ORIENTATION_FOUND: 'IdentifyTurbineOdomPosition',
                                                ERROR: ERROR,
                                                })
            # drone identification of the pose for the turbine center
            smach.StateMachine.add('IdentifyTurbineOdomPosition', IdentifyTurbineOdomPosition(),
                                   transitions={WT_ODOM_POSITION_FOUND: WT_ODOM_IDENTIFIED,
                                                ERROR: ERROR,
                                                })
            sm.execute(
                parent_ud=sm.userdata
            )

            self.server.set_succeeded()

    def msg_to_dict(self, msg):
        ret = {}
        for sl in msg.__slots__:
            ret[sl] = msg.__getattribute__(sl)
        return ret


if __name__ == '__main__':
    rospy.init_node('mission_find_turbine_odom_server')
    server = FindTurbineOdomServer()
    rospy.spin()
