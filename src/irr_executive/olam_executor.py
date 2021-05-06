#!/usr/bin/env python

import sys

import rospy
from mavros_msgs.msg import DebugValue
from mavros_msgs.srv import ButtonChange, SetMode


class CommandStatus(object):

    # Command
    RESET = 72
    POWER_OFF = 81
    PREPARE_FOR_ENGAGEMENT = 82
    HOME_POSITION = 83
    DETENTION = 84
    HOLD_POSITION = 85

    # General status
    UNKNOWN = 0
    ERROR = 1

    # Mode status
    MODE_POWERED_OFF = 2
    MODE_POSITION_HOLDING = 3  # this is for home position

    # Execution status
    EXECUTING = 0
    FAILED_EXECUTION = 1
    SUCCESSFUL_EXECUTION = 2


class ActionExecutor(object):

    ACTION_SUCCESS = 1
    ACTION_FAIL = 0
    OUT_OF_DURATION = -1
    EXTERNAL_INTERVENTION = -2
    TARGET_COMPID = 56

    def __init__(self, namespace, updated_frequency=10):
        # connection through mavros
        self.namespace = namespace
        self.command = CommandStatus.UNKNOWN
        self.command_sequence = 0
        # line_status, sequence, execution
        self.status = [
            CommandStatus.UNKNOWN, CommandStatus.UNKNOWN, CommandStatus.UNKNOWN
        ]
        self.external_intervened = False
        self._rate = rospy.Rate(updated_frequency)
        # Service proxies
        rospy.loginfo("Waiting for /%s/mavros/button_change/send ..." %
                      self.namespace)
        rospy.wait_for_service('/%s/mavros/button_change/send' % namespace)
        self._button_proxy = rospy.ServiceProxy(
            '/%s/mavros/button_change/send' % self.namespace, ButtonChange)
        # Debug Subscriber
        rospy.Subscriber('/%s/mavros/debug_value/debug_vector' %
                         self.namespace,
                         DebugValue,
                         self._debug_cb,
                         queue_size=1)
        # Service to run other commands
        rospy.Service('%s/olam/send' % self.namespace, SetMode,
                      self.execute_command)

    def execute_command(self, req):
        """
        Service function for emergency
        """
        response = self.ACTION_FAIL
        if req.base_mode == 0 or req.custom_mode.upper() == 'reset'.upper():
            response = self.reset_controller()
        elif req.base_mode == 1 or req.custom_mode.upper() == 'home'.upper():
            response = self.home_position()
        elif req.base_mode == 2 or req.custom_mode.upper() == 'engage'.upper():
            response = self.engagement_position()
        return response == self.ACTION_SUCCESS

    def wait_for_olam(self):
        """
        Buffer function to make sure olam is ready
        """
        while self.status == [
                CommandStatus.UNKNOWN, CommandStatus.UNKNOWN,
                CommandStatus.UNKNOWN
        ]:
            self._rate.sleep()

    def _debug_cb(self, msg):
        """
            msg: Message in the form of DebugValue(
                    header, index, name, value_float, value_int, *data, type
            )
        """
        if msg.name == 'OLAMS':
            self.status = list(map(int, msg.data))
            rospy.logwarn("OLAM DEBUG_VECT: %s" % str(self.status))
        return

    def _request_via_srv(self, command):
        self.command_sequence = self.status[1] + 1
        start = rospy.Time.now()
        response = self._button_proxy(
            1, 32467892, command,
            rospy.get_param("/%s/mavros/target_system_id" % self.namespace, 1),
            self.TARGET_COMPID)
        end = rospy.Time.now()
        requested = response.result == 1
        requested = True if (not requested) and (
            (end - start) > rospy.Duration(5, 0)) else requested
        rospy.logwarn("REQUEST %d: %s, ACK_DUR: %d.%d, SEQ: %d" %
                      (command, str(response.result), (end - start).secs,
                       (end - start).nsecs, self.command_sequence))
        return requested

    def _action(self, duration=rospy.Duration(60, 0)):
        """
            Template action
        """
        achieved = False
        start = rospy.Time.now()
        requested = self._request_via_srv(self.command)
        while not achieved and (not self.external_intervened) and (
                not rospy.is_shutdown()) and (rospy.Time.now() -
                                              start) < duration:
            if not requested:
                requested = self._request_via_srv(self.command)
            achieved = (self.status[1] == self.command_sequence) and (
                self.status[2] > CommandStatus.EXECUTING)
            if self.status[2] == CommandStatus.FAILED_EXECUTION:
                achieved = False
                requested = False
            self._rate.sleep()
        response = int(achieved)
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        elif self.external_intervened:
            response = self.EXTERNAL_INTERVENTION
        return response

    def reset_controller(self, duration=rospy.Duration(60, 0)):
        """
            Resetting the OLAM comm module
        """
        rospy.loginfo("Resetting controller...")
        self.command = CommandStatus.RESET
        return self._action(duration=duration)

    def engagement_position(self, duration=rospy.Duration(60, 0)):
        """
            Tightening the string for engagement
        """
        rospy.loginfo("OLAM is preparing for engagement ...")
        self.command = CommandStatus.PREPARE_FOR_ENGAGEMENT
        return self._action(duration)

    def home_position(self, duration=rospy.Duration(60, 0)):
        """
            Return the hand of OLAM to home position
        """
        rospy.loginfo("OLAM is going to home position...")
        self.command = CommandStatus.HOME_POSITION
        return self._action(duration)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(2)

    rospy.init_node('olam_test')
    olam = ActionExecutor('corin')
    id_to_act = {
        0: olam.reset_controller,
        1: olam.engagement_position,
        2: olam.home_position,
    }
    olam.wait_for_olam()
    for i in sys.argv[1].replace(" ", "").split(","):
        success = id_to_act[int(i)](rospy.Duration(30))
        print("SUCCESS:", success)
        if int(i) == 1:
            continue
        rospy.sleep(1)
