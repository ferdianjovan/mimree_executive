#!/usr/bin/env python

import sys
import rospy
from mavros_msgs.msg import DebugValue
from mavros_msgs.srv import ButtonChange, SetMode


class CommandStatus(object):

    # Command
    POWER_OFF = 60
    TAKEOFF_MODE = 61
    LANDING_MODE = 62
    SWING_REDUCTION = 63
    CLOSE_HOOK = 66
    OPEN_HOOK = 67
    RESET = 71

    # General status
    UNKNOWN = 0
    OFFLINE = 1
    ERROR = 2

    HINGE_TORQUE_OFF = 10
    HINGE_TAKEOFF = 20
    HINGE_LANDING_TRANSITION = 30
    HINGE_LANDING = 31
    HINGE_SWING_REDUCTION = 40

    HOOK_CLOSED = 10
    HOOK_OPEN = 11
    HOOK_LOOSE = 12
    HOOK_CLOSING_TRANSITION = 20
    HOOK_OPENING_TRANSITION = 21


class ActionExecutor(object):

    ACTION_SUCCESS = 1
    ACTION_FAIL = 0
    OUT_OF_DURATION = -1
    EXTERNAL_INTERVENTION = -2
    TARGET_COMPID = 55

    def __init__(self, namespace, updated_frequency=10):
        # connection through mavros
        self.namespace = namespace
        self.command = CommandStatus.UNKNOWN
        # hinge_status, hook_status, payload
        self.status = [
            CommandStatus.UNKNOWN, CommandStatus.UNKNOWN, CommandStatus.UNKNOWN
        ]
        self.external_intervened = False
        self._hinge_failing = [False for _ in range(10)]
        self._rate = rospy.Rate(updated_frequency)
        # Service proxies
        rospy.loginfo("Waiting for /%s/mavros/button_change/send ..." %
                      self.namespace)
        rospy.wait_for_service("/%s/mavros/button_change/send" %
                               self.namespace)
        self._button_proxy = rospy.ServiceProxy(
            '/%s/mavros/button_change/send' % self.namespace, ButtonChange)
        # Debug Subscriber
        rospy.Subscriber('/%s/mavros/debug_value/debug_vector' %
                         self.namespace,
                         DebugValue,
                         self._debug_cb,
                         queue_size=1)
        # Service to run other commands
        rospy.Service('%s/lhm/send' % self.namespace, SetMode,
                      self.execute_command)

    def execute_command(self, req):
        """
        Service function for emergency
        """
        response = self.ACTION_FAIL
        if req.base_mode == 0 or req.custom_mode.upper() == 'reset'.upper():
            response = self.reset_controller()
        elif req.base_mode == 1 or req.custom_mode.upper() == 'takeoff'.upper(
        ):
            response = self.takeoff_preparation()
        elif req.base_mode == 2 or req.custom_mode.upper() == 'landing'.upper(
        ):
            response = self.landing_preparation()
        elif req.base_mode == 3 or req.custom_mode.upper() == 'swing'.upper():
            response = self.swing_reduction()
        elif req.base_mode == 4 or req.custom_mode.upper() == 'close'.upper():
            response = self.close_hook()
        elif req.base_mode == 5 or req.custom_mode.upper() == 'open'.upper():
            response = self.open_hook()
        return response == self.ACTION_SUCCESS

    def _debug_cb(self, msg):
        """
            msg: Message in the form of DebugValue(
                    header, index, name, value_float, value_int, *data, type
            )
        """
        if msg.name == 'LHMS':
            self.status = list(map(int, msg.data))
            self._hinge_failing.append(self.status[0] == CommandStatus.ERROR)
            self._hinge_failing = self._hinge_failing[1:]
            # rospy.logwarn("LHM DEBUG_VECT: %s" % str(self.status))
        if not (False in self._hinge_failing):
            rospy.logerr("Hinge in %s is damaged!" % self.namespace)
        return

    def _request_via_srv(self, command):
        start = rospy.Time.now()
        response = self._button_proxy(
            1, 64218375, command,
            rospy.get_param("/%s/mavros/target_system_id" % self.namespace, 1),
            self.TARGET_COMPID)
        end = rospy.Time.now()
        requested = response.result == 1
        rospy.logwarn("REQUEST %d: %s, ACK_DUR: %d.%d" %
                      (command, str(response.result), (end - start).secs,
                       (end - start).nsecs))
        return requested

    def _action(self,
                target_cond,
                status_idx,
                transition_cond=None,
                transition=False,
                duration=rospy.Duration(60, 0)):
        """
            Template action
        """
        start = rospy.Time.now()
        requested = self._request_via_srv(self.command)
        achieved = False
        transitioning = False
        while not achieved and (not self.external_intervened) and (
                not rospy.is_shutdown()):
            if (rospy.Time.now() - start >= duration) or ({True} == set(
                    self._hinge_failing)):
                break
            if not (requested or (transitioning and transition)):
                requested = True if (
                    (transitioning and transition) or
                    (self._request_via_srv(self.command))) else False
            transitioning = True if (
                (transitioning and transition) or
                (self.status[status_idx] == transition_cond)) else False
            achieved = self.status[status_idx] == target_cond
            self._rate.sleep()

        response = int(achieved and (False in self._hinge_failing))
        rospy.loginfo("%s action is %s" % (self.namespace, bool(response)))
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        elif self.external_intervened:
            response = self.EXTERNAL_INTERVENTION
        return response

    def _action_without_status_check(self, duration=rospy.Duration(60, 0)):
        start = rospy.Time.now()
        requested = False
        while not requested and (not self.external_intervened) and (
                not rospy.is_shutdown()):
            if (rospy.Time.now() - start >= duration) or ({True} == set(
                    self._hinge_failing)):
                break
            requested = self._request_via_srv(self.command)
            self._rate.sleep()

        response = int(requested and (False in self._hinge_failing))
        rospy.loginfo("%s action is %s" % (self.namespace, bool(response)))
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        elif self.external_intervened:
            response = self.EXTERNAL_INTERVENTION
        return response

    def reset_controller(self, duration=rospy.Duration(60, 0)):
        rospy.loginfo("Resetting controller...")
        self.command = CommandStatus.RESET
        return self._action(self.status[2], 2, duration=duration)

    def swing_reduction(self, duration=rospy.Duration(60, 0)):
        rospy.loginfo("Swing reduction...")
        self.command = CommandStatus.SWING_REDUCTION
        return self._action(CommandStatus.HINGE_SWING_REDUCTION,
                            0,
                            duration=duration)

    def takeoff_preparation(self, duration=rospy.Duration(60, 0)):
        """
            Takeoff preparation by folding the hinge
        """
        rospy.loginfo("Bending hinge for taking-off...")
        self.command = CommandStatus.TAKEOFF_MODE
        return self._action(CommandStatus.HINGE_TAKEOFF, 0, duration=duration)

    def landing_preparation(self, duration=rospy.Duration(60, 0)):
        """
            Landing preparation by folding the hinge
        """
        rospy.loginfo("Bending hinge for landing...")
        self.command = CommandStatus.LANDING_MODE
        return self._action(CommandStatus.HINGE_LANDING, 0,
                            CommandStatus.HINGE_LANDING_TRANSITION, True,
                            duration)

    def open_hook(self, duration=rospy.Duration(60, 0)):
        """
            Open hook to drop IRR
        """
        rospy.loginfo("Opening hook...")
        self.command = CommandStatus.OPEN_HOOK
        return self._action(CommandStatus.HOOK_OPEN, 1,
                            CommandStatus.HOOK_OPENING_TRANSITION, True,
                            duration)

    def close_hook(self, duration=rospy.Duration(60, 0)):
        """
            Close hook
        """
        rospy.loginfo("Closing hook...")
        self.command = CommandStatus.CLOSE_HOOK
        return self._action(CommandStatus.HOOK_CLOSED, 1,
                            CommandStatus.HOOK_CLOSING_TRANSITION, True,
                            duration)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(2)

    rospy.init_node('lhm_test')
    lhm = ActionExecutor('halcyon')
    id_to_act = {
        0: lhm.takeoff_preparation,
        1: lhm.landing_preparation,
        2: lhm.swing_reduction,
        3: lhm.close_hook,
        4: lhm.open_hook,
        5: lhm.reset_controller
    }
    rospy.sleep(1)
    for i in sys.argv[1].replace(" ", "").split(","):
        success = id_to_act[int(i)](rospy.Duration(30))
        print("SUCCESS:", success)
        if int(i) == 1:
            continue
        rospy.sleep(2)
