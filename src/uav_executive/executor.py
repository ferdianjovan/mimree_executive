#!/usr/bin/env python

# Other 3rd party packages
import numpy as np
# ROS packages
import rospy
# MAVROS packages
from mavros_msgs.msg import (HomePosition, State, StatusText, Waypoint,
                             WaypointReached)
from mavros_msgs.srv import (CommandBool, CommandHome, CommandLong, CommandTOL,
                             SetMode, WaypointClear, WaypointPush,
                             WaypointSetCurrent)
from sensor_msgs.msg import BatteryState, NavSatFix, Range
from std_msgs.msg import Float64, Header


class ActionExecutor(object):

    INIT_VOLTAGE = 12.587
    MINIMUM_VOLTAGE = 12.19
    EXTERNAL_INTERVENTION = -2
    OUT_OF_DURATION = -1
    ACTION_SUCCESS = 1
    ACTION_FAIL = 0

    def __init__(self, namespace, waypoints, update_frequency=10.):
        """
        A Class that interfaces with MAVROS for executing actions
        """
        self.namespace = namespace
        self.landed = True
        self.home_moved = False
        self.rel_alt = 0.
        self.rangefinder = -1.
        self.wp_reached = -1
        self.previous_mode = ''
        self.current_mode = ''
        self._cancel_action = False
        self.external_intervened = False
        self.state = State()
        self.waypoints = list()
        self.home = HomePosition()
        self.global_pose = NavSatFix()
        self.battery_voltages = [self.INIT_VOLTAGE for _ in range(100)]
        self.low_battery = False
        self._waypoints = waypoints
        self._current_wp = -1
        self._rel_alt = [0. for _ in range(20)]
        self._rel_alt_seq = 0
        self._rangefinder = [-1. for _ in range(20)]
        self._min_range = -1.
        self._status_text = ''
        self._arm_status = [False for _ in range(5)]

        # Service proxies
        rospy.loginfo('Waiting for service /%s/mavros/cmd/command ...' %
                      self.namespace)
        rospy.wait_for_service('/%s/mavros/cmd/command' % self.namespace)
        self._command_proxy = rospy.ServiceProxy(
            '/%s/mavros/cmd/command' % self.namespace, CommandLong)
        rospy.loginfo('Waiting for service /%s/mavros/mission/push ...' %
                      self.namespace)
        rospy.wait_for_service('/%s/mavros/mission/push' % self.namespace)
        self._add_wp_proxy = rospy.ServiceProxy(
            '/%s/mavros/mission/push' % self.namespace, WaypointPush)
        rospy.loginfo(
            'Waiting for service /%s/mavros/mission/set_current ...' %
            self.namespace)
        rospy.wait_for_service('/%s/mavros/mission/set_current' %
                               self.namespace)
        self._set_current_wp_proxy = rospy.ServiceProxy(
            '/%s/mavros/mission/set_current' % self.namespace,
            WaypointSetCurrent)
        rospy.loginfo('Waiting for /%s/mavros/set_mode ...' % self.namespace)
        rospy.wait_for_service('/%s/mavros/set_mode' % self.namespace)
        self._set_mode_proxy = rospy.ServiceProxy(
            '/%s/mavros/set_mode' % self.namespace, SetMode)
        rospy.loginfo('Wait for service /%s/mavros/mission/clear ...' %
                      self.namespace)
        rospy.wait_for_service('/%s/mavros/mission/clear' % self.namespace)
        self._clear_wp_proxy = rospy.ServiceProxy(
            '/%s/mavros/mission/clear' % self.namespace, WaypointClear)
        rospy.loginfo('Waiting for /%s/mavros/cmd/arming ...' % self.namespace)
        rospy.wait_for_service('/%s/mavros/cmd/arming' % self.namespace)
        self._arming_proxy = rospy.ServiceProxy(
            '/%s/mavros/cmd/arming' % self.namespace, CommandBool)
        rospy.loginfo('Waiting for /%s/mavros/cmd/takeoff ...' %
                      self.namespace)
        rospy.wait_for_service('/%s/mavros/cmd/takeoff' % self.namespace)
        self._takeoff_proxy = rospy.ServiceProxy(
            '/%s/mavros/cmd/takeoff' % self.namespace, CommandTOL)
        rospy.loginfo('Waiting for /%s/mavros/cmd/set_home ...' %
                      self.namespace)
        rospy.wait_for_service('/%s/mavros/cmd/set_home' % self.namespace)
        self._set_home_proxy = rospy.ServiceProxy(
            '/%s/mavros/cmd/set_home' % self.namespace, CommandHome)

        self._rate = rospy.Rate(update_frequency)

        # Subscribers
        rospy.Subscriber('/%s/mavros/state' % self.namespace,
                         State,
                         self._state_cb,
                         queue_size=10)
        # halt until mavros is connected to a uav
        rospy.loginfo('Waiting for a connection between MAVROS and UAV ...')
        while (not self.state.connected):
            self._rate.sleep()
        rospy.Subscriber('/%s/mavros/home_position/home' % self.namespace,
                         HomePosition,
                         self._home_cb,
                         queue_size=10)
        rospy.Subscriber('/%s/mavros/global_position/rel_alt' % self.namespace,
                         Float64,
                         self._relative_alt_cb,
                         queue_size=10)
        rospy.Subscriber('/%s/mavros/battery' % self.namespace,
                         BatteryState,
                         self._battery_cb,
                         queue_size=10)
        rospy.Subscriber('/%s/mavros/global_position/global' % self.namespace,
                         NavSatFix,
                         self._global_pose_cb,
                         queue_size=10)
        rospy.Subscriber('/%s/mavros/mission/reached' % self.namespace,
                         WaypointReached,
                         self._wp_reached_cb,
                         queue_size=10)
        rospy.Subscriber('/%s/mavros/statustext/recv' % self.namespace,
                         StatusText,
                         self._status_text_cb,
                         queue_size=10)
        rospy.Subscriber('/%s/mavros/rangefinder/rangefinder' % self.namespace,
                         Range,
                         self._rangefinder_cb,
                         queue_size=10)

        # Auto call functions
        rospy.Timer(self._rate.sleep_dur, self.update_landing_status)
        rospy.Timer(10 * self._rate.sleep_dur, self.intervene_observer)
        rospy.Timer(self._rate.sleep_dur, self.update_wp_position)
        rospy.loginfo('Adding WPs ...')
        # Adding initial waypoints' configuration
        while not self.add_waypoints():
            self._rate.sleep()

    def _rangefinder_cb(self, msg):
        """
        Rangefinder call back
        """
        self._rangefinder[msg.header.seq % 20] = msg.range
        self.rangefinder = np.mean(self._rangefinder)
        if (self._min_range == -1) and (-1. not in self._rangefinder):
            self._min_range = np.mean(self._rangefinder)

    def _status_text_cb(self, msg):
        """
        Status text call back
        """
        if msg.text == 'PreArm: Gyros not calibrated':
            self._status_text = msg.text

    def _wp_reached_cb(self, msg):
        """
        Waypoint reached call back
        """
        self.wp_reached = msg.wp_seq

    def _state_cb(self, msg):
        """
        UAV current state callback
        """
        if self.current_mode == '':
            self.current_mode = msg.mode
        self.state = msg
        self._arm_status[msg.header.seq % len(self._arm_status)] = msg.armed

    def _home_cb(self, msg):
        """
        Home position callback
        """
        if self.home.header != Header():
            prev_home = np.array(
                [self.home.geo.latitude, self.home.geo.longitude])
            current_home = np.array([msg.geo.latitude, msg.geo.longitude])
            if np.linalg.norm(prev_home - current_home) > 3e-06:
                self.home_moved = True
                self.home = msg
            else:
                self.home_moved = False
        else:
            self.home = msg

    def _relative_alt_cb(self, msg):
        """
        Relative altitude callback
        """
        self._rel_alt[self._rel_alt_seq % 20] = msg.data
        self._rel_alt_seq += 1
        self.rel_alt = np.mean(self._rel_alt)

    def _global_pose_cb(self, msg):
        """
        UAV global position callback
        """
        self.global_pose = msg

    def _battery_cb(self, msg):
        """
        UAV Battery state callback
        """
        self.battery_voltages[msg.header.seq % 100] = msg.voltage
        self.low_battery = (np.mean(self.battery_voltages) <=
                            self.MINIMUM_VOLTAGE) and (self._current_wp != 0)

    def update_wp_position(self, event):
        """
        Update UAV position relative to UAV waypoints
        """
        wp = -1
        for idx, waypoint in enumerate(self.waypoints):
            temp = np.array([waypoint.x_lat, waypoint.y_long])
            cur_pos = np.array([
                self.global_pose.latitude,
                self.global_pose.longitude,
            ])
            alt_diff = abs(self.global_pose.altitude -
                           (self.home.geo.altitude + waypoint.z_alt))
            if np.linalg.norm(cur_pos - temp) < 1e-5 and alt_diff < 0.2:
                wp = idx
                break
        self._current_wp = wp

    def set_battery(self, init_batt, min_batt):
        """
        Setting initial battery and minimum battery condition
        """
        rospy.loginfo('Setting up battery requirements...')
        self.INIT_VOLTAGE = init_batt
        self.MINIMUM_VOLTAGE = min_batt
        self.battery_voltages = [self.INIT_VOLTAGE for _ in range(100)]
        self.low_battery = False

    def intervene_observer(self, event):
        """
        Watch whether human operator intervenes
        """
        mode_status_check = (self.current_mode != '') and (
            self.state.mode not in [self.current_mode, self.previous_mode])
        # stabilize_on_land_check = (
        #     self.state.mode == 'STABILIZE') and self.landed
        self.external_intervened = mode_status_check or self._cancel_action
        # and (not stabilize_on_land_check)

    def update_landing_status(self, event):
        """
        Automated update landing (or flying) status
        """
        if self._min_range > -1.:
            self.landed = (self.rangefinder <= (1.5 * self._min_range)) and (
                self.rangefinder >= (0.5 * self._min_range))
        elif self._status_text == 'PreArm: Gyros not calibrated':
            self.landed = (False in self._arm_status)
        else:
            self.landed = (not self.state.armed) or (self.rel_alt <= 0.1)

    def add_waypoints(self, index=0):
        """
        Setting up waypoints for the UAV
        """
        if self.home.header == Header():
            rospy.logwarn(
                'Home has not been set! Setting current location as home.')
            self.set_current_location_as_home()
        # Clear current wps available
        self._clear_wp_proxy()
        # First wp is home with landing action
        home_wp = Waypoint(Waypoint.FRAME_GLOBAL_REL_ALT, 21, False, False, .6,
                           1., 0, 0, self.home.geo.latitude,
                           self.home.geo.longitude, 0.)
        wps = [home_wp]
        # Iterating other wps given by planner
        for waypoint in self._waypoints:
            wp = Waypoint(Waypoint.FRAME_GLOBAL_REL_ALT, 16, False, False,
                          waypoint['hold_time'], waypoint['radius'], 0,
                          waypoint['yaw'], waypoint['lat'], waypoint['long'],
                          waypoint['rel_alt'])
            wps.append(wp)
        # Push waypoints to mavros service
        response = self._add_wp_proxy(index, wps)
        if response.success:
            rospy.loginfo('%d waypoints are added' % response.wp_transfered)
        else:
            rospy.logwarn('No waypoint is added!')
        self.waypoints = wps
        return response.success

    def set_current_target_wp(self, wp_index):
        """
        Set target wp for UAV to go
        """
        response = self._set_current_wp_proxy(wp_index)
        if response.success:
            rospy.loginfo('Setting current target waypoint to %d' % wp_index)
        else:
            rospy.logwarn('Waypoint %d can\'t be set!' % wp_index)
        return response.success

    def set_current_location_as_home(self):
        """
        Set current location as the new home location
        """
        response = self._set_home_proxy(True, 0., 0., 0.)
        if response.success:
            rospy.loginfo('Setting current location as the new home ...')
        else:
            rospy.logwarn('Current location can\'t be set as the new home!')
        return response.success

    def guided_mode(self, duration=rospy.Duration(60, 0), mode='guided'):
        """
        Guided mode action
        """
        start = rospy.Time.now()
        guided = False
        if self.state.mode != mode.upper():
            while (rospy.Time.now() - start <
                   duration) and (not rospy.is_shutdown()) and (
                       not guided) and (not self.external_intervened):
                guided = self._set_mode_proxy(0, mode).mode_sent
                if guided:
                    self.previous_mode = self.current_mode
                    self.current_mode = mode.upper()
                self._rate.sleep()
            rospy.loginfo('UAV changes its mode to %s ...' % mode.upper())
        else:
            self.previous_mode = self.current_mode
            self.current_mode = mode.upper()
            guided = True
        if (rospy.Time.now() - start) > duration:
            guided = self.OUT_OF_DURATION
        if self.external_intervened:
            guided = self.EXTERNAL_INTERVENTION
        return int(guided)

    def arm(self, duration=rospy.Duration(60, 0)):
        """
        Arm throttle action
        """
        start = rospy.Time.now()
        self.guided_mode(duration)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        armed = False
        if not self.state.armed:
            while (rospy.Time.now() - start <
                   duration) and (not rospy.is_shutdown()) and (
                       not armed) and (not self.external_intervened):
                armed = self._arming_proxy(True).success
                self._rate.sleep()
            rospy.loginfo('Arming the UAV ...')
        else:
            armed = True
        if (rospy.Time.now() - start) > duration:
            armed = self.OUT_OF_DURATION
        if self.external_intervened:
            armed = self.EXTERNAL_INTERVENTION
        return int(armed)

    def takeoff(self, altitude, duration=rospy.Duration(60, 0)):
        """
        Take off action
        """
        start = rospy.Time.now()
        self.guided_mode(duration)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        took_off = True
        if self.landed:
            took_off = self._takeoff_proxy(0.1, 0, 0, 0, altitude).success
            rospy.loginfo('UAV is taking off to %d meter height' % altitude)
            while (rospy.Time.now() - start <
                   duration) and (not rospy.is_shutdown()
                                  ) and (self.rel_alt - altitude) < -0.5 and (
                                      not self.external_intervened):
                if not took_off:
                    took_off = self._takeoff_proxy(0.1, 0, 0, 0,
                                                   altitude).success
                if (self.rel_alt > (altitude / 2.)) and (np.mean(
                        np.diff(self._rel_alt)) <= 0.08):
                    break
                if self.low_battery:
                    rospy.logwarn('Battery is below minimum voltage!')
                    break
                self._rate.sleep()
        if (rospy.Time.now() - start) > duration:
            took_off = self.OUT_OF_DURATION
        if self.external_intervened:
            took_off = self.EXTERNAL_INTERVENTION
        return int(took_off)

    def goto(self,
             waypoint,
             duration=rospy.Duration(60, 0),
             low_battery_trip=False):
        """
        Go to specific waypoint action
        """
        self.wp_reached = -1
        start = rospy.Time.now()
        if not self.set_current_target_wp(waypoint):
            return self.ACTION_FAIL
        auto = False
        while (rospy.Time.now() - start <
               duration) and (not rospy.is_shutdown()) and (
                   not self.external_intervened) and (not auto):
            auto = self._set_mode_proxy(0, 'auto').mode_sent
            if auto:
                self.previous_mode = self.current_mode
                self.current_mode = 'AUTO'
            self._rate.sleep()
        rospy.loginfo('Setting mode to AUTO ...')
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        while (rospy.Time.now() - start < duration) and not (
                rospy.is_shutdown()) and (not self.external_intervened) and (
                    (waypoint != self.wp_reached)):
            if not low_battery_trip and self.low_battery:
                rospy.logwarn('Battery is below minimum voltage!')
                break
            self._rate.sleep()
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        response = int(waypoint == self.wp_reached)
        if not (self.low_battery or self.external_intervened):
            self.guided_mode(duration=duration)
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        elif self.external_intervened:
            response = self.EXTERNAL_INTERVENTION
        return response

    def request_arm(self, disarm=False, duration=rospy.Duration(60, 0)):
        """
        Request for arming UAV
        """
        start = rospy.Time.now()
        if not disarm:
            rospy.loginfo('Waiting for the UAV to be ARMED ...')
        else:
            rospy.loginfo('Waiting for the UAV to be DISARMED ...')
        while (rospy.Time.now() - start < duration) and not (
                rospy.is_shutdown()) and (not self.external_intervened) and (
                    not (disarm ^ self.state.armed)):
            self._rate.sleep()
        response = int(self.state.armed ^ disarm)
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        elif self.external_intervened:
            response = self.EXTERNAL_INTERVENTION
        return response

    def full_mission_auto(self,
                          waypoint,
                          duration=rospy.Duration(60, 0),
                          low_battery_trip=False):
        """
        Go from waypoint i to the last
        """
        self.wp_reached = -1
        wps_reached = list()
        start = rospy.Time.now()
        if not self.set_current_target_wp(waypoint):
            return self.ACTION_FAIL
        auto = False
        while (rospy.Time.now() - start <
               duration) and (not rospy.is_shutdown()) and (
                   not self.external_intervened) and len(wps_reached) < len(
                       self.waypoints[waypoint:]):
            if not auto:
                rospy.loginfo('Setting mode to AUTO ...')
                auto = self._set_mode_proxy(0, 'auto').mode_sent
                self.previous_mode = self.current_mode if auto else (
                    self.previous_mode)
                self.current_mode = 'AUTO' if auto else self.current_mode
            if self.wp_reached != -1 and self.wp_reached not in wps_reached:
                wps_reached.append(self.wp_reached)
            if not low_battery_trip and self.low_battery:
                rospy.logwarn('Battery is below minimum voltage!')
                break
            self._rate.sleep()
        response = int(len(wps_reached) == len(self.waypoints[waypoint:]))
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        elif self.external_intervened:
            response = self.EXTERNAL_INTERVENTION
        return response

    def return_to_launch(self,
                         monitor_home=False,
                         duration=rospy.Duration(600, 0)):
        """
        Return to home action
        """
        start = rospy.Time.now()
        if self._set_mode_proxy(0, 'rtl').mode_sent:
            rospy.loginfo('Setting mode to RTL ...')
            self.previous_mode = self.current_mode
            self.current_mode = 'RTL'
        while (rospy.Time.now() - start <
               duration) and not (rospy.is_shutdown()) and (
                   not self.external_intervened) and (not self.landed):
            cond = monitor_home and self.home_moved and (self.rel_alt < 10.)
            if cond or self.current_mode != 'RTL':
                duration = duration - (rospy.Time.now() - start)
                start = rospy.Time.now()
                self.emergency_landing(duration)
            self._rate.sleep()
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        disarm = self.request_arm(True, duration)
        landed = int(self.landed and disarm == self.ACTION_SUCCESS)
        if (rospy.Time.now() - start) > duration:
            landed = self.OUT_OF_DURATION
        if self.external_intervened:
            landed = self.EXTERNAL_INTERVENTION
        return landed

    def emergency_landing(self, duration=rospy.Duration(600, 0)):
        """
        Emergency land to home when home wobbles
        """
        start = rospy.Time.now()
        landed = 0
        while (rospy.Time.now() - start <
               duration) and not (rospy.is_shutdown()) and (
                   not self.external_intervened) and (not self.landed):
            if landed == self.ACTION_SUCCESS:
                self._rate.sleep()
                continue
            cur_pos = np.array([
                self.global_pose.latitude,
                self.global_pose.longitude,
            ])
            home_pos = np.array(
                [self.home.geo.latitude, self.home.geo.longitude])
            altitude = 0.9
            if np.linalg.norm(cur_pos - home_pos) < 3e-6 and (self.rel_alt <=
                                                              altitude):
                if self._min_range > -1 and np.abs(
                        np.max(np.diff(self._rangefinder[-5:]))) > 1.:
                    continue
                landed = self.guided_mode(50 * self._rate.sleep_dur,
                                          mode='land')
            else:
                if not self._clear_wp_proxy().success:
                    if self.current_mode != 'GUIDED':
                        self.guided_mode(10 * self._rate.sleep_dur)
                    continue
                # wps[0] must be home waypoint
                home_wp = Waypoint(Waypoint.FRAME_GLOBAL_REL_ALT, 21, False,
                                   False, 1.0, 1., 0, 0,
                                   self.home.geo.latitude,
                                   self.home.geo.longitude, 0.)
                wp = Waypoint(Waypoint.FRAME_GLOBAL_REL_ALT, 16, False, False,
                              0.1, 0.3, 0, 0., self.home.geo.latitude,
                              self.home.geo.longitude, altitude / 3.)
                wps = [home_wp, wp]
                self.waypoints = wps
                # Push waypoints to mavros service
                if self._add_wp_proxy(0, wps).success:
                    self.goto(1, 50 * self._rate.sleep_dur, True)
            self._rate.sleep()
        landed = int(self.landed)
        if bool(landed):
            self.add_waypoints()
        self.guided_mode(10 * self._rate.sleep_dur)
        if (rospy.Time.now() - start) > duration:
            landed = self.OUT_OF_DURATION
        if self.external_intervened:
            landed = self.EXTERNAL_INTERVENTION
        return landed

    def reboot_autopilot_computer(self, duration=rospy.Duration(60, 0)):
        """
        Preflight reboot shutdown for autopilot and onboard computer
        """
        start = rospy.Time.now()
        result = self._command_proxy(broadcast=False,
                                     command=246,
                                     confirmation=0,
                                     param1=1.,
                                     param2=1.,
                                     param3=0.,
                                     param4=0.,
                                     param5=0.,
                                     param6=0.,
                                     param7=0.)
        response = self.ACTION_SUCCESS if result.success else self.ACTION_FAIL
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        return response
