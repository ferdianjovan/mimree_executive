#!/usr/bin/env python

# Other 3rd party packages
import numpy as np
# ROS packages
import rospy
# MAVROS packages
from mavros_msgs.msg import (HomePosition, State, StatusText, Waypoint,
                             WaypointReached)
from mavros_msgs.srv import (CommandBool, CommandHome, SetMode, WaypointClear,
                             WaypointPush, WaypointSetCurrent)
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Float64, Header


class ActionExecutor(object):

    INIT_FUEL = 100.0
    MINIMUM_FUEL = 25.0
    EXTERNAL_INTERVENTION = -2
    OUT_OF_DURATION = -1
    ACTION_SUCCESS = 1
    ACTION_FAIL = 0

    def __init__(self, namespace, waypoints, uavs=list(),
                 update_frequency=10.):
        """
        A Class that interfaces with MAVROS for executing actions
        """
        self.wp_reached = -1
        self.current_mode = ''
        self.previous_mode = ''
        self.namespace = namespace
        self._cancel_action = False
        self.external_intervened = False
        self.state = State()
        self.waypoints = list()
        self.home = HomePosition()
        self.global_pose = NavSatFix()
        self.heading = 0.0
        self.fuel = self.INIT_FUEL
        self.low_fuel = False
        self._waypoints = waypoints
        self._current_wp = -1
        self._status_text = ''
        self._rate = rospy.Rate(update_frequency)

        # Service proxies
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
        rospy.loginfo('Waiting for /%s/mavros/cmd/set_home ...' %
                      self.namespace)
        rospy.wait_for_service('/%s/mavros/cmd/set_home' % self.namespace)
        self._set_home_proxy = rospy.ServiceProxy(
            '/%s/mavros/cmd/set_home' % self.namespace, CommandHome)
        # UAV service proxies for updating UAV home position
        if len(uavs):
            self._uav_home_proxies = {
                uav:
                rospy.ServiceProxy('/%s/mavros/cmd/set_home' % uav.namespace,
                                   CommandHome)
                for uav in uavs
            }
            self.uav_home_wp = {uav: HomePosition() for uav in uavs}
            self._uav_home_dist = {uav: float('inf') for uav in uavs}
            for uav in uavs:
                rospy.Subscriber('/%s/mavros/home_position/home' %
                                 uav.namespace,
                                 HomePosition,
                                 lambda i: self._uav_home_cb(i, uav),
                                 queue_size=10)
            rospy.Timer(20 * self._rate.sleep_dur, self.update_uav_home_pos)

        # Subscribers
        rospy.Subscriber('/%s/mavros/state' % self.namespace,
                         State,
                         self._state_cb,
                         queue_size=10)
        # halt until mavros is connected to a asv
        rospy.loginfo('Waiting for a connection between MAVROS and ASV ...')
        while (not self.state.connected):
            self._rate.sleep()

        rospy.Subscriber('/%s/mavros/global_position/compass_hdg' %
                         self.namespace,
                         Float64,
                         self._heading_cb,
                         queue_size=10)
        rospy.Subscriber('/%s/mavros/home_position/home' % self.namespace,
                         HomePosition,
                         self._home_cb,
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

        # Auto call functions
        rospy.Timer(10 * self._rate.sleep_dur, self.intervene_observer)
        rospy.Timer(self._rate.sleep_dur, self.update_wp_position)
        rospy.loginfo('Adding WPs ...')
        # change mode just to fill self.current_mode and self.previous_mode
        self.guided_mode()
        # Adding initial waypoints' configuration
        while not self.add_waypoints():
            self._rate.sleep()

    def _status_text_cb(self, msg):
        """
        Status text call back
        """
        if msg.text == 'Reached destination':
            self._status_text = msg.text
        else:
            self._status_text = ''

    def _heading_cb(self, msg):
        """
        ASV heading callback
        """
        self.heading = msg.data / 180. * np.pi

    def _uav_home_cb(self, msg, uav):
        """
        Home position callback specific for UAV
        """
        self.uav_home_wp[uav] = msg

    def _wp_reached_cb(self, msg):
        """
        Waypoint reached call back
        """
        self.wp_reached = msg.wp_seq

    def _state_cb(self, msg):
        """
        ASV current state callback
        """
        if self.current_mode == '':
            self.current_mode = msg.mode
        self.state = msg

    def _home_cb(self, msg):
        """
        Home position callback
        """
        self.home = msg
        if len(self.waypoints):
            home_wp = Waypoint(Waypoint.FRAME_GLOBAL_REL_ALT, 21, False, False,
                               .6, 1., 0, 0, self.home.geo.latitude,
                               self.home.geo.longitude, 0.)
            self.waypoints[0] = home_wp

    def _global_pose_cb(self, msg):
        """
        ASV global position callback
        """
        self.global_pose = msg

    def _battery_cb(self, msg):
        """
        ASV Battery state callback
        """
        self.fuel = msg.percentage * 100.
        self.low_fuel = (self.fuel <=
                         self.MINIMUM_FUEL) and not (self._current_wp == 0)

    def update_uav_home_pos(self, event):
        """
        Update UAV home position when ASV moves
        """
        for uav in self._uav_home_proxies.keys():
            home_wp = np.array([
                self.uav_home_wp[uav].geo.latitude,
                self.uav_home_wp[uav].geo.longitude
            ])
            if self._uav_home_dist[uav] == float('inf'):
                if self.global_pose != NavSatFix():
                    asv_home = np.array([
                        self.global_pose.latitude, self.global_pose.longitude
                    ])
                    self._uav_home_dist[uav] = np.linalg.norm(home_wp -
                                                              asv_home)
                continue
            landpad_lat = self.global_pose.latitude - self._uav_home_dist[
                uav] * np.cos(self.heading)
            landpad_long = self.global_pose.longitude - self._uav_home_dist[
                uav] * np.sin(self.heading)
            landpad = np.array([landpad_lat, landpad_long])
            if np.linalg.norm(home_wp - landpad) > 1e-06:
                self._uav_home_proxies[uav](False, landpad_lat, landpad_long,
                                            2.1)

    def update_wp_position(self, event):
        """
        Update ASV position relative to ASV waypoints
        """
        wp = -1
        for idx, waypoint in enumerate(self.waypoints):
            waypoint = np.array([waypoint.x_lat, waypoint.y_long])
            cur_pos = np.array(
                [self.global_pose.latitude, self.global_pose.longitude])
            if np.linalg.norm(cur_pos - waypoint) < 5e-05:
                wp = idx
                break
        self._current_wp = wp

    def set_battery(self, init_batt, min_batt):
        """
        Setting initial fuel and minimum fuel condition
        """
        rospy.loginfo('Setting up fuel requirements...')
        self.INIT_FUEL = init_batt
        self.MINIMUM_FUEL = min_batt
        self.fuel = self.INIT_FUEL
        self.low_fuel = False

    def intervene_observer(self, event):
        """
        Watch whether human operator intervenes
        """
        mode_status_check = (self.current_mode != '') and (
            self.state.mode not in [self.current_mode, self.previous_mode])
        self.external_intervened = mode_status_check or self._cancel_action

    def add_waypoints(self, index=0):
        """
        Setting up waypoints for the ASV
        """
        while self.home.header == Header() and (not rospy.is_shutdown()):
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
                          0.)
            wps.append(wp)
        # Push waypoints to mavros service
        response = self._add_wp_proxy(index, wps)
        if response.success:
            rospy.loginfo('%d waypoints are added to %s!' %
                          (response.wp_transfered, self.namespace))
        else:
            rospy.logwarn('No waypoint is added to %s!' % self.namespace)
        self.waypoints = wps
        return response.success

    def set_current_target_wp(self, wp_index):
        """
        Set target wp for ASV to go
        """
        response = self._set_current_wp_proxy(wp_index)
        if response.success:
            rospy.loginfo('Setting current target for %s to waypoint %d' %
                          (self.namespace, wp_index))
        else:
            rospy.logwarn('%s can not set waypoint %d!' %
                          (self.namespace, wp_index))
        return response.success

    def set_current_location_as_home(self):
        """
        Set current location as the new home location
        """
        response = self._set_home_proxy(True, 0., 0., 0.)
        if response.success:
            rospy.loginfo(
                'Setting current location of %s as the new home ...' %
                self.namespace)
        else:
            rospy.logwarn('%s can not set current location as the new home!' %
                          self.namespace)
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
            rospy.loginfo('ASV changes its mode to %s ...' % mode.upper())
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
        armed = self._arming_proxy(True).success
        if not self.state.armed:
            while (rospy.Time.now() - start <
                   duration) and (not rospy.is_shutdown()) and (
                       not armed) and (not self.external_intervened):
                armed = self._arming_proxy(True).success
                self._rate.sleep()
            rospy.loginfo('Arming the ASV ...')
        if (rospy.Time.now() - start) > duration:
            armed = self.OUT_OF_DURATION
        if self.external_intervened:
            armed = self.EXTERNAL_INTERVENTION
        return int(armed)

    def goto(self,
             waypoint,
             duration=rospy.Duration(60, 0),
             low_fuel_trip=False):
        """
        Go to specific waypoint action
        """
        self.wp_reached = -1
        start = rospy.Time.now()
        if not self.set_current_target_wp(waypoint):
            return self.ACTION_FAIL
        auto = self._set_mode_proxy(0, 'auto').mode_sent
        while (rospy.Time.now() - start <
               duration) and (not rospy.is_shutdown()) and (
                   not self.external_intervened) and (not auto):
            auto = self._set_mode_proxy(0, 'auto').mode_sent
            self._rate.sleep()
        self.previous_mode = self.current_mode if auto else self.previous_mode
        self.current_mode = 'AUTO' if auto else self.current_mode
        rospy.loginfo('Setting mode to AUTO ...')
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        while (rospy.Time.now() - start < duration) and not (
                rospy.is_shutdown()) and (not self.external_intervened) and (
                    (waypoint != self.wp_reached)):
            if not low_fuel_trip and self.low_fuel:
                rospy.logwarn('Battery is below minimum voltage!')
                break
            self._rate.sleep()
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        response = int(waypoint == self.wp_reached)
        if not (self.low_fuel or self.external_intervened):
            self.guided_mode(duration=duration)
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        elif self.external_intervened:
            response = self.EXTERNAL_INTERVENTION
        return response

    def return_to_launch(self, duration=rospy.Duration(600, 0)):
        """
        Return to home action
        """
        start = rospy.Time.now()
        rtl_set = self._set_mode_proxy(0, 'rtl').mode_sent
        if rtl_set:
            self.previous_mode = self.current_mode
            self.current_mode = 'RTL'
        rospy.loginfo('Setting mode to RTL ...')
        while (rospy.Time.now() - start <
               duration) and not (rospy.is_shutdown()) and (
                   not self.external_intervened) and self._current_wp != 0:
            if not rtl_set:
                rtl_set = self._set_mode_proxy(0, 'rtl').mode_sent
                if rtl_set:
                    self.previous_mode = self.current_mode
                    self.current_mode = 'RTL'
            if self._status_text == 'Reached destination':
                break
            self._rate.sleep()
        rtl_set = (((self._current_wp == 0) or
                    (self._status_text == 'Reached destination')) and rtl_set)
        if (rospy.Time.now() - start) > duration:
            rtl_set = self.OUT_OF_DURATION
        if self.external_intervened:
            rtl_set = self.EXTERNAL_INTERVENTION

        return int(rtl_set)
