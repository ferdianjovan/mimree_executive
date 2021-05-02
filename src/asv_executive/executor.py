#!/usr/bin/env python

# Other 3rd party packages
import numpy as np
# ROS packages
import rospy
# MAVROS packages
from mavros_msgs.msg import GlobalPositionTarget, HomePosition, State
from mavros_msgs.srv import CommandBool, CommandHome, SetMode
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Float64, Int32, Header
from uav_executive.executor import xy_to_longlat, yaw_ned_to_enu


class ActionExecutor(object):

    INIT_FUEL = 100.0
    MINIMUM_FUEL = 25.0
    EXTERNAL_INTERVENTION = -2
    OUT_OF_DURATION = -1
    ACTION_SUCCESS = 1
    ACTION_FAIL = 0

    def __init__(self, namespace, waypoints, update_frequency=10.):
        """
        A Class that interfaces with MAVROS for executing actions
        """
        self.current_mode = ''
        self.previous_mode = ''
        self.namespace = namespace['name']
        self.fuel_rate_mean = 1.0
        self.fuel_rate_std = 1.0
        self.low_fuel = False
        self.fuel = self.INIT_FUEL
        self.set_battery(namespace['max_fuel'], namespace['min_fuel'],
                         namespace['fuel_rate'])
        self._cancel_action = False
        self.external_intervened = False
        self.state = State()
        self.waypoints = waypoints
        self.home = HomePosition()
        self.global_pose = NavSatFix()
        self.heading = 0.0
        self._current_wp = -1
        self._radius = 1e-04
        self._rate = rospy.Rate(update_frequency)

        # Service proxies
        rospy.loginfo('Waiting for /%s/mavros/set_mode ...' % self.namespace)
        rospy.wait_for_service('/%s/mavros/set_mode' % self.namespace)
        self._set_mode_proxy = rospy.ServiceProxy(
            '/%s/mavros/set_mode' % self.namespace, SetMode)
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
        if len(namespace['uav_onboard']):
            self._uav_home_proxies = {
                uav: rospy.ServiceProxy('/%s/mavros/cmd/set_home' % uav,
                                        CommandHome)
                for uav in namespace['uav_onboard']
            }
            self.uav_home_wp = {
                uav: HomePosition()
                for uav in namespace['uav_onboard']
            }
            self._uav_home_offset = {
                uav: np.ones(4) * float('inf')
                for uav in namespace['uav_onboard']
            }
            self._uav_home_pose_pub = {
                uav: rospy.Publisher(
                    '/%s_launchpad/mavros/global_position/global' % uav,
                    NavSatFix,
                    queue_size=3)
                for uav in namespace['uav_onboard']
            }
            self._uav_home_heading_pub = {
                uav: rospy.Publisher(
                    '/%s_launchpad/mavros/global_position/compass_hdg' % uav,
                    Float64,
                    queue_size=3)
                for uav in namespace['uav_onboard']
            }
            for uav in namespace['uav_onboard']:
                rospy.Subscriber('/%s/mavros/home_position/home' % uav,
                                 HomePosition,
                                 lambda i: self._uav_home_cb(i, uav),
                                 queue_size=1)
            rospy.Timer(2 * self._rate.sleep_dur, self.update_uav_home_pos)

        # Subscribers
        rospy.Subscriber('/%s/mavros/state' % self.namespace,
                         State,
                         self._state_cb,
                         queue_size=1)
        # halt until mavros is connected to a asv
        rospy.loginfo('Waiting for a connection to %s ...' % self.namespace)
        while (not self.state.connected):
            self._rate.sleep()

        rospy.Subscriber('/%s/mavros/global_position/compass_hdg' %
                         self.namespace,
                         Float64,
                         self._heading_cb,
                         queue_size=1)
        rospy.Subscriber('/%s/mavros/home_position/home' % self.namespace,
                         HomePosition,
                         self._home_cb,
                         queue_size=1)
        rospy.Subscriber('/%s/mavros/battery' % self.namespace,
                         BatteryState,
                         self._battery_cb,
                         queue_size=1)
        rospy.Subscriber('/%s/mavros/global_position/global' % self.namespace,
                         NavSatFix,
                         self._global_pose_cb,
                         queue_size=1)

        # Publisher
        self._setpoint_pub = rospy.Publisher('/%s/mavros/setpoint_raw/global' %
                                             self.namespace,
                                             GlobalPositionTarget,
                                             queue_size=3)
        self._rotate_cam = rospy.Publisher('/%s/activate_rotation' %
                                           self.namespace,
                                           Int32,
                                           queue_size=3)

        # Auto call functions
        rospy.Timer(10 * self._rate.sleep_dur, self.intervene_observer)
        rospy.Timer(self._rate.sleep_dur, self.update_wp_position)
        # change mode just to fill self.current_mode and self.previous_mode
        self.guided_mode()
        # Adding initial waypoints' configuration
        self.set_current_location_as_home()

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
        self.low_fuel = (self.fuel <= self.MINIMUM_FUEL *
                         1.5) and not (self._current_wp == 0)

    def calculate_fuel_rate(self, init_fuel, init_time,
                            std_dev_likelihood=1.0):
        """
        Calculate fuel rate consumption
        """
        x = float(init_fuel - self.fuel)
        x = x / (rospy.Time.now() - init_time).secs
        self.fuel_rate_mean = (
            (float(self.fuel_rate_mean) / self.fuel_rate_std**2) +
            (x / std_dev_likelihood**2)) / ((1. / self.fuel_rate_std**2) +
                                            (1. / std_dev_likelihood**2))
        self.fuel_rate_std = 1. / ((1. / self.fuel_rate_std**2) +
                                   (1. / std_dev_likelihood))

    def update_uav_home_pos(self, event):
        """
        Update UAV home position when ASV moves
        """
        for uav in self._uav_home_proxies.keys():
            home_wp = np.array([
                self.uav_home_wp[uav].geo.latitude,
                self.uav_home_wp[uav].geo.longitude,
                self.uav_home_wp[uav].geo.altitude, self.heading
            ])
            if (self._uav_home_offset[uav] == np.ones(4) * float('inf')).all():
                if self.global_pose != NavSatFix():
                    asv_home = np.array([
                        self.global_pose.latitude, self.global_pose.longitude,
                        self.global_pose.altitude, 0.0
                    ])
                    self._uav_home_offset[uav] = home_wp - asv_home
                continue
            # Update the launchpad position relative to last known pos
            heading = (self.heading - self._uav_home_offset[uav][-1])
            launchpad_lat = self.global_pose.latitude + (
                -1 * self._uav_home_offset[uav][1] * np.sin(heading) +
                self._uav_home_offset[uav][0] * np.cos(heading))
            launchpad_long = self.global_pose.longitude + (
                self._uav_home_offset[uav][1] * np.cos(heading) +
                self._uav_home_offset[uav][0] * np.sin(heading))
            launchpad_alt = self.global_pose.altitude + self._uav_home_offset[
                uav][2]
            # Publish the launchpad position and its (asv) heading
            launchpad_pose = NavSatFix()
            launchpad_pose.header = self.global_pose.header
            launchpad_pose.status = self.global_pose.status
            launchpad_pose.latitude = launchpad_lat
            launchpad_pose.longitude = launchpad_long
            launchpad_pose.altitude = launchpad_alt
            self._uav_home_pose_pub[uav].publish(launchpad_pose)
            self._uav_home_heading_pub[uav].publish(
                Float64(self.heading * 180. / np.pi))
            launchpad = np.array([launchpad_lat, launchpad_long])
            if np.linalg.norm(home_wp[:2] - launchpad) > 3e-06:
                self._uav_home_proxies[uav](False, self.heading * 180. / np.pi,
                                            launchpad_lat, launchpad_long,
                                            self._uav_home_offset[uav][2])

    def set_battery(self, init_batt, min_batt, fuel_rate):
        """
        Setting initial fuel and minimum fuel condition
        """
        rospy.loginfo('%s is setting up fuel requirements...' % self.namespace)
        self.INIT_FUEL = init_batt
        self.MINIMUM_FUEL = min_batt
        self.fuel = self.INIT_FUEL
        self.fuel_rate_mean, self.fuel_rate_std = fuel_rate
        self.low_fuel = False

    def intervene_observer(self, event):
        """
        Watch whether human operator intervenes
        """
        mode_status_check = (self.current_mode != '') and (
            self.state.mode not in [self.current_mode, self.previous_mode])
        self.external_intervened = mode_status_check or self._cancel_action

    def update_wp_position(self, event):
        """
        Update ASV position relative to ASV waypoints
        """
        wp = -1
        cur_pos = np.array(
            [self.global_pose.latitude, self.global_pose.longitude])
        for idx, waypoint in enumerate(self.waypoints):
            waypoint = np.array([waypoint['lat'], waypoint['long']])
            if np.linalg.norm(cur_pos - waypoint) < self._radius:
                wp = idx + 1
                break
        if wp == -1 and self.home.header != Header():
            waypoint = np.array(
                [self.home.geo.latitude, self.home.geo.longitude])
            wp = 0 if np.linalg.norm(cur_pos - waypoint) < self._radius else wp
        self._current_wp = wp

    def set_current_location_as_home(self):
        """
        Set current location as the new home location
        """
        response = False
        while (not response) and (not rospy.is_shutdown()):
            response = self._set_home_proxy(True, 0., 0., 0., 0.).success
            self._rate.sleep()
        if response:
            rospy.loginfo(
                '%s is setting current location as the new home ...' %
                self.namespace)
        return response

    def guided_mode(self, duration=rospy.Duration(60, 0), mode='guided'):
        """
        Guided mode action
        """
        start = rospy.Time.now()
        while (rospy.Time.now() - start <
               duration) and (not rospy.is_shutdown()) and (
                   self.state.mode !=
                   mode.upper()) and (not self.external_intervened):
            self._set_mode_proxy(0, mode)
            self._rate.sleep()
        rospy.loginfo('%s changes its mode to %s ...' %
                      (self.namespace, mode.upper()))
        if self.state.mode == mode.upper():
            self.previous_mode = self.current_mode
            self.current_mode = mode.upper()
        response = int(self.state.mode == mode.upper())
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        if self.external_intervened:
            response = self.EXTERNAL_INTERVENTION
        return response

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
            rospy.loginfo('%s is being armed ...' % self.namespace)
        if (rospy.Time.now() - start) > duration:
            armed = self.OUT_OF_DURATION
        if self.external_intervened:
            armed = self.EXTERNAL_INTERVENTION
        return int(armed)

    def return_to_launch(self, duration=rospy.Duration(600, 0)):
        """
        Return to home action
        """
        start = rospy.Time.now()
        rtl_set = self.guided_mode(duration, mode='rtl')
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        while (rospy.Time.now() - start <
               duration) and not (rospy.is_shutdown()) and (
                   not self.external_intervened) and self._current_wp != 0:
            self._rate.sleep()
        rtl_set = (self._current_wp == 0) and rtl_set
        if (rospy.Time.now() - start) > duration:
            rtl_set = self.OUT_OF_DURATION
        if self.external_intervened:
            rtl_set = self.EXTERNAL_INTERVENTION
        return int(rtl_set)

    def goto(self,
             waypoint,
             duration=rospy.Duration(60, 0),
             low_fuel_trip=False):
        """
        Go to specific waypoint action
        """
        init_fuel = self.fuel
        start = rospy.Time.now()
        wp = self.waypoints[waypoint - 1]
        original = GlobalPositionTarget()
        original.header.seq = 1
        original.header.stamp = rospy.Time.now()
        original.header.frame_id = 'map'
        original.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        original.type_mask = 0b001111111000
        original.latitude = wp['lat']
        original.longitude = wp['long']
        original.altitude = 0.0
        original.yaw = yaw_ned_to_enu(wp['yaw'])
        original.yaw_rate = 2.0
        rospy.loginfo("%s is going to waypoint %d, lat: %.5f, long: %.5f" %
                      (self.namespace, waypoint, wp['lat'], wp['long']))
        reached_original = self.goto_coordinate(original,
                                                low_fuel_trip=low_fuel_trip,
                                                radius=self._radius,
                                                duration=duration)
        rospy.loginfo("%s is reaching waypoint %d, lat: %.5f, long: %.5f" %
                      (self.namespace, waypoint, wp['lat'], wp['long']))
        if reached_original == self.ACTION_SUCCESS:
            self.calculate_fuel_rate(init_fuel, start, self.fuel_rate_std)
        return reached_original

    def goto_coordinate(self,
                        target,
                        radius=5e-5,
                        low_fuel_trip=False,
                        duration=rospy.Duration(60, 0)):
        """
        Go to specific global coordinate action
        """
        reached = False
        start = rospy.Time.now()
        self.guided_mode(duration=duration)
        if not self.state.armed:
            duration = duration - (rospy.Time.now() - start)
            start = rospy.Time.now()
            self.arm(duration)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        while (rospy.Time.now() - start <
               duration) and not (rospy.is_shutdown()) and (
                   not self.external_intervened) and (not reached):
            if not low_fuel_trip and self.low_fuel:
                rospy.logwarn('Battery is below minimum voltage!')
                break
            target.header.stamp = rospy.Time.now()
            self._setpoint_pub.publish(target)
            temp = np.array([target.latitude, target.longitude])
            cur_pos = np.array([
                self.global_pose.latitude,
                self.global_pose.longitude,
            ])
            if np.linalg.norm(cur_pos - temp) < radius:
                reached = True
            self._rate.sleep()
        response = int(reached)
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        elif self.external_intervened:
            response = self.EXTERNAL_INTERVENTION
        return response

    def inspect_wt(self, duration=rospy.Duration(600, 0)):
        """
        ASV inspection for a WT that involves pointing a camera
        to the WT while ASV is moving back and forth
        """
        fuel = self.fuel
        start = rospy.Time.now()
        # position where drone is originated in one of the wps
        wp = self.waypoints[self._current_wp - 1]
        original = GlobalPositionTarget()
        original.header.seq = 1
        original.header.stamp = rospy.Time.now()
        original.header.frame_id = 'map'
        original.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        original.type_mask = 0b001111111000
        original.latitude = wp['lat']
        original.longitude = wp['long']
        original.altitude = 0.0
        original.yaw = yaw_ned_to_enu(wp['yaw'])
        original.yaw_rate = 2.0
        rospy.loginfo("%s is scanning a wind turbine..." % self.namespace)
        first_blade = self.blade_inspect(original, [0.0, 90.0, 0.0], duration)
        if first_blade == self.ACTION_SUCCESS:
            self.calculate_fuel_rate(fuel, start, self.fuel_rate_std)
        rospy.loginfo("%s has done the inspection..." % self.namespace)
        return first_blade

    def blade_inspect(self,
                      original,
                      target_position,
                      duration=rospy.Duration(300, 0)):
        """
        Inspecting the blade given the [original] pose to return to
        and end [target] position to scan
        """
        start = rospy.Time.now()
        reached_original = self.ACTION_FAIL
        # rotate the camera
        for _ in range(3):
            self._rotate_cam.publish(Int32(1))
            rospy.sleep(0.1)
        # position where ASV is supposed to be
        heading = self.heading
        offset_x = (target_position[0] * np.cos(heading) +
                    target_position[1] * np.sin(heading))
        offset_y = (-1 * target_position[0] * np.sin(heading) +
                    target_position[1] * np.cos(heading))
        latitude_offset, longitude_offset = xy_to_longlat(
            offset_x, offset_y, self.global_pose.latitude)
        target = GlobalPositionTarget()
        target.header.seq = 1
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'map'
        target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        target.type_mask = 0b001111111000
        target.latitude = self.global_pose.latitude + latitude_offset
        target.longitude = self.global_pose.longitude + longitude_offset
        target.altitude = target_position[2]
        target.yaw = yaw_ned_to_enu(heading)
        target.yaw_rate = 2.0
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        reached_target = self.goto_coordinate(target,
                                              low_fuel_trip=False,
                                              duration=duration)
        rospy.loginfo("%s is returning to %3.5f, %3.5f position..." %
                      (self.namespace, original.latitude, original.longitude))
        if reached_target == self.ACTION_SUCCESS:
            # rotate the camera
            for _ in range(3):
                self._rotate_cam.publish(Int32(-1))
                rospy.sleep(0.1)
            original.header.seq = 1
            original.header.stamp = rospy.Time.now()
            original.header.frame_id = 'map'
            duration = duration - (rospy.Time.now() - start)
            start = rospy.Time.now()
            reached_original = self.goto_coordinate(original,
                                                    low_fuel_trip=False,
                                                    duration=duration)
        # rotate the camera
        for _ in range(3):
            self._rotate_cam.publish(Int32(0))
            rospy.sleep(0.1)
        return np.min([reached_target, reached_original])
