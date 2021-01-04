#!/usr/bin/env python

import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import (GlobalPositionTarget, HomePosition, State,
                             StatusText, Waypoint, WaypointReached)
from mavros_msgs.srv import (CommandBool, CommandHome, CommandLong, CommandTOL,
                             SetMode, WaypointClear, WaypointPush,
                             WaypointSetCurrent)
from sensor_msgs.msg import BatteryState, NavSatFix, Range
from std_msgs.msg import Float64, Header


def yaw_ned_to_enu(radian):
    """
    The incoming yaw is in NED frame
    yaw ENU <-> NED using Quaternion:
    Swap X and Y, Invert Z
    Leave W be
    Rotate 90 degrees about z axis (yaw)
    """
    q_ned = tf.transformations.quaternion_from_euler(0, 0, radian)
    q_enu = np.array([q_ned[1], q_ned[0], -q_ned[2], q_ned[3]])
    q_90 = tf.transformations.quaternion_from_euler(0, 0, np.pi / 2.)
    q_enu = tf.transformations.quaternion_multiply(quaternion0=q_enu,
                                                   quaternion1=q_90)
    yaw_enu = tf.transformations.euler_from_quaternion(q_enu)[-1]
    return yaw_enu


def xy_to_longlat(x, y, latitude):
    """
    Convert x, y offset position to latitude and longitude equivalent
    using ENU coordinate system
    """
    earth = 6378.137
    m = (1.0 / (np.pi / 180.0 * earth)) / 1000.0
    latitude_offset = (y * m)
    longitude_offset = (x * m) / np.cos(latitude * (np.pi / 180.))
    return latitude_offset, longitude_offset


class ActionExecutor(object):
    MINIMUM_VOLTAGE = 12.19
    MINIMUM_ALTITUDE = 0.5
    INIT_VOLTAGE = 12.587
    EXTERNAL_INTERVENTION = -2
    OUT_OF_DURATION = -1
    ACTION_SUCCESS = 1
    ACTION_FAIL = 0

    def __init__(self,
                 namespace,
                 waypoints,
                 asv_carrier='',
                 update_frequency=10.):
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
        self.heading = 0.0
        self.waypoints = list()
        self.home = HomePosition()
        self.global_pose = NavSatFix()
        self.battery_voltages = [self.INIT_VOLTAGE for _ in range(100)]
        self.low_battery = False
        self._waypoints = waypoints
        self._current_wp = -1
        self._rel_alt = [0. for _ in range(20)]
        self._rangefinder = [-1. for _ in range(10)]
        self._min_range = -1.
        self._status_text = ''
        self._arm_status = [False for _ in range(5)]
        self.asv_carrier = asv_carrier
        self.target_heading = [0.0 for _ in range(20)]
        self.target_global_pose = [NavSatFix() for _ in range(20)]

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

        # Publisher
        self._setpoint_pub = rospy.Publisher('/%s/mavros/setpoint_raw/global' %
                                             self.namespace,
                                             GlobalPositionTarget,
                                             queue_size=10)

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
        rospy.Subscriber('/%s/mavros/global_position/compass_hdg' %
                         self.namespace,
                         Float64,
                         self._heading_cb,
                         queue_size=10)
        # Auto call functions
        rospy.Timer(self._rate.sleep_dur, self.update_wp_position)
        rospy.Timer(self._rate.sleep_dur, self.update_landing_status)
        rospy.Timer(10 * self._rate.sleep_dur, self.intervene_observer)
        rospy.loginfo('Adding WPs ...')
        # Adding initial waypoints' configuration
        self.set_current_location_as_home()
        while not self.add_waypoints():
            self._rate.sleep()

    def _heading_cb(self, msg):
        """
        UAV heading callback
        """
        self.heading = msg.data / 180. * np.pi

    def _rangefinder_cb(self, msg):
        """
        Rangefinder call back
        """
        self._rangefinder.append(msg.range)
        self._rangefinder = self._rangefinder[1:]
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
        self._rel_alt.append(msg.data)
        self._rel_alt = self._rel_alt[1:]
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
        self.battery_voltages[msg.header.seq %
                              len(self.battery_voltages)] = msg.voltage
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
            if np.linalg.norm(cur_pos - temp) < 1e-5 and alt_diff < 0.5:
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
        self.external_intervened = mode_status_check or self._cancel_action

    def update_landing_status(self, event):
        """
        Automated update landing (or flying) status
        """
        landed = (not self.state.armed)
        if self._min_range > -1.:
            self.landed = (self.rangefinder <=
                           (self._min_range + 0.1)) or landed
        else:
            self.landed = landed or (self.rel_alt <= 0.1)
        # elif self._status_text == 'PreArm: Gyros not calibrated':
        #     self.landed = (False in self._arm_status)

    def add_waypoints(self, index=0):
        """
        Setting up waypoints for the UAV
        """
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
        response = False
        while (not response) and (not rospy.is_shutdown()):
            response = self._set_home_proxy(True, 0., 0., 0., 0.).success
            self._rate.sleep()
        if response:
            rospy.loginfo('Setting current location as the new home ...')
        return response

    def guided_mode(self, duration=rospy.Duration(60, 0), mode='guided'):
        """
        [Guided] mode action
        """
        start = rospy.Time.now()
        while (rospy.Time.now() - start <
               duration) and (not rospy.is_shutdown()) and (
                   self.state.mode !=
                   mode.upper()) and (not self.external_intervened):
            self._set_mode_proxy(0, mode)
            self._rate.sleep()
        rospy.loginfo('UAV changes its mode to %s ...' % mode.upper())
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
        self.guided_mode(duration)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        while (rospy.Time.now() - start <
               duration) and (not rospy.is_shutdown()) and (
                   not self.state.armed) and (not self.external_intervened):
            self._arming_proxy(True)
            self._rate.sleep()
        rospy.loginfo('Arming the UAV ...')
        response = self.state.armed
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        if self.external_intervened:
            response = self.EXTERNAL_INTERVENTION
        return int(response)

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
            while (rospy.Time.now() - start < duration) and (
                    not rospy.is_shutdown()
            ) and abs(self.rel_alt - altitude) > 0.5 and (
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
        wp = self.waypoints[waypoint]
        target = GlobalPositionTarget()
        target.header.seq = 1
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'map'
        target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        # this type of masking will ignore accel and veloci
        target.type_mask = 0b001111111000
        target.latitude = wp.x_lat
        target.longitude = wp.y_long
        target.altitude = wp.z_alt
        target.yaw = yaw_ned_to_enu(wp.param4)
        target.yaw_rate = 2.0
        start = rospy.Time.now()
        response = self.goto_coordinate(target, low_battery_trip, duration)
        if response == self.ACTION_SUCCESS:
            duration = duration - (rospy.Time.now() - start)
            # rospy.loginfo(min(duration, rospy.Duration(wp.param1)).secs)
            rospy.sleep(min(duration, rospy.Duration(wp.param1)))
        return response

    def goto_coordinate(self,
                        target,
                        low_battery_trip=False,
                        duration=rospy.Duration(60, 0)):
        """
        Go to specific global coordinate action
        """
        reached = False
        start = rospy.Time.now()
        self.guided_mode(duration=duration)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        while (rospy.Time.now() - start <
               duration) and not (rospy.is_shutdown()) and (
                   not self.external_intervened) and (not reached):
            if not low_battery_trip and self.low_battery:
                rospy.logwarn('Battery is below minimum voltage!')
                break
            target.header.stamp = rospy.Time.now()
            self._setpoint_pub.publish(target)
            temp = np.array([target.latitude, target.longitude])
            cur_pos = np.array([
                self.global_pose.latitude,
                self.global_pose.longitude,
            ])
            if np.linalg.norm(cur_pos - temp) < 5e-6 and abs(
                    self._rel_alt[-1] - target.altitude) < 0.5:
                reached = True
            self._rate.sleep()
        response = int(reached)
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
        self.guided_mode(duration=duration)
        duration = duration - (rospy.Time.now() - start)
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

    def return_to_launch(self,
                         monitor_home=False,
                         duration=rospy.Duration(600, 0)):
        """
        Return to home action
        """
        start = rospy.Time.now()
        if not monitor_home or not self.home_moved:
            self.guided_mode(duration, 'rtl')
        self._rate.sleep()
        while (rospy.Time.now() - start <
               duration) and not (rospy.is_shutdown()) and (
                   not self.external_intervened) and (not self.landed):
            cond = monitor_home and self.home_moved
            if cond or self.current_mode != 'RTL':
                duration = duration - (rospy.Time.now() - start)
                start = rospy.Time.now()
                self.land_on_asv(duration)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        disarm = self.request_arm(True, duration)
        landed = int(self.landed and disarm == self.ACTION_SUCCESS)
        if (rospy.Time.now() - start) > duration:
            landed = self.OUT_OF_DURATION
        if self.external_intervened:
            landed = self.EXTERNAL_INTERVENTION
        return landed

    def _target_global_pose_cb(self, msg):
        """
        Target-to-follow global position callback
        """
        self.target_global_pose.append(msg)
        self.target_global_pose = self.target_global_pose[1:]

    def _target_heading_cb(self, msg):
        """
        Target-to-follow heading callback
        """
        self.target_heading.append((msg.data / 180. * np.pi))
        self.target_heading = self.target_heading[1:]

    def predict_target_pose(self, predicted_pose_step=4):
        """
        Predicting the geo position of ASV at @predicted_pose_step time steps
        """
        len_poses = len(self.target_global_pose)
        latitudes = np.array(
            [pose.latitude for pose in self.target_global_pose])
        longitudes = np.array(
            [pose.longitude for pose in self.target_global_pose])
        altitudes = np.array(
            [pose.altitude for pose in self.target_global_pose])
        time_units = range(len_poses)
        lat_model = np.poly1d(np.polyfit(time_units, latitudes, 3))
        long_model = np.poly1d(np.polyfit(time_units, longitudes, 3))
        alt_model = np.poly1d(np.polyfit(time_units, altitudes, 3))
        est_lat = lat_model(len_poses + predicted_pose_step)
        est_long = long_model(len_poses + predicted_pose_step)
        est_alt = alt_model(len_poses + predicted_pose_step)
        return est_lat, est_long, est_alt

    def follow_target(self,
                      target,
                      home=False,
                      offset=[0., 0., 0.],
                      follow_duration=rospy.Duration(60, 0),
                      duration=rospy.Duration(600, 0)):
        """
        Follow a target and keep an @offset distance for @follow_duration
        @offset=[x, y, z] in (m) assuming the target heading 0 degree is north
        Following @home does not care about battery
        """
        # Start and set to guide mode
        start = rospy.Time.now()
        self.guided_mode(duration=duration)
        # Collect pose and heading of the target
        self.target_heading = [0.0 for _ in range(len(self.target_heading))]
        self.target_global_pose = [
            NavSatFix() for _ in range(len(self.target_global_pose))
        ]
        pose_sub = rospy.Subscriber('/%s/mavros/global_position/global' %
                                    target,
                                    NavSatFix,
                                    self._target_global_pose_cb,
                                    queue_size=10)
        head_sub = rospy.Subscriber('/%s/mavros/global_position/compass_hdg' %
                                    target,
                                    Float64,
                                    self._target_heading_cb,
                                    queue_size=10)
        # Start following the target
        followed_duration = rospy.Duration(0, 0)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        while (rospy.Time.now() - start < duration) and not (
                rospy.is_shutdown()) and (not self.external_intervened) and (
                    followed_duration < follow_duration):
            if self.low_battery and not home:
                rospy.logwarn('Battery is below minimum voltage!!!')
                break
            heading = self.target_heading[-1]
            if self.target_global_pose[-1] == NavSatFix():
                self._rate.sleep()
                continue
            elif self.target_global_pose[0] == NavSatFix():
                latitude = self.target_global_pose[-1].latitude
                longitude = self.target_global_pose[-1].longitude
                altitude = self.target_global_pose[-1].altitude
            else:
                latitude, longitude, altitude = self.predict_target_pose(5)
            # convert offset from meters to lat and long in ENU system
            offset_x = (offset[0] * np.cos(heading) +
                        offset[1] * np.sin(heading))
            offset_y = (-1 * offset[0] * np.sin(heading) +
                        offset[1] * np.cos(heading))
            latitude_offset, longitude_offset = xy_to_longlat(
                offset_x, offset_y, latitude)
            # Setup target position
            target = GlobalPositionTarget()
            target.header.seq = 1
            target.header.stamp = rospy.Time.now()
            target.header.frame_id = 'map'
            target.type_mask = 0b001111111000
            target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
            # Due to yaw_ned_to_enu conversion, the sin and cos are flipped
            target.latitude = latitude + latitude_offset
            target.longitude = longitude + longitude_offset
            if (self._min_range >
                    -1) and (self._rangefinder[-1] - self._min_range <
                             self.MINIMUM_ALTITUDE):
                rospy.logerr("%s is %.3f meters away from impact!" %
                             (self.namespace,
                              (self._rangefinder[-1] - self._min_range)))
                target_alt = self._rel_alt[-1]
            else:
                target_alt = (altitude - self.home.geo.altitude + offset[2])
            target.altitude = target_alt
            target.yaw = yaw_ned_to_enu(heading)
            target.yaw_rate = 2.0
            # Publish aimed position
            self._setpoint_pub.publish(target)
            # Check uav position with target
            latitude_offset, longitude_offset = xy_to_longlat(
                offset_x, offset_y, self.target_global_pose[-1].latitude)
            target_pose = np.array([
                self.target_global_pose[-1].latitude + latitude_offset,
                self.target_global_pose[-1].longitude + longitude_offset
            ])
            uav_pose = np.array([
                self.global_pose.latitude,
                self.global_pose.longitude,
            ])
            if abs(target_alt - self._rel_alt[-1]) < 0.5:
                if np.linalg.norm(uav_pose - target_pose) < 5e-6:
                    rospy.loginfo("%s has found target, following %d seconds" %
                                  (self.namespace, followed_duration.secs))
                    followed_duration += self._rate.sleep_dur
                else:
                    followed_duration = rospy.Duration(0, 0)
                    rospy.loginfo(
                        "Target is out of range, resetting the duration")
            self._rate.sleep()
        # Unregister subscriptions
        pose_sub.unregister()
        head_sub.unregister()
        # Prepare response
        response = int(followed_duration >= follow_duration)
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        elif self.external_intervened:
            response = self.EXTERNAL_INTERVENTION
        return response

    def land_on_asv(self, duration=rospy.Duration(600, 0)):
        """
        Land on ASV position
        """
        start = rospy.Time.now()
        # First step approaching the launchpad (side or behind)
        offset = [
            0.0,
            rospy.get_param("~dist_initial_landing", 5.0),
            rospy.get_param("~altitude_initial_landing", 10.)
        ]
        if "behind" in rospy.get_param("~landing_approach", "side"):
            rospy.loginfo("Approaching ASV from behind ...")
        else:
            offset = [offset[1], offset[0], offset[2]]
            rospy.loginfo("Approaching ASV from side ...")
        self.follow_target('%s_launchpad' % self.namespace, True, offset,
                           rospy.Duration(3, 0), duration)
        # Second step approaching the launchpad (side or behind)
        landed = self.ACTION_FAIL
        offset = [0.0, 0.0, rospy.get_param("~takeoff_altitude", 10.)]
        rospy.loginfo("Trying to land on ASV ...")
        while (rospy.Time.now() - start <
               duration) and not (rospy.is_shutdown()) and (
                   not self.external_intervened) and (not self.landed):
            if landed == self.ACTION_SUCCESS:
                self._rate.sleep()
                continue
            duration = duration - (rospy.Time.now() - start)
            start = rospy.Time.now()
            self.follow_target('%s_launchpad' % self.namespace, True, offset,
                               rospy.Duration(3, 0), duration)
            rangefinder_ok = (self._min_range > -1)
            rangefinder_ok = rangefinder_ok and (
                (self.rangefinder - self._min_range) <
                (self.MINIMUM_ALTITUDE + 0.1))
            rel_alt_ok = (self._rel_alt[-1] <
                          self.MINIMUM_ALTITUDE) and (self._min_range <= -1)
            if rel_alt_ok or rangefinder_ok:
                rospy.loginfo("UAV is landing ...")
                duration = duration - (rospy.Time.now() - start)
                start = rospy.Time.now()
                landed = self.guided_mode(duration, mode='land')
                self._rate.sleep()
            elif offset[-1] <= self.MINIMUM_ALTITUDE:
                offset[-1] -= 0.1
            else:
                offset[-1] = self.MINIMUM_ALTITUDE
        # Status check to report
        landed = int(self.landed)
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

    def inspect_wt(self, duration=rospy.Duration(900, 0)):
        """
        UAV Inspection for a WT that involves flying close and parallel
        to each blade of a WT
        """
        start = rospy.Time.now()
        # position where drone is originated in one of the wps
        wp = self.waypoints[self._current_wp]
        original = GlobalPositionTarget()
        original.header.seq = 1
        original.header.stamp = rospy.Time.now()
        original.header.frame_id = 'map'
        original.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        original.type_mask = 0b001111111000
        original.latitude = wp.x_lat
        original.longitude = wp.y_long
        original.altitude = wp.z_alt
        original.yaw = yaw_ned_to_enu(wp.param4)
        original.yaw_rate = 2.0
        rospy.loginfo("Scan first blade...")
        first_blade = self.blade_inspect(original, original.altitude,
                                         [30.0, 0.0, 0.0], duration)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        rospy.loginfo("Scan second blade...")
        second_blade = self.blade_inspect(original, original.altitude, [
            30.0 * np.cos(138. / 180.0 * np.pi), 0.000,
            85.6 * np.sin(120. / 180.0 * np.pi)
        ], duration)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        rospy.loginfo("Scan third blade...")
        third_blade = self.blade_inspect(original, original.altitude, [
            30.0 * np.cos(234. / 180.0 * np.pi), 0.000,
            85.6 * np.sin(240. / 180.0 * np.pi)
        ], duration)
        rospy.loginfo("Inspection is done...")
        return np.min([first_blade, second_blade, third_blade])

    def blade_inspect(self,
                      original,
                      relative_altitude,
                      target_position,
                      duration=rospy.Duration(300, 0)):
        """
        Inspecting the blade given the [original] pose to return to
        and end [target] position to scan
        """
        start = rospy.Time.now()
        reached_original = self.ACTION_FAIL
        # position where drone is supposed to be
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
        target.altitude = self.rel_alt + target_position[2]
        target.yaw = yaw_ned_to_enu(heading)
        target.yaw_rate = 2.0
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        reached_target = self.goto_coordinate(target, duration)
        rospy.loginfo("Return to %3.5f, %3.5f position..." %
                      (original.latitude, original.longitude))
        if reached_target == self.ACTION_SUCCESS:
            original.header.seq = 1
            original.header.stamp = rospy.Time.now()
            original.header.frame_id = 'map'
            duration = duration - (rospy.Time.now() - start)
            start = rospy.Time.now()
            reached_original = self.goto_coordinate(original, duration)
        return np.min([reached_target, reached_original])
