#!/usr/bin/env python

import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseArray, PoseStamped
from mavros_msgs.msg import GlobalPositionTarget, HomePosition, State
from mavros_msgs.srv import CommandHome, CommandTOL, SetMode
from sensor_msgs.msg import BatteryState, Imu, NavSatFix, Range
from simple_pid import PID
from std_msgs.msg import Float64, Header, Int64, String
from uav_executive.lhm_executor import ActionExecutor as LHMExecutor


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

    MINIMUM_VOLTAGE = 10.0
    MINIMUM_ALTITUDE = 0.2
    INIT_VOLTAGE = 100.0
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
        self.battery_rate_mean = 1.0
        self.battery_rate_std = 1.0
        self.battery_voltages = list()
        self.low_battery = False
        self.set_battery(namespace['max_fuel'], namespace['min_fuel'],
                         namespace['fuel_rate'])
        self._cancel_action = False
        self.external_intervened = False
        self.state = State()
        self.home = HomePosition()
        self.global_pose = NavSatFix()
        self.local_pose = PoseStamped()
        self.heading = 0.0
        self.waypoints = [None]
        self._current_wp = -1
        self._radius = 1e-5
        self._rate = rospy.Rate(update_frequency)
        # UAV specific variables
        self.irr_name = namespace['irr_attached']
        self._irr_ready_to_be_picked = 0
        self.landed = True
        self.home_moved = False
        self.rel_alt = 0.
        self.rangefinder = -1.
        self._alt_radius = 0.5
        self._rel_alt = [0. for _ in range(5)]
        self._rangefinder = [-1. for _ in range(5)]
        self._min_range = -1.
        self.deploy_msg = Int64()
        self.target_heading = [0.0 for _ in range(5)]
        self.target_global_pose = [NavSatFix() for _ in range(5)]
        self.target_imu = [Imu() for _ in range(5)]
        # LHM Controller
        if namespace['retrieve_system'] and (
                "simulation" not in rospy.get_param("~scenario_type",
                                                    "simulation")):
            self.lhm = LHMExecutor(self.namespace, update_frequency)
        if "simulation" in rospy.get_param("~scenario_type", "simulation"):
            self.blade_pose = [[0., 0., 0.] for _ in range(10)]
            rospy.Subscriber('/%s/edge_wt_detector' % self.namespace,
                             PoseArray,
                             self._wt_cb,
                             queue_size=1)
            # simulated winch system
            self._lhm_pub = rospy.Publisher('/attach_plugin/attach',
                                            String,
                                            queue_size=3)

        # Subscribers
        rospy.Subscriber('/%s/mavros/state' % self.namespace,
                         State,
                         self._state_cb,
                         queue_size=1)
        # halt until mavros is connected to a uav
        rospy.loginfo('Waiting for a connection to %s ...' % self.namespace)
        while (not self.state.connected):
            self._rate.sleep()
        rospy.Subscriber('/%s/mavros/home_position/home' % self.namespace,
                         HomePosition,
                         self._home_cb,
                         queue_size=1)
        rospy.Subscriber('/%s/mavros/global_position/rel_alt' % self.namespace,
                         Float64,
                         self._relative_alt_cb,
                         queue_size=1)
        rospy.Subscriber('/%s/mavros/modified_battery' % self.namespace,
                         BatteryState,
                         self._battery_cb,
                         queue_size=1)
        rospy.Subscriber('/%s/mavros/global_position/raw/unfix' %
                         self.namespace,
                         NavSatFix,
                         self._global_pose_cb,
                         queue_size=1)
        rospy.Subscriber('/%s/mavros/local_position/pose' % self.namespace,
                         PoseStamped,
                         self._local_pose_cb,
                         queue_size=1)
        rospy.Subscriber('/%s/mavros/rangefinder/rangefinder' % self.namespace,
                         Range,
                         self._rangefinder_cb,
                         queue_size=1)
        rospy.Subscriber('/%s/mavros/global_position/compass_hdg' %
                         self.namespace,
                         Float64,
                         self._heading_cb,
                         queue_size=1)

        # Service proxies
        rospy.loginfo('Waiting for /%s/mavros/cmd/set_home ...' %
                      self.namespace)
        rospy.wait_for_service('/%s/mavros/cmd/set_home' % self.namespace)
        self._set_home_proxy = rospy.ServiceProxy(
            '/%s/mavros/cmd/set_home' % self.namespace, CommandHome)

        rospy.loginfo('Waiting for /%s/mavros/set_mode ...' % self.namespace)
        rospy.wait_for_service('/%s/mavros/set_mode' % self.namespace)
        self._set_mode_proxy = rospy.ServiceProxy(
            '/%s/mavros/set_mode' % self.namespace, SetMode)
        rospy.loginfo('Waiting for /%s/mavros/cmd/takeoff ...' %
                      self.namespace)
        rospy.wait_for_service('/%s/mavros/cmd/takeoff' % self.namespace)
        self._takeoff_proxy = rospy.ServiceProxy(
            '/%s/mavros/cmd/takeoff' % self.namespace, CommandTOL)
        # Publisher
        self._setpoint_pub = rospy.Publisher('/%s/mavros/setpoint_raw/global' %
                                             self.namespace,
                                             GlobalPositionTarget,
                                             queue_size=1)

        rospy.sleep(3)
        self.set_current_location_as_home()
        # Adding initial waypoints' configuration
        while self.waypoints[0] is None:
            self._rate.sleep()
        self.waypoints = self.waypoints + waypoints
        # Auto call functions
        rospy.Timer(self._rate.sleep_dur, self.update_wp_position)
        rospy.Timer(self._rate.sleep_dur, self.update_landing_status)
        rospy.Timer(10 * self._rate.sleep_dur, self.intervene_observer)
        # change mode just to fill self.current_mode and self.previous_mode
        self.guided_mode()

    def _wt_cb(self, msg):
        """
        Blade position estimate
        """
        if len(msg.poses):
            self.blade_pose.append([
                msg.poses[-1].position.x, msg.poses[-1].position.y,
                msg.poses[-1].position.z
            ])
            self.blade_pose = self.blade_pose[1:]

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

    def _state_cb(self, msg):
        """
        UAV current state callback
        """
        if self.current_mode == '':
            self.current_mode = msg.mode
        self.state = msg

    def _home_cb(self, msg):
        """
        Home position callback
        """
        if self.home.header != Header():
            prev_home = np.array(
                [self.home.geo.latitude, self.home.geo.longitude])
            current_home = np.array([msg.geo.latitude, msg.geo.longitude])
            if np.linalg.norm(prev_home - current_home) > self._radius:
                self.home_moved = True
                self.home = msg
            else:
                self.home_moved = False
        else:
            self.home = msg
        self.home.geo.altitude = self.global_pose.altitude
        self.waypoints[0] = {
            'yaw': self.heading,
            'lat': msg.geo.latitude,
            'long': msg.geo.longitude,
            'rel_alt': rospy.get_param("~takeoff_altitude", 10.)
        }

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

    def _local_pose_cb(self, msg):
        """
        UAV local position callback
        """
        self.local_pose = msg

    def _battery_cb(self, msg):
        """
        UAV Battery state callback
        """
        # self.battery_voltages[msg.header.seq %
        #                       len(self.battery_voltages)] = msg.voltage
        self.battery_voltages[msg.header.seq % len(
            self.battery_voltages)] = msg.percentage * 100.
        # delta = self.INIT_VOLTAGE - self.MINIMUM_VOLTAGE
        # self.low_battery = (np.mean(self.battery_voltages) <=
        #                     (self.MINIMUM_VOLTAGE +
        #                      (0.1 * delta))) and (self._current_wp != 0)
        self.low_battery = (np.mean(self.battery_voltages) <=
                            self.MINIMUM_VOLTAGE * 1.5) and (self._current_wp
                                                             != 0)

    def set_battery(self, init_batt, min_batt, batt_rate):
        """
        Setting initial battery and minimum battery condition
        """
        rospy.loginfo('%s is setting up battery requirements...' %
                      self.namespace)
        self.INIT_VOLTAGE = init_batt
        self.MINIMUM_VOLTAGE = min_batt
        self.battery_voltages = [self.INIT_VOLTAGE for _ in range(20)]
        self.battery_rate_mean, self.battery_rate_std = batt_rate
        self.low_battery = False

    def intervene_observer(self, event):
        """
        Watch whether human operator intervenes
        """
        mode_status_check = (self.current_mode != '') and (
            self.state.mode not in [self.current_mode, self.previous_mode])
        self.external_intervened = mode_status_check or self._cancel_action

    def update_wp_position(self, event):
        """
        Update UAV position relative to UAV waypoints
        """
        wp = -1
        cur_pos = np.array(
            [self.global_pose.latitude, self.global_pose.longitude])
        for idx, waypoint in enumerate(self.waypoints):
            temp = np.array([waypoint['lat'], waypoint['long']])
            alt_diff = abs(self._rel_alt[-1] - waypoint['rel_alt'])
            if idx == 0 and (np.linalg.norm(cur_pos - temp) < self._radius):
                wp = idx
                break
            elif (np.linalg.norm(cur_pos - temp) <
                  self._radius) and (alt_diff < self._alt_radius):
                wp = idx
                break
        self._current_wp = wp

    def update_landing_status(self, event):
        """
        Automated update landing (or flying) status
        """
        landed = (not self.state.armed)
        if self.irr_name == '' and self._min_range > -1.:
            self.landed = (self.rangefinder <=
                           (self._min_range + 0.1)) or landed
        else:
            self.landed = landed or (self.rel_alt <= 0.1)

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
        [Guided] mode action
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

    def calculate_batt_rate(self, init_batt, init_time,
                            std_dev_likelihood=2.0):
        """
        Calculate battery rate consumption
        """
        x = float(init_batt - np.mean(self.battery_voltages))
        x = x / (rospy.Time.now() - init_time).secs
        print("bat mean: %.5f, bat std: %.5f" %
              (self.battery_rate_mean, self.battery_rate_std))
        std = 1.0 / (1.0 / (self.battery_rate_std**2) + 1.0 /
                     (std_dev_likelihood**2))
        self.battery_rate_mean = (float(self.battery_rate_mean) /
                                  (self.battery_rate_std**2) + x /
                                  (std_dev_likelihood**2)) * std
        self.battery_rate_std = np.max([0.001, std])
        print("bat mean: %.5f, bat std: %.5f" %
              (self.battery_rate_mean, self.battery_rate_std))

    def takeoff(self, altitude, duration=rospy.Duration(60, 0)):
        """
        Take off action
        """
        battery = np.mean(self.battery_voltages)
        init_time = rospy.Time.now()
        start = rospy.Time.now()
        took_off = True
        if hasattr(self, 'lhm'):
            took_off = took_off and (self.lhm.close_hook(
                duration=duration) == self.ACTION_SUCCESS)
            duration = duration - (rospy.Time.now() - start)
            start = rospy.Time.now()
            took_off = took_off and (
                self.lhm.takeoff_preparation(duration) == self.ACTION_SUCCESS)
            duration = duration - (rospy.Time.now() - start)
            start = rospy.Time.now()
        elif hasattr(self, '_lhm_pub') and self.irr_name != "":
            for _ in range(3):
                self._lhm_pub.publish('%s,body,%s,base_link,1' %
                                      (self.namespace, self.irr_name))
                self._rate.sleep()
        took_off = took_off and (self.request_arm(
            duration=duration) == self.ACTION_SUCCESS)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        if self.landed and took_off:
            took_off = self._takeoff_proxy(0.1, 0, 0, 0, altitude).success
            if took_off:
                rospy.loginfo('%s is taking off to %d meter height' %
                              (self.namespace, altitude))
            while (rospy.Time.now() - start < duration) and (
                    not rospy.is_shutdown()
            ) and abs(self.rel_alt - altitude) > 0.5 and (
                    not self.external_intervened):
                if not took_off:
                    took_off = self._takeoff_proxy(0.1, 0, 0, 0,
                                                   altitude).success
                    if took_off:
                        rospy.loginfo('%s is taking off to %d meter height' %
                                      (self.namespace, altitude))
                if (self.rel_alt > (altitude / 2.)) and (np.mean(
                        np.diff(self._rel_alt)) <= 0.08):
                    took_off = True
                    rospy.loginfo('%s took-off to %d meter height' %
                                  (self.namespace, altitude))
                    break
                if self.low_battery:
                    rospy.logwarn('Battery is below minimum voltage!')
                    break
                self._rate.sleep()
        took_off = self.ACTION_SUCCESS if (
            (abs(self.rel_alt - altitude) <= 0.5) and
            (not took_off)) else took_off
        self.calculate_batt_rate(battery, init_time, self.battery_rate_std)
        if (rospy.Time.now() - start) > duration:
            took_off = self.OUT_OF_DURATION
        if self.external_intervened:
            took_off = self.EXTERNAL_INTERVENTION
        return int(took_off)

    def goto(self,
             waypoint,
             duration=rospy.Duration(60, 0),
             low_battery_trip=False,
             calculate_fuel_consumption=True):
        """
        Go to specific predefined waypoints
        """
        init_batt = np.mean(self.battery_voltages)
        init_time = rospy.Time.now()
        start = rospy.Time.now()
        if hasattr(self, 'lhm'):
            self.lhm.swing_reduction(duration)
            duration = duration - (rospy.Time.now() - start)
            start = rospy.Time.now()
        wp = self.waypoints[waypoint]
        target = GlobalPositionTarget()
        target.header.seq = 1
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'map'
        target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        # this type of masking will ignore accel and veloci
        target.type_mask = 0b001111111000
        target.latitude = wp['lat']
        target.longitude = wp['long']
        target.altitude = wp['rel_alt']
        target.yaw = yaw_ned_to_enu(wp['yaw'])
        target.yaw_rate = 0.2
        rospy.loginfo("%s is going to waypoint %d, lat: %.5f, long: %.5f" %
                      (self.namespace, waypoint, wp['lat'], wp['long']))
        response = self.goto_coordinate(target,
                                        radius=self._radius,
                                        alt_radius=self._alt_radius,
                                        low_battery_trip=low_battery_trip,
                                        duration=duration)
        rospy.loginfo("%s is reaching waypoint %d, lat: %.5f, long: %.5f" %
                      (self.namespace, waypoint, wp['lat'], wp['long']))
        if calculate_fuel_consumption:
            self.calculate_batt_rate(init_batt, init_time,
                                     self.battery_rate_std)
        return response

    def goto_coordinate(self,
                        target,
                        radius=5e-5,
                        alt_radius=0.5,
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
            if np.linalg.norm(cur_pos - temp) < radius and abs(
                    self._rel_alt[-1] - target.altitude) < alt_radius:
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
        if not disarm:
            self.guided_mode(duration=duration)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        if not disarm:
            rospy.loginfo('Waiting for %s to be ARMED ...' % self.namespace)
        else:
            rospy.loginfo('Waiting for %s to be DISARMED ...' % self.namespace)
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
        battery = np.mean(self.battery_voltages)
        init_time = rospy.Time.now()
        start = rospy.Time.now()
        unloaded = True
        if self.irr_name != '':
            unloaded = self.dropping_irr(duration) == self.ACTION_SUCCESS
            if not unloaded:
                return self.ACTION_FAIL
            duration = duration - (rospy.Time.now() - start)
            start = rospy.Time.now()
        if not monitor_home or not self.home_moved:
            self.guided_mode(duration, 'rtl')
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
        self.calculate_batt_rate(battery, init_time, self.battery_rate_std)
        if (rospy.Time.now() - start) > duration:
            landed = self.OUT_OF_DURATION
        if self.external_intervened:
            landed = self.EXTERNAL_INTERVENTION
        return landed

    def dropping_irr(self,
                     duration=rospy.Duration(600, 0),
                     offset=[0., -2., 0.]):
        """
        Fly to the backside of the landing zone, drop irr,
        and come back to original position
        """
        start = rospy.Time.now()
        self.guided_mode(duration=duration)
        heading = self.heading
        wp = self.waypoints[0]
        # Flying side
        offset_x = (offset[0] * np.cos(heading) + offset[1] * np.sin(heading))
        offset_y = (-1. * offset[0] * np.sin(heading) +
                    offset[1] * np.cos(heading))
        latitude_offset, longitude_offset = xy_to_longlat(
            offset_x, offset_y, wp['lat'])
        target = GlobalPositionTarget()
        target.header.seq = 1
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'map'
        target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        target.type_mask = 0b111111111000
        target.latitude = wp['lat'] + latitude_offset
        target.longitude = wp['long'] + longitude_offset
        target.altitude = 2.2 if hasattr(self, 'lhm') else 0.5
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        rospy.loginfo("%s is flying side to drop load ..." % self.namespace)
        go_side = self.goto_coordinate(target,
                                       radius=self._radius,
                                       duration=duration)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        rospy.loginfo("%s is dropping load ..." % self.namespace)
        if hasattr(self, 'lhm'):
            hook_open = self.lhm.open_hook(duration=duration)
        elif hasattr(self, '_lhm_pub'):
            for _ in range(3):
                self._lhm_pub.publish('%s,body,%s,base_link,0' %
                                      (self.namespace, self.irr_name))
                self._rate.sleep()
            hook_open = self.ACTION_SUCCESS
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        return_back = self.goto(0, duration, True)
        duration = duration - (rospy.Time.now() - start)
        if hasattr(self, 'lhm'):
            lhm_prepared = self.lhm.landing_preparation(duration)
        elif hasattr(self, '_lhm_pub'):
            lhm_prepared = self.ACTION_SUCCESS
        response = np.min([hook_open, go_side, lhm_prepared, return_back])
        if response == self.ACTION_SUCCESS:
            self.irr_name = ''
        return response

    def inspect_wt(self, duration=rospy.Duration(900, 0)):
        """
        UAV Inspection for a WT that involves flying close and parallel
        to each blade of a WT
        """
        battery = np.mean(self.battery_voltages)
        init_time = rospy.Time.now()
        start = rospy.Time.now()
        # position where drone is originated in one of the wps
        wp = self.waypoints[self._current_wp]
        original = GlobalPositionTarget()
        original.header.seq = 1
        original.header.stamp = rospy.Time.now()
        original.header.frame_id = 'map'
        original.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        original.type_mask = 0b001111111000
        original.latitude = wp['lat']
        original.longitude = wp['long']
        original.altitude = wp['rel_alt']
        original.yaw = yaw_ned_to_enu(wp['yaw'])
        original.yaw_rate = 0.2
        rospy.loginfo("%s is scanning the first blade..." % self.namespace)
        first_blade = self.blade_inspect(original, [10.0, 0.0, 0.0], duration)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        rospy.loginfo("%s is scan second blade..." % self.namespace)
        second_blade = self.blade_inspect(original, [
            5.0 * np.cos(138. / 180.0 * np.pi), 0.000,
            14.27 * np.sin(138. / 180.0 * np.pi)
        ], duration)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        rospy.loginfo("%s is scan third blade..." % self.namespace)
        third_blade = self.blade_inspect(original, [
            5.0 * np.cos(225. / 180.0 * np.pi), 0.000,
            14.27 * np.sin(225. / 180.0 * np.pi)
        ], duration)
        self.calculate_batt_rate(battery, init_time, self.battery_rate_std)
        rospy.loginfo("%s has done the inspection..." % self.namespace)
        return np.min([first_blade, second_blade, third_blade])

    def blade_inspect(self,
                      original,
                      target_position,
                      duration=rospy.Duration(300, 0),
                      fly_return=True):
        """
        Inspecting the blade given the [original] pose to return to
        and end [target] position to scan
        """
        start = rospy.Time.now()
        reached_original = self.ACTION_SUCCESS
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
        target.yaw_rate = 0.2
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        reached_target = self.goto_coordinate(target,
                                              radius=self._radius,
                                              duration=duration)
        rospy.loginfo("%s is returning to %3.5f, %3.5f position..." %
                      (self.namespace, original.latitude, original.longitude))
        if reached_target == self.ACTION_SUCCESS and fly_return:
            original.header.seq = 1
            original.header.stamp = rospy.Time.now()
            original.header.frame_id = 'map'
            duration = duration - (rospy.Time.now() - start)
            start = rospy.Time.now()
            reached_original = self.goto_coordinate(original,
                                                    radius=self._radius,
                                                    duration=duration)
        return np.min([reached_target, reached_original])

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

    def _target_imu_cb(self, msg):
        """
        Target-to-follow imu callback
        """
        self.target_imu.append(msg)
        self.target_imu = self.target_imu[1:]

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
                      yaw_offset=0.,
                      desired_follower_alt=None,
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
        pose_sub = rospy.Subscriber('/%s/mavros/global_position/raw/unfix' %
                                    target,
                                    NavSatFix,
                                    self._target_global_pose_cb,
                                    queue_size=1)
        head_sub = rospy.Subscriber('/%s/mavros/global_position/compass_hdg' %
                                    target,
                                    Float64,
                                    self._target_heading_cb,
                                    queue_size=1)
        # Start following the target
        followed_duration = rospy.Duration(0, 0)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        while (rospy.Time.now() - start < duration) and not (
                rospy.is_shutdown()) and (not self.external_intervened) and (
                    followed_duration < follow_duration):
            if self.low_battery and not home:
                rospy.logwarn('%s battery is below minimum voltage!!!' %
                              self.namespace)
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
            # Due to yaw_ned_to_enu conversion, the sin and cos are flipped
            target.latitude = latitude + latitude_offset
            target.longitude = longitude + longitude_offset
            target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
            if (self._min_range >
                    -1) and (self._rangefinder[-1] - self._min_range <
                             self.MINIMUM_ALTITUDE):
                rospy.logerr("%s is %.3f meters away from impact!" %
                             (self.namespace,
                              (self._rangefinder[-1] - self._min_range)))
                target_alt = self.global_pose.altitude + 0.2
            elif desired_follower_alt is not None:
                target_alt = desired_follower_alt
            else:
                target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
                target_alt = altitude + offset[2]
            target.altitude = target_alt
            target.yaw = yaw_ned_to_enu(heading + (yaw_offset / 180.) * np.pi)
            target.yaw_rate = 0.2
            # Publish aimed position
            # rospy.loginfo(target)
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
            if abs(target_alt - self.global_pose.altitude
                   ) < 0.7 and np.linalg.norm(uav_pose - target_pose) < 6e-6:
                rospy.loginfo("%s has found target, following %d seconds" %
                              (self.namespace, followed_duration.secs))
                followed_duration += self._rate.sleep_dur
            else:
                followed_duration = rospy.Duration(0, 0)
                rospy.loginfo("Target is out of range, resetting the duration")
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
            rospy.get_param("~dist_initial_landing", -5.0),
            rospy.get_param("~altitude_initial_landing", 10.)
        ]
        if "behind" in rospy.get_param("~landing_approach", "side"):
            rospy.loginfo("Approaching ASV from behind ...")
        else:
            offset = [offset[1], offset[0], offset[2]]
            rospy.loginfo("Approaching ASV from side ...")
        self.follow_target('%s_launchpad' % self.namespace, True, offset, 0.,
                           None, rospy.Duration(3, 0), duration)
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
                               0., None, rospy.Duration(3, 0), duration)
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

    def deploy_irr(self, duration=rospy.Duration(600, 0)):
        """
        Deploying a crawler
        """
        battery = np.mean(self.battery_voltages)
        start = rospy.Time.now()
        if "simulation" in rospy.get_param("~scenario_type", "simulation"):
            response = self.simu_drm_irr(self.irr_name, duration=duration)
        else:
            response = self.real_deploy_irr(duration)
        self.calculate_batt_rate(battery, start, self.battery_rate_std)
        if response == self.ACTION_SUCCESS:
            self.irr_name = ''
        return response

    def real_deploy_irr(self, duration=rospy.Duration(600, 0)):
        """
        Deploying a crawler in real world
        """
        wp = self.waypoints[self._current_wp]
        deploy_sub = rospy.Subscriber('/%s/mission/status' % self.namespace,
                                      Int64,
                                      self._deploy_cb,
                                      queue_size=1)
        deploy_pub = rospy.Publisher('/%s/mission/status' % self.namespace,
                                     Int64,
                                     queue_size=3)
        start = rospy.Time.now()
        response = self.ACTION_FAIL
        rospy.loginfo("%s is deploying %s" % (self.namespace, self.irr_name))
        # we dont put duration here because of external system involve
        while not (rospy.is_shutdown()) and (not self.external_intervened):
            if self.deploy_msg.data == 0:
                for _ in range(3):
                    deploy_pub.publish(Int64(1))
                    self._rate.sleep()
            elif self.deploy_msg.data in [2, 3]:
                response = self.ACTION_SUCCESS if (
                    self.deploy_msg.data == 2) else self.ACTION_FAIL
                break
            self._rate.sleep()
        self.deploy_msg.data = 0
        # Unregister subscriptions
        deploy_sub.unregister()
        original = GlobalPositionTarget()
        original.header.seq = 1
        original.header.stamp = rospy.Time.now()
        original.header.frame_id = 'map'
        original.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        original.type_mask = 0b001111111000
        original.latitude = wp['lat']
        original.longitude = wp['long']
        original.altitude = wp['rel_alt']
        original.yaw = yaw_ned_to_enu(wp['yaw'])
        original.yaw_rate = 0.2
        rospy.loginfo(
            "Deploying is done, %s is going back to original wp ..." %
            self.namespace)
        duration = duration - (rospy.Time.now() - start)
        back_to_wp = self.goto_coordinate(original,
                                          radius=self._radius,
                                          duration=duration)
        # Prepare response
        if response == self.ACTION_SUCCESS:
            self.irr_name = ''
        return np.min([response, back_to_wp])

    def _deploy_cb(self, msg):
        """
        Deployment status callback
        """
        self.deploy_msg = msg

    def refuelling(self, duration=rospy.Duration(600, 0)):
        """
        Refuel
        """
        start = rospy.Time.now()
        while (rospy.Time.now() - start < duration) and not (
                rospy.is_shutdown()) and (not self.external_intervened) and (
                    np.mean(self.battery_voltages) < 99.0):
            rospy.loginfo("Battery is charging, it is currently at %.2f" %
                          (np.mean(self.battery_voltages)))
            self._rate.sleep()
        return self.ACTION_SUCCESS

    def simu_drm_irr(self, irr, deploy=True, duration=rospy.Duration(600, 0)):
        """
        Deploying a crawler in simulation
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
        original.latitude = wp['lat']
        original.longitude = wp['long']
        original.altitude = wp['rel_alt']
        original.yaw = yaw_ned_to_enu(wp['yaw'])
        original.yaw_rate = 0.2
        rospy.loginfo("%s is locating the blade..." % self.namespace)
        # locate blade
        response = self.blade_inspect(original, [8.0, 0.0, 0.0], duration,
                                      False)
        rospy.sleep(self._rate.sleep_dur * 70)
        heading = self.heading
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        # getting the highest blade point
        prev_blade = sorted(self.blade_pose, key=lambda x: x[2],
                            reverse=True)[0]
        # fly above the blade
        up_pose = [
            prev_blade[0] - self.local_pose.pose.position.x,
            prev_blade[1] - self.local_pose.pose.position.y + 0.7,
            prev_blade[2] - self.local_pose.pose.position.z + 3.
        ]
        rospy.loginfo("Blade is located, %s is flying up..." % self.namespace)
        fly_up = self.blade_inspect(original, up_pose, duration, False)
        rospy.sleep(self._rate.sleep_dur * 50)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        if deploy:
            rospy.loginfo("%s is preparing to drop %s" % (self.namespace, irr))
            # rotate right 90 degree while going down
            target = GlobalPositionTarget()
            target.header.seq = 1
            target.header.stamp = rospy.Time.now()
            target.header.frame_id = 'map'
            target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
            target.type_mask = 0b001111000111
            target.latitude = self.global_pose.latitude
            target.longitude = self.global_pose.longitude
            target.altitude = self.rel_alt - self._rangefinder[-1]
            target.velocity.z = -0.1
            target.yaw = yaw_ned_to_enu(heading + (np.pi * 93.5 / 180.))
            target.yaw_rate = 0.2
            reached_target = self.goto_coordinate(target,
                                                  radius=self._radius,
                                                  duration=duration)
        else:
            rospy.loginfo("%s is preparing to pick %s" % (self.namespace, irr))
            reached_target = self.follow_target(
                irr, False, [0., 0., self._min_range + 0.5], 0., None,
                rospy.Duration(3, 0), duration)
        for _ in range(3):
            self._lhm_pub.publish('%s,body,%s,base_link,%d' %
                                  (self.namespace, irr, int(not deploy)))
            self._rate.sleep()

        target = GlobalPositionTarget()
        target.header.seq = 1
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'map'
        target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        target.type_mask = 0b111111000111
        target.latitude = self.global_pose.latitude
        target.longitude = self.global_pose.longitude
        target.altitude = self.rel_alt + 3.
        target.velocity.z = 0.1
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        reached_target = self.goto_coordinate(target,
                                              radius=self._radius,
                                              duration=duration)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        rospy.loginfo("%s is flying back to waypoint ..." % self.namespace)
        reached_original = self.goto_coordinate(original,
                                                radius=self._radius,
                                                duration=duration)
        return np.min([response, fly_up, reached_target, reached_original])

    def retrieve_irr(self, irr, duration=rospy.Duration(600, 0)):
        """
        Retrieving a crawler
        """
        battery = np.mean(self.battery_voltages)
        init = rospy.Time.now()
        start = rospy.Time.now()
        if "simulation" in rospy.get_param("~scenario_type", "simulation"):
            while (self._irr_ready_to_be_picked == 0
                   ) and not (rospy.is_shutdown()) and (
                       rospy.Time.now() - start <
                       duration) and (not self.external_intervened):
                self._rate.sleep()
            duration = duration - (rospy.Time.now() - start)
            start = rospy.Time.now()
            if self._irr_ready_to_be_picked > 0:
                response = self.simu_drm_irr(irr, False, duration)
            else:
                response = self.ACTION_FAIL
            self._irr_ready_to_be_picked = 0
        else:
            response = self.real_retrieve_irr(irr, duration)
        self.calculate_batt_rate(battery, init, self.battery_rate_std)
        if response == self.ACTION_SUCCESS:
            self.irr_name = irr
        return response

    def real_retrieve_irr(self, irr, duration=rospy.Duration(600, 0)):
        """
        Retrieving a crawler in real world
        """
        start = rospy.Time.now()
        wp = self._current_wp
        rospy.loginfo("%s is approaching side of %s ..." %
                      (self.namespace, irr))
        maximum_distance = 5
        pickup_radius = 0.12
        approaching = self.follow_target(irr, False,
                                         [0., -1 * maximum_distance, 1.6], 0.,
                                         None, rospy.Duration(3, 0), duration)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        rospy.loginfo("%s is waiting for engagement ..." % self.namespace)
        while (self._irr_ready_to_be_picked == 0) and not (
                rospy.is_shutdown()) and (rospy.Time.now() - start < duration
                                          ) and (not self.external_intervened):
            self._rate.sleep()
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        if self._irr_ready_to_be_picked > 0:
            rospy.loginfo("%s is trying to engage with %s ..." %
                          (self.namespace, irr))
            engaging = self.irr_follow_engagement(irr, pickup_radius, duration)
            duration = duration - (rospy.Time.now() - start)
            start = rospy.Time.now()
        else:
            engaging = self.ACTION_FAIL
        self._irr_ready_to_be_picked = 0
        back_to_wp = self.goto(wp, duration, True, False)
        return np.min([approaching, engaging, back_to_wp])

    def irr_follow_engagement(self,
                              irr,
                              pickup_radius=0.5,
                              duration=rospy.Duration(600, 0)):
        """
        Follow irr and pickup
        """
        # Start and set to guide mode
        start = rospy.Time.now()
        self.guided_mode(duration=duration)
        # Collect pose and heading of the target
        self.target_heading = [0.0 for _ in range(len(self.target_heading))]
        self.target_global_pose = [
            NavSatFix() for _ in range(len(self.target_global_pose))
        ]
        self.target_imu = [Imu() for _ in range(len(self.target_imu))]
        pose_sub = rospy.Subscriber('/%s/mavros/global_position/raw/unfix' %
                                    irr,
                                    NavSatFix,
                                    self._target_global_pose_cb,
                                    queue_size=1)
        head_sub = rospy.Subscriber('/%s/mavros/global_position/compass_hdg' %
                                    irr,
                                    Float64,
                                    self._target_heading_cb,
                                    queue_size=1)
        imu_sub = rospy.Subscriber('/%s/mavros/imu/data' % irr,
                                   Imu,
                                   self._target_imu_cb,
                                   queue_size=1)
        # Start following the target
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        pickup_zone = False
        engaged = self.ACTION_FAIL
        pid = PID(10000., 1000., 0.0, 0.)
        pid.output_limits = (-0.4, 0.4)
        pid.sample_time = self._rate.sleep_dur.secs
        while (rospy.Time.now() - start < duration) and not (
                rospy.is_shutdown()) and (not self.external_intervened):
            if self.target_global_pose[-1] == NavSatFix():
                self._rate.sleep()
                continue
            rpy = tf.transformations.euler_from_quaternion([
                self.target_imu[-1].orientation.x,
                self.target_imu[-1].orientation.y,
                self.target_imu[-1].orientation.z,
                self.target_imu[-1].orientation.w
            ])
            # if the IRR is tilted
            roll_tilted = (np.abs(rpy[0]) > 0.28 and np.abs(rpy[0]) < 0.52)
            pitch_tilted = (np.abs(rpy[1]) > 0.28 and np.abs(rpy[1]) < 0.52)
            if roll_tilted or pitch_tilted:
                rospy.loginfo("%s is tilting due to engagment! Flying up..." %
                              irr)
                engaged = self.ACTION_SUCCESS
                break
            if hasattr(self, 'lhm') and self.lhm.status[2]:
                rospy.loginfo("%s is engaged! Flying up..." % self.namespace)
                engaged = self.ACTION_SUCCESS
                break
            latitude_offset, longitude_offset = xy_to_longlat(
                pickup_radius, 0, self.target_global_pose[-1].latitude)
            pick_up_zone = np.linalg.norm(
                np.array([latitude_offset, longitude_offset]))
            dist = np.linalg.norm(
                np.array([
                    self.target_global_pose[-1].latitude,
                    self.target_global_pose[-1].longitude
                ]) - np.array(
                    [self.global_pose.latitude, self.global_pose.longitude]))
            if dist < pick_up_zone:
                if not pickup_zone:
                    rospy.loginfo("%s is entering picking-up zone..." %
                                  self.namespace)
                pickup_zone = True
            if dist >= pick_up_zone and pickup_zone:
                if engaged == self.ACTION_FAIL:
                    rospy.loginfo(
                        "%s has no engagement detected! Flying up..." %
                        self.namespace)
                engaged = self.ACTION_SUCCESS
                break
            # pid for side movement
            heading = self.target_heading[-1]
            delta_lat = self.global_pose.latitude - self.target_global_pose[
                -1].latitude
            delta_long = self.global_pose.longitude - self.target_global_pose[
                -1].longitude
            x = pid(delta_long * np.cos(heading) - delta_lat * np.sin(heading))
            # Setup target position
            target = GlobalPositionTarget()
            target.header.seq = 1
            target.header.stamp = rospy.Time.now()
            target.header.frame_id = 'map'
            target.type_mask = 0b001111000111
            target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
            # target.velocity.x = x * np.cos(heading) + 0.3 * np.sin(heading)
            # target.velocity.y = -1 * x * np.sin(heading) + 0.3 * np.cos(
            #     heading)
            target.velocity.x = 0.3 * np.sin(heading)
            target.velocity.y = 0.3 * np.cos(heading)
            target.yaw = yaw_ned_to_enu(heading)
            target.yaw_rate = 0.2
            self._setpoint_pub.publish(target)
            self._rate.sleep()
        # Unregister subscriptions
        pose_sub.unregister()
        head_sub.unregister()
        imu_sub.unregister()
        # Flying up
        target = GlobalPositionTarget()
        target.header.seq = 1
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'map'
        target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        target.type_mask = 0b111111111000
        target.latitude = self.global_pose.latitude
        target.longitude = self.global_pose.longitude
        target.altitude = self.rel_alt + rospy.get_param(
            "~takeoff_altitude", 10)
        duration = duration - (rospy.Time.now() - start)
        start = rospy.Time.now()
        up = self.goto_coordinate(target,
                                  radius=self._radius,
                                  duration=duration)
        if hasattr(self, 'lhm') and self.lhm.status[2]:
            rospy.loginfo("%s is detected in the hook..." % irr)
            engaged = self.ACTION_SUCCESS
        if (rospy.Time.now() - start) > duration:
            engaged = self.OUT_OF_DURATION
        elif self.external_intervened:
            engaged = self.EXTERNAL_INTERVENTION
        return np.min([up, engaged])
