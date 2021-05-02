#!/usr/bin/env python

# Other 3rd party packages
import numpy as np
# ROS packages
import rospy
# MAVROS packages
from mavros_msgs.srv import CommandHome
from sensor_msgs.msg import BatteryState, NavSatFix
from mavros_msgs.msg import HomePosition
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from irr_executive.olam_executor import ActionExecutor as OLAM


class ActionExecutor(object):

    INIT_FUEL = 100.0
    MINIMUM_FUEL = 25.0
    EXTERNAL_INTERVENTION = -2
    OUT_OF_DURATION = -1
    ACTION_SUCCESS = 1
    ACTION_FAIL = 0

    def __init__(self, namespace, waypoints, update_frequency=10.):
        """
        A Class that interfaces with MAVROS and Bladebug Controller for executing actions
        """
        self.namespace = namespace['name']
        self.fuel_rate_mean = 1.0
        self.fuel_rate_std = 1.0
        self.low_fuel = False
        self.fuel = self.INIT_FUEL
        self.set_battery(namespace['max_fuel'], namespace['min_fuel'],
                         namespace['fuel_rate'])
        self._cancel_action = False
        self.external_intervened = False
        self.waypoints = [None]
        self.home = HomePosition()
        self.global_pose = NavSatFix()
        self.heading = 0.0
        self._current_wp = -1
        self._radius = 1e-05
        self._rate = rospy.Rate(update_frequency)
        # Subscribers
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
        self._setpoint_pub = rospy.Publisher('/joy', Joy, queue_size=3)
        # Service proxies
        rospy.loginfo('Waiting for /%s/mavros/cmd/set_home ...' %
                      self.namespace)
        rospy.wait_for_service('/%s/mavros/cmd/set_home' % self.namespace)
        self._set_home_proxy = rospy.ServiceProxy(
            '/%s/mavros/cmd/set_home' % self.namespace, CommandHome)
        self.set_current_location_as_home()
        # Adding initial waypoints' configuration
        while self.waypoints[0] is None:
            self._rate.sleep()
        self.waypoints = self.waypoints + waypoints
        if "simulation" in rospy.get_param("~scenario_type", "simulation"):
            self.embrace_pose()
        else:
            self.olam = OLAM(self.namespace, update_frequency)
        # Auto call functions
        rospy.Timer(10 * self._rate.sleep_dur, self.intervene_observer)

    def set_battery(self, init_batt, min_batt, fuel_rate):
        """
        Setting initial fuel and minimum fuel condition
        """
        rospy.loginfo('%s is setting up battery requirements...' %
                      self.namespace)
        self.INIT_FUEL = init_batt
        self.MINIMUM_FUEL = min_batt
        self.fuel = self.INIT_FUEL
        self.fuel_rate_mean, self.fuel_rate_std = fuel_rate
        self.low_fuel = False

    def _heading_cb(self, msg):
        """
        IRR heading callback
        """
        self.heading = msg.data / 180. * np.pi

    def _home_cb(self, msg):
        """
        Home position callback
        """
        self.home = msg
        self.waypoints[0] = {
            'yaw': 0.0,
            'lat': msg.geo.latitude,
            'long': msg.geo.longitude,
            'repair': False,
            'inspect': False
        }

    def _global_pose_cb(self, msg):
        """
        IRR global position callback
        """
        self.global_pose = msg

    def _battery_cb(self, msg):
        """
        IRR Battery state callback
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

    def intervene_observer(self, event):
        """
        Watch whether human operator intervenes
        """
        self.external_intervened = self._cancel_action

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

    def embrace_pose(self):
        """
        Stand-up pose
        """
        buttons = Joy()
        buttons.header.stamp = rospy.Time.now()
        buttons.header.frame_id = 'map'
        buttons.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        buttons.buttons = [0, 0, 0, 1, 0, 0, 0, 0, 0]
        for _ in range(3):
            self._setpoint_pub.publish(buttons)
            self._rate.sleep()
        rospy.sleep(3)

    def rotate(self, degree=180, duration=rospy.Duration(600, 0)):
        """
        Rotating a crawler for [degree] degree
        """
        fuel = self.fuel
        start = rospy.Time.now()
        if "simulation" in rospy.get_param("~scenario_type", "simulation"):
            response = self.simulated_rotation(degree, duration)
        else:
            response = self.real_navigation(duration)
        if response == self.ACTION_SUCCESS:
            self.calculate_fuel_rate(fuel, start, self.fuel_rate_std)
        return response

    def simulated_rotation(self, degree, duration=rospy.Duration(60, 0)):
        """
        Rotate for [degree] degree
        """
        response = self.ACTION_FAIL
        start = rospy.Time.now()
        heading = self.heading
        offset = degree / 180. * np.pi
        buttons = Joy()
        buttons.header.stamp = rospy.Time.now()
        buttons.header.frame_id = 'map'
        buttons.axes = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        buttons.buttons = [0, 0, 1, 0, 0, 0, 0, 0, 0]
        for _ in range(3):
            self._setpoint_pub.publish(buttons)
            self._rate.sleep()
        while (rospy.Time.now() - start < duration) and (
                not rospy.is_shutdown()) and (not self.external_intervened):
            current_heading = self.heading
            if abs(current_heading - heading) > offset:
                self.embrace_pose()
                response = self.ACTION_SUCCESS
                break
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        if self.external_intervened:
            response = self.EXTERNAL_INTERVENTION
        return response

    def prepare_for_retrieval(self, duration=rospy.Duration(600, 0)):
        """
        Prepare for engagement by the drone
        """
        response = self.ACTION_FAIL
        if hasattr(self, 'olam'):
            response = self.olam.engagement_position(duration)
        elif "simulation" in rospy.get_param("~scenario_type", "simulation"):
            self.embrace_pose()
            response = self.ACTION_SUCCESS
        return response

    def navigate(self, duration=rospy.Duration(600, 0)):
        """
        Retrieving a crawler
        """
        fuel = self.fuel
        start = rospy.Time.now()
        if "simulation" in rospy.get_param("~scenario_type", "simulation"):
            response = self.simulated_navigation(duration)
        else:
            response = self.real_navigation(duration)
        if response == self.ACTION_SUCCESS:
            self.calculate_fuel_rate(fuel, start, self.fuel_rate_std)
        return response

    def real_navigation(self, duration=rospy.Duration(600, 0)):
        """
        Retrieving a crawler in real world
        """
        rospy.sleep(3.0)
        return self.ACTION_SUCCESS

    def simulated_navigation(self, duration=rospy.Duration(60, 0)):
        """
        Walk for 1 meter
        """
        response = self.ACTION_FAIL
        start = rospy.Time.now()
        self.set_current_location_as_home()
        buttons = Joy()
        buttons.header.stamp = rospy.Time.now()
        buttons.header.frame_id = 'map'
        buttons.axes = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        buttons.buttons = [0, 0, 1, 0, 0, 0, 0, 0, 0]
        for _ in range(3):
            self._setpoint_pub.publish(buttons)
            self._rate.sleep()
        home_pos = np.array([self.home.geo.latitude, self.home.geo.longitude])
        while (rospy.Time.now() - start < duration) and (
                not rospy.is_shutdown()) and (not self.external_intervened):
            cur_pos = np.array(
                [self.global_pose.latitude, self.global_pose.longitude])
            if np.linalg.norm(cur_pos - home_pos) > self._radius:
                self.embrace_pose()
                response = self.ACTION_SUCCESS
                break
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        if self.external_intervened:
            response = self.EXTERNAL_INTERVENTION
        return response
