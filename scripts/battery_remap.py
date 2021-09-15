#!/usr/bin/env python

import sys
import argparse
import numpy as np
from copy import deepcopy
from threading import Lock

import rospy
from rosplan_dispatch_msgs.msg import ActionDispatch
from sensor_msgs.msg import BatteryState


class BatteryRemap(object):

    mutex = Lock()
    MAX_VOLTAGE = 12.5

    def __init__(self, name, multiplier=0.33):
        self._delta = [0.0 for i in range(30)]
        self.vehicle_name = name
        self.multiplier = multiplier
        self.battery = BatteryState()
        self.modified_batt = BatteryState()
        self.batt_pub = rospy.Publisher('/%s/mavros/modified_battery' % name,
                                        BatteryState,
                                        queue_size=3)
        rospy.Subscriber('/%s/mavros/battery' % name,
                         BatteryState,
                         self.batt_cb,
                         queue_size=1)
        rospy.Subscriber('/rosplan_plan_dispatcher/action_dispatch',
                         ActionDispatch,
                         self.dispatch_cb,
                         queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.battery_update)

    def battery_update(self, event):
        """
        Publish modified battery in x interval
        """
        self.batt_pub.publish(self.modified_batt)

    def batt_cb(self, msg):
        """
        Battery callback specific for one vehicle
        """
        self.mutex.acquire()
        if self.battery == BatteryState():
            self.battery = deepcopy(msg)
            self.modified_batt = deepcopy(msg)
        elif msg.percentage > 0.0:
            idx = msg.header.seq % len(self._delta)
            self._delta[idx] = (msg.percentage - self.battery.percentage)
            new_perc = self.modified_batt.percentage + (self._delta[idx] *
                                                        self.multiplier)
            self.battery = deepcopy(msg)
            self.modified_batt = deepcopy(msg)
            self.modified_batt.percentage = new_perc
        elif msg.voltage <= self.MAX_VOLTAGE:
            delta = np.min([
                -0.0001,
                np.random.normal(loc=np.mean(self._delta),
                                 scale=np.std(self._delta))
            ])
            new_perc = self.modified_batt.percentage + (delta *
                                                        self.multiplier)
            self.modified_batt = deepcopy(msg)
            self.modified_batt.percentage = new_perc
        self.mutex.release()

    def dispatch_cb(self, msg):
        """
        Recharge battery when refuelling action is dispatched
        """
        self.mutex.acquire()
        if msg.name in ['uav_refuelling', 'uav_refuelling_home']:
            while self.modified_batt.percentage < 1.:
                self.modified_batt.percentage += 0.05
                rospy.sleep(0.1)
        self.mutex.release()


if __name__ == '__main__':
    rospy.init_node("battery_remap")
    parser_arg = argparse.ArgumentParser(prog=rospy.get_name())
    parser_arg.add_argument("-n",
                            dest='name',
                            default='hector',
                            help='Vehicle Name')
    args = parser_arg.parse_args(sys.argv[1:3])
    battery_remap = BatteryRemap(args.name,
                                 rospy.get_param("~batt_multiplier", .33))
    rospy.spin()
