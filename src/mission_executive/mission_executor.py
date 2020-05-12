#!/usr/bin/env python

import argparse

import roslib
# ROS Packages
import rospy
# 3rd Party Packages
import yaml
from std_srvs.srv import Empty
from rosplan_dispatch_msgs.srv import DispatchService
from uav_executive.planner_interface import PlannerInterface as PIUAV
from asv_executive.planner_interface import PlannerInterface as PIASV


class MissionExec(object):
    def __init__(self, filename, configname, update_frequency=2.):
        self._rate = rospy.Rate(update_frequency)
        config = self.load_mission_config_file(filename, configname)
        # UAV planner
        if len(config['uavs']) and len(config['uav_waypoints']):
            self.uav_exec = PIUAV(config['uavs'], config['uav_waypoints'],
                                  config['asv_waypoints'])
        else:
            self.uav_exec = None
        # ASV planner
        if len(config['asvs']) and len(config['asv_waypoints']):
            uavs = list()
            if self.uav_exec is not None:
                uavs = self.uav_exec.uavs
            self.asv_exec = PIASV(config['asvs'], config['asv_waypoints'],
                                  uavs, config['uav_carrier'])
        else:
            self.asv_exec = None
        # Service proxies
        rospy.loginfo('Waiting for rosplan services...')
        rospy.wait_for_service(
            '/rosplan_problem_interface/problem_generation_server')
        self._problem_proxy = rospy.ServiceProxy(
            '/rosplan_problem_interface/problem_generation_server', Empty)
        rospy.wait_for_service('/rosplan_planner_interface/planning_server')
        self._planner_proxy = rospy.ServiceProxy(
            '/rosplan_planner_interface/planning_server', Empty)
        rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
        self._parser_proxy = rospy.ServiceProxy(
            '/rosplan_parsing_interface/parse_plan', Empty)
        rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
        self._dispatch_proxy = rospy.ServiceProxy(
            '/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
        self._cancel_plan_proxy = rospy.ServiceProxy(
            '/rosplan_plan_dispatcher/cancel_dispatch', Empty)
        # Service
        rospy.Service('%s/resume_plan' % rospy.get_name(), Empty,
                      self.resume_plan)
        # Auto call functions
        rospy.Timer(50 * self._rate.sleep_dur, self.low_battery_replan)

    def load_mission_config_file(self, filename, configname):
        """
        Load mission configuration from file
        located at the package under the config folder
        """
        pkg_path = roslib.packages.get_pkg_dir('mimree_executive')
        configs = yaml.load(open(pkg_path + '/config/%s' % filename, 'r'))
        config = [i for i in configs if i['config'] == configname][0]
        return config

    def resume_plan(self, req):
        """
        Function to flag down external intervention, and replan
        """
        if self.uav_exec is not None:
            self.uav_exec.resume_plan()
        if self.asv_exec is not None:
            self.asv_exec.resume_plan()
        rospy.Timer(self._rate.sleep_dur, self.execute, oneshot=True)
        return list()

    def inspection_mission(self, full=False):
        """
        Inspection mission
        """
        if self.uav_exec is not None:
            self.uav_exec.inspection_mission()
            if self.asv_exec is not None and not full:
                self.asv_exec.deploy_retrieve_mission()
            elif self.asv_exec is not None and full:
                self.asv_exec.inspection_mission()
            self.execute()
        if self.asv_exec is not None and self.uav_exec is None:
            self.asv_exec.inspection_mission()
            self.execute()

    def low_battery_replan(self, event=True):
        """
        Assets return home due to low battery voltage
        """
        replan = False
        if self.uav_exec is not None and True in self.uav_exec.lowbat.values():
            self.uav_exec.low_battery_return_mission(all_return=False)
            self.uav_exec.low_battery_mission = True
            replan = True
        if self.asv_exec is not None and True in self.asv_exec.lowfuel.values(
        ):
            if self.uav_exec is not None and self.asv_exec.low_fuel[
                    self.asv_exec.uav_carrier]:
                self.uav_exec.low_battery_return_mission(all_return=True)
            self.asv_exec.low_fuel_return_mission()
            self.asv_exec.low_fuel_mission = True
            replan = True
        if replan:
            achieved = self.execute()
            if self.uav_exec is not None:
                self.uav_exec.low_battery_mission = not achieved
            if self.asv_exec is not None:
                self.asv_exec.low_fuel_mission = not achieved

    def execute(self, event=True):
        """
        Execute plan using ROSPlan
        """
        rospy.loginfo('Generating mission plan ...')
        self._problem_proxy()
        self._rate.sleep()
        rospy.loginfo('Planning ...')
        self._planner_proxy()
        self._rate.sleep()
        rospy.loginfo('Execute mission plan ...')
        self._parser_proxy()
        self._rate.sleep()
        response = self._dispatch_proxy()
        if response.goal_achieved:
            rospy.loginfo('Mission Succeed')
        else:
            rospy.loginfo('Mission Failed')
        return response.goal_achieved


if __name__ == '__main__':
    rospy.init_node('mission_executive')
    parser_arg = argparse.ArgumentParser(prog=rospy.get_name())
    parser_arg.add_argument("-f",
                            dest='config_file',
                            default='wp_config_simulation.yaml',
                            help='waypoint\'s file name')
    parser_arg.add_argument('-m',
                            dest='config_mission',
                            default='mangalia',
                            help='Mission configuration')
    args = parser_arg.parse_args()
    mission_exec = MissionExec(args.config_file, args.config_mission)
    mission_exec.inspection_mission()
    rospy.spin()
