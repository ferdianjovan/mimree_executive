#!/usr/bin/env python

import argparse
import sys

import roslib
# ROS Packages
import rospy
# 3rd Party Packages
import yaml
from asv_executive.planner_interface import PlannerInterface as PIASV
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse
from rosplan_knowledge_msgs.msg import ExprBase, KnowledgeItem
from rosplan_knowledge_msgs.srv import (KnowledgeUpdateService,
                                        KnowledgeUpdateServiceRequest)
from rosplan_dispatch_msgs.msg import ActionFeedback
from std_srvs.srv import Empty
from uav_executive.planner_interface import PlannerInterface as PIUAV
from irr_executive.planner_interface import PlannerInterface as PIIRR


class MissionExec(object):
    def __init__(self, filename, configname, update_frequency=2.):
        self.current_mission = {'uav': list(), 'asv': list(), 'irr': list()}
        self._rate = rospy.Rate(update_frequency)
        config = self.load_mission_config_file(filename, configname)
        self.config = config
        self.metric_optimization = self._get_metric_optimisation()
        # ASV planner
        if len(config['asvs']) and len(config['asv_waypoints']):
            self.asv_exec = PIASV(config['asvs'], config['asv_waypoints'],
                                  config['asv_waypoint_connection'],
                                  config['asv_waypoint_to_wt'])
        else:
            self.asv_exec = None
        # UAV planner
        if len(config['uavs']) and len(config['uav_waypoints']):
            self.uav_exec = PIUAV(config['uavs'], config['uav_waypoints'],
                                  config['uav_waypoint_connection'],
                                  config['uav_waypoint_to_wt'])
        else:
            self.uav_exec = None
        if len(config['irrs']) and len(config['irr_waypoints']):
            self.irr_exec = PIIRR(config['irrs'], config['irr_waypoints'],
                                  config['irr_waypoint_connection'],
                                  config['irr_waypoint_to_wt'])
        else:
            self.irr_exec = None
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
        self._knowledge_update_proxy = rospy.ServiceProxy(
            '/rosplan_knowledge_base/update', KnowledgeUpdateService)
        # Service
        rospy.Service('%s/resume_plan' % rospy.get_name(), Empty,
                      self.resume_plan)
        rospy.Service('%s/return_home' % rospy.get_name(), Empty,
                      self.return_home_mission)
        rospy.Service('%s/cancel_plan' % rospy.get_name(), Empty,
                      self.cancel_plan)
        # Subscriber
        rospy.Subscriber('/rosplan_plan_dispatcher/action_feedback',
                         ActionFeedback,
                         self._feedback_cb,
                         queue_size=2)
        # Auto call functions
        rospy.Timer(50 * self._rate.sleep_dur, self.low_battery_replan)

    def update_weight(self):
        """
        Updating the normal distribution for duration and fuel for each robot
        """
        # update uav
        for idx, conf_uav in enumerate(self.config['uavs']):
            for uav in self.uav_exec.uavs:
                if uav.namespace == conf_uav['name']:
                    self.config['uavs'][idx]['fuel_rate'] = [
                        uav.battery_rate_mean, uav.battery_rate_std
                    ]
                    break
        for idx, conf_conn in enumerate(
                self.config['uav_waypoint_connection']):
            for conn in self.uav_exec.connections:
                if set(conf_conn[:2]) == set(conn[:2]):
                    self.config['uav_waypoint_connection'][idx] = conn
                    break
        # update asv
        for idx, conf_asv in enumerate(self.config['asvs']):
            for asv in self.asv_exec.asvs:
                if asv.namespace == conf_asv['name']:
                    self.config['asvs'][idx]['fuel_rate'] = [
                        asv.fuel_rate_mean, asv.fuel_rate_std
                    ]
                    break
        for idx, conf_conn in enumerate(
                self.config['asv_waypoint_connection']):
            for conn in self.asv_exec.connections:
                if set(conf_conn[:2]) == set(conn[:2]):
                    self.config['asv_waypoint_connection'][idx] = conn
                    break
        # update irr
        for idx, conf_irr in enumerate(self.config['irrs']):
            for irr in self.irr_exec.irrs:
                if irr.namespace == conf_irr['name']:
                    self.config['irrs'][idx]['fuel_rate'] = [
                        irr.fuel_rate_mean, irr.fuel_rate_std
                    ]
                    break
        for idx, conf_conn in enumerate(
                self.config['irr_waypoint_connection']):
            for conn in self.irr_exec.connections:
                if set(conf_conn[:2]) == set(conn[:2]):
                    self.config['irr_waypoint_connection'][idx] = conn
                    break
        pkg_path = roslib.packages.get_pkg_dir('mimree_executive')
        yaml.dump(self.config,
                  open(pkg_path + '/config/learned_config.yaml', 'w'))

    def _feedback_cb(self, msg):
        """
        Monitor action feedback
        """
        if msg.status == 'action out of duration':
            self.cancel_plan(Empty())
            rospy.sleep(self._rate.sleep_dur * 30)
            self.resume_plan(Empty())

    def _get_metric_optimisation(self):
        """
        Optimising generated plan to focus on minimizing total time
        """
        request = KnowledgeUpdateServiceRequest()
        request.update_type = KnowledgeUpdateServiceRequest.ADD_METRIC
        request.knowledge.knowledge_type = KnowledgeItem.EXPRESSION
        request.knowledge.optimization = "minimize"
        request.knowledge.expr.tokens.append(
            ExprBase(expr_type=ExprBase.SPECIAL,
                     special_type=ExprBase.TOTAL_TIME))
        return request

    def load_mission_config_file(self, filename, configname):
        """
        Load mission configuration from file
        located at the package under the config folder
        """
        pkg_path = roslib.packages.get_pkg_dir('mimree_executive')
        configs = yaml.load(open(pkg_path + '/config/%s' % filename, 'r'))
        config = [i for i in configs if i['config'] == configname][0]
        return config

    def add_mission(self, mission):
        """
        Adding missions based on the type of vehicle executing them
        """
        mission = mission.replace("[", "").replace("]", "").replace(" ", "")
        mission = mission.split(",")
        self.current_mission = {
            'uav': [
                val for idx, val in enumerate(mission[:-1])
                if idx % 2 == 0 and mission[idx + 1] == 'uav'
            ],
            'asv': [
                val for idx, val in enumerate(mission[:-1])
                if idx % 2 == 0 and mission[idx + 1] == 'asv'
            ],
            'irr': [
                val for idx, val in enumerate(mission[:-1])
                if idx % 2 == 0 and mission[idx + 1] == 'irr'
            ]
        }
        succeed = True
        if self.uav_exec is not None:
            succeed = succeed and self.uav_exec.add_mission(
                self.current_mission['uav'], self.current_mission['irr'])
        if self.asv_exec is not None:
            succeed = succeed and self.asv_exec.add_mission(
                self.current_mission['asv'])
        if self.irr_exec is not None:
            succeed = succeed and self.irr_exec.add_mission(
                self.current_mission['irr'])
        return succeed

    def launch(self, event=True):
        """
        Execute plan using ROSPlan
        """
        self._knowledge_update_proxy(self.metric_optimization)
        if self.uav_exec is not None:
            self.uav_exec.resume_plan()
        if self.asv_exec is not None:
            self.asv_exec.resume_plan()
        if self.irr_exec is not None:
            self.irr_exec.resume_plan()
        response = DispatchServiceResponse()
        for _ in range(3):
            try:
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
                break
            except rospy.service.ServiceException:
                continue
            self._rate.sleep()
        if response.goal_achieved:
            rospy.loginfo('Mission Succeed')
        else:
            rospy.loginfo('Mission Failed')
        return response.goal_achieved

    def resume_plan(self, req):
        """
        Function to flag down external intervention, and replan
        """
        if self.uav_exec is not None:
            self.uav_exec.resume_plan()
        if self.asv_exec is not None:
            self.asv_exec.resume_plan()
        if self.irr_exec is not None:
            self.irr_exec.resume_plan()
        rospy.Timer(self._rate.sleep_dur, self.launch, oneshot=True)
        return list()

    def cancel_plan(self, req):
        """
        Function to cancel all plan across assets
        """
        self._cancel_plan_proxy()
        rospy.loginfo("Cancelling the mission as requested...")
        if self.uav_exec is not None:
            self.uav_exec.cancel_plan()
        if self.asv_exec is not None:
            self.asv_exec.cancel_plan()
        if self.irr_exec is not None:
            self.irr_exec.cancel_plan()
        return list()

    def return_home_mission(self, req):
        """
        Mission to return home
        """
        self.current_mission = {
            'uav': ['home'],
            'asv': ['home'],
            'irr': ['home']
        }
        if self.uav_exec is not None:
            self.uav_exec.clear_mission()
            self.uav_exec.home_mission()
        if self.asv_exec is not None:
            self.asv_exec.clear_mission()
            self.asv_exec.home_mission()
        if self.irr_exec is not None:
            self.irr_exec.clear_mission()
            self.irr_exec.home_mission()
        self.cancel_plan(req)
        rospy.sleep(4 * self._rate.sleep_dur)
        self.launch()
        return list()

    def low_battery_replan(self, event=True):
        """
        Assets return home due to low battery voltage
        """
        replan = False
        if self.irr_exec is not None:
            if {True, False} == set(self.irr_exec.lowfuel.values()):
                replan = True
            elif True in self.irr_exec.lowfuel.values():
                self.irr_exec.clear_mission()
                self.irr_exec.home_mission()
        if (self.uav_exec is not None):
            if {True, False} == set(self.uav_exec.lowbat.values()):
                replan = True
            elif True in self.uav_exec.lowbat.values():
                self.uav_exec.clear_mission()
                self.uav_exec.home_mission()
                if self.irr_exec is not None:
                    self.irr_exec.clear_mission()
                    self.irr_exec.home_mission()
                replan = True
        if self.asv_exec is not None:
            if {True, False} == set(self.asv_exec.lowfuel.values()):
                replan = True
            elif True in self.asv_exec.lowfuel.values():
                self.asv_exec.clear_mission()
                self.asv_exec.home_mission()
                if self.uav_exec is not None:
                    self.uav_exec.clear_mission()
                    self.uav_exec.home_mission()
                if self.irr_exec is not None:
                    self.irr_exec.clear_mission()
                    self.irr_exec.home_mission()
                replan = True
        if replan:
            rospy.logwarn("Battery / Fuel of one of assets is low!")
            self.cancel_plan(event)
            rospy.sleep(4 * self._rate.sleep_dur)
            self.launch()


if __name__ == '__main__':
    rospy.init_node('mission_executive')
    parser_arg = argparse.ArgumentParser(prog=rospy.get_name())
    parser_arg.add_argument("-f",
                            dest='config_file',
                            default='wp_config_simulation.yaml',
                            help='waypoint\'s file name')
    parser_arg.add_argument('-c',
                            dest='config_mission',
                            default='mangalia',
                            help='Mission configuration')
    parser_arg.add_argument('-m',
                            dest='mission',
                            default='[WT1,irr,WT2,asv,WT3,uav,WT4,uav]',
                            help="format: [WT[1|2|3|4],[irr|asv|uav]]")
    args = parser_arg.parse_args(sys.argv[1:7])
    mission_exec = MissionExec(args.config_file, args.config_mission)
    if mission_exec.add_mission(args.mission):
        mission_exec.launch()
        mission_exec.update_weight()
    rospy.spin()
