#!/usr/bin/env python

# 3rd Party Packages
import re
import numpy as np
from threading import Lock

# ROS Packages
import rospy
from asv_executive.executor import ActionExecutor
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv import (GetAttributeService,
                                        GetDomainOperatorDetailsService,
                                        GetDomainPredicateDetailsService,
                                        KnowledgeUpdateService,
                                        KnowledgeUpdateServiceRequest)
from scipy.stats import norm


class PlannerInterface(object):

    mutex = Lock()

    def __init__(self,
                 names,
                 asv_waypoints,
                 connections,
                 wt_to_wp,
                 update_frequency=10.):
        """
        A Class that interfaces with ROSPlan for dispatching asv actions and
        updating asv knowledge during a mission
        """
        self.action_sequence = 0
        self.wt_to_wp = wt_to_wp
        self.connections = connections
        self.dispatch_actions = list()
        self.total_wp = len(asv_waypoints)
        self.goal_state = [list(), list()]
        self._rate = rospy.Rate(update_frequency)
        self.lowfuel = {name['name']: False for name in names}
        self.asvs = [ActionExecutor(name, asv_waypoints) for name in names]

        # Rosplan Service proxies
        rospy.loginfo('Waiting for service /rosplan_knowledge_base/update ...')
        rospy.wait_for_service('/rosplan_knowledge_base/update')
        self._knowledge_update_proxy = rospy.ServiceProxy(
            '/rosplan_knowledge_base/update', KnowledgeUpdateService)
        rospy.loginfo(
            'Waiting for /rosplan_knowledge_base/domain/predicate_details ...')
        rospy.wait_for_service(
            '/rosplan_knowledge_base/domain/predicate_details')
        self._predicate_proxy = rospy.ServiceProxy(
            '/rosplan_knowledge_base/domain/predicate_details',
            GetDomainPredicateDetailsService)
        rospy.wait_for_service(
            '/rosplan_knowledge_base/domain/operator_details')
        self._operator_proxy = rospy.ServiceProxy(
            '/rosplan_knowledge_base/domain/operator_details',
            GetDomainOperatorDetailsService)
        rospy.loginfo(
            'Waiting for /rosplan_knowledge_base/state/propositions ...')
        rospy.wait_for_service('/rosplan_knowledge_base/state/propositions')
        self._proposition_proxy = rospy.ServiceProxy(
            '/rosplan_knowledge_base/state/propositions', GetAttributeService)

        # Subscribers and Publishers
        rospy.Subscriber('/rosplan_plan_dispatcher/action_dispatch',
                         ActionDispatch,
                         self._dispatch_cb,
                         queue_size=10)
        rospy.Subscriber('/rosplan_plan_dispatcher/action_feedback',
                         ActionFeedback,
                         self._feedback_cb,
                         queue_size=10)
        self._feedback_publisher = rospy.Publisher(
            '/rosplan_plan_dispatcher/action_feedback',
            ActionFeedback,
            queue_size=10)

        if not self.set_instances():
            rospy.logerr('ASV instances in PDDL can\'t be set!')
        if not self.set_init(names, asv_waypoints, connections):
            rospy.logerr('ASV initial state can\'t be set!')
        # Auto call functions
        rospy.Timer(self._rate.sleep_dur, self._wp_update)
        rospy.Timer(10 * self._rate.sleep_dur, self._fuel_update)
        rospy.Timer(self._rate.sleep_dur, self._execute_action)
        rospy.sleep(20 * self._rate.sleep_dur)

    def _feedback_cb(self, msg):
        """
        Function to make sure this executes actions in the right order
        """
        self.action_sequence = msg.action_id + 1 if (
            msg.status == 'action achieved') else self.action_sequence

    def _apply_operator_effect(self, op_name, dispatch_params):
        """
        Add / remove knowledge based on operator effect and parameters
        from dispatched action
        """
        predicate_names = list()
        parameters = list()
        update_types = list()
        response = self._operator_proxy(op_name)
        predicates = (response.op.at_start_add_effects +
                      response.op.at_end_add_effects)
        for predicate in predicates:
            predicate_names.append(predicate.name)
            params = list()
            for typed_param in predicate.typed_parameters:
                for param in dispatch_params:
                    if typed_param.key == param.key:
                        params.append(param)
                        break
            parameters.append(params)
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        predicates = (response.op.at_start_del_effects +
                      response.op.at_end_del_effects)
        for predicate in predicates:
            predicate_names.append(predicate.name)
            params = list()
            for typed_param in predicate.typed_parameters:
                for param in dispatch_params:
                    if typed_param.key == param.key:
                        params.append(param)
                        break
            parameters.append(params)
            update_types.append(KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE)
        succeed = self.update_predicates(predicate_names, parameters,
                                         update_types)
        return succeed

    def _action(self, action_dispatch, action_func, action_params=list()):
        """
        Template uav action to respond to the dispatched action
        """
        self.publish_feedback(action_dispatch.action_id, 'action enabled')
        start_time = rospy.Time(action_dispatch.dispatch_time)
        duration = rospy.Duration(action_dispatch.duration)
        self._rate.sleep()
        rospy.loginfo('Dispatching %s action at %s with duration %s ...' %
                      (action_dispatch.name, str(
                          start_time.secs), str(duration.to_sec())))
        action_response = action_func(*action_params)
        if action_response == ActionExecutor.ACTION_SUCCESS:
            if self._apply_operator_effect(action_dispatch.name,
                                           action_dispatch.parameters):
                self.publish_feedback(action_dispatch.action_id,
                                      'action achieved')
            else:
                self.publish_feedback(action_dispatch.action_id,
                                      'action failed')
        elif action_response == ActionExecutor.OUT_OF_DURATION:
            rospy.logwarn("Action %s took longer than the allocated duration" %
                          action_dispatch.name)
            self.publish_feedback(action_dispatch.action_id,
                                  'action out of duration')
        elif action_response == ActionExecutor.EXTERNAL_INTERVENTION:
            rospy.logwarn(
                "External intervention is detected, cancelling mission!")
            self.publish_feedback(action_dispatch.action_id, 'action failed')
        else:
            self.publish_feedback(action_dispatch.action_id, 'action failed')
        return True

    def _fuel_update(self, event):
        """
        Update fuel value on ROSPlan knowledge base
        """
        for asv in self.asvs:
            # fuel status update
            self.update_functions(
                ['fuel'], [[KeyValue('v', asv.namespace)]], [asv.fuel], [
                    KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE,
                ])
            self.lowfuel[asv.namespace] = asv.low_fuel

    def _wp_update(self, event):
        """
        Add or remove waypoint facts on ROSPlan knowledge base
        """
        wp_asv = list()
        params = list()
        pred_names = list()
        update_types = list()
        attributes = self._proposition_proxy('at').attributes
        for attribute in attributes:
            name = attribute.values[0].value
            asv = [i for i in self.asvs if i.namespace == name]
            if len(asv):
                wp_asv.append(name)
                at = int(re.findall(r'\d+', attribute.values[1].value)[0])
                if asv[0]._current_wp != -1 and at != asv[0]._current_wp:
                    # add current wp that asv resides
                    pred_names.append('at')
                    params.append([
                        KeyValue('v', asv[0].namespace),
                        KeyValue('wp', 'asv_wp%d' % asv[0]._current_wp)
                    ])
                    update_types.append(
                        KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
                    # remove previous wp that asv resided
                    pred_names.append('at')
                    params.append([
                        KeyValue('v', asv[0].namespace),
                        KeyValue('wp', 'asv_wp%d' % at)
                    ])
                    update_types.append(
                        KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE)
        for asv in self.asvs:
            if not (asv.namespace in wp_asv) and asv._current_wp != -1:
                # add current wp that asv resides
                pred_names.append('at')
                params.append([
                    KeyValue('v', asv.namespace),
                    KeyValue('wp', 'asv_wp%d' % asv._current_wp)
                ])
                update_types.append(
                    KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        if pred_names != list():
            self.update_predicates(pred_names, params, update_types)

    def publish_feedback(self, action_id, fbstatus):
        """
        Function to publish action feedback to action_feedback topic
        """
        feedback = ActionFeedback()
        feedback.action_id = action_id
        feedback.status = fbstatus
        self._feedback_publisher.publish(feedback)

    def set_instances(self):
        """
        Set initial instances for ASVs to ROSPlan
        """
        ins_types = list()
        ins_names = list()
        update_types = list()
        for idx in range(self.total_wp + 1):
            ins_types.append('waypoint')
            ins_names.append('asv_wp' + str(idx))
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        for asv in self.asvs:
            ins_types.append('asv')
            ins_names.append(asv.namespace)
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        return self.update_instances(ins_types, ins_names, update_types)

    def set_init(self, asvs, waypoints, connections):
        """
        Adding facts to the initial state
        """
        succeed = True
        connections.extend([[j, i, k, l] for i, j, k, l in connections
                            if i != j])
        for i in connections:
            # add wp connection
            succeed = succeed and self.update_predicates(['connected'], [[
                KeyValue('wp1', 'asv_wp' + str(i[0])),
                KeyValue('wp2', 'asv_wp' + str(i[1]))
            ]], [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            # add min action duration on the wp connection
            succeed = succeed and self.update_functions(['min_dur'], [[
                KeyValue('wp1', 'asv_wp' + str(i[0])),
                KeyValue('wp2', 'asv_wp' + str(i[1]))
            ]], [float(i[2])], [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            # add max action duration on the wp connection
            succeed = succeed and self.update_functions(['max_dur'], [[
                KeyValue('wp1', 'asv_wp' + str(i[0])),
                KeyValue('wp2', 'asv_wp' + str(i[1]))
            ]], [norm.ppf(0.95, loc=float(i[2]), scale=float(i[3]))
                 ], [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
        # add ASV waypoints for UAV to take-off
        for idx, val in enumerate(waypoints):
            if val['takeoff']:
                succeed = succeed and self.update_predicates(
                    ['takeoff_post'],
                    [[KeyValue('wp', 'asv_wp' + str(idx + 1))]],
                    [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
                succeed = succeed and self.update_predicates(['connected'], [[
                    KeyValue('wp1', 'asv_wp' + str(idx + 1)),
                    KeyValue('wp2', 'uav_wp0')
                ]], [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
                for uav_wp_id in val['uav_wp']:
                    succeed = succeed and self.update_predicates(
                        ['connected'], [[
                            KeyValue('wp1', 'asv_wp' + str(idx + 1)),
                            KeyValue('wp2', 'uav_wp' + str(uav_wp_id))
                        ]], [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
        # add ASV features
        for asv in asvs:
            if asv['lr_camera_system']:
                succeed = succeed and self.update_predicates(
                    ['has_lr_camera'], [[KeyValue('v', asv['name'])]],
                    [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            if asv['charging_dock'] and len(asv['uav_onboard']) > 0:
                succeed = succeed and self.update_predicates(
                    ['has_charging_dock', 'charging_post'],
                    [[KeyValue('asv', asv['name'])],
                     [KeyValue('wp', 'uav_wp0')]], [
                         KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE,
                         KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE
                     ])
            for uav in asv['uav_onboard']:
                succeed = succeed and self.update_predicates(['has_uav'], [[
                    KeyValue('asv', asv['name']),
                    KeyValue('uav', uav)
                ]], [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            # add fuel, min, max condition
            succeed = succeed and self.update_functions(
                ['min_fuel'], [[KeyValue('v', asv['name'])]],
                [float(asv['min_fuel'])],
                [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            succeed = succeed and self.update_functions(
                ['max_fuel'], [[KeyValue('v', asv['name'])]],
                [float(asv['max_fuel'])],
                [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            succeed = succeed and self.update_functions(
                ['fuel'], [[KeyValue('v', asv['name'])]],
                [float(asv['max_fuel'])],
                [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            succeed = succeed and self.update_functions(
                ['consumption_rate'], [[KeyValue('v', asv['name'])]], [
                    norm.ppf(0.95,
                             loc=asv['fuel_rate'][0],
                             scale=asv['fuel_rate'][1])
                ], [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
        return succeed

    def add_mission(self, WTs):
        """
        Add (inspection) missions for ASVs to do
        """
        params = list()
        pred_names = list()
        update_types = list()
        inspect_wps = ['asv_wp' + str(self.wt_to_wp[wt]) for wt in WTs]
        for wp in inspect_wps:
            pred_names.append('turbine_inspected_at')
            params.append([KeyValue('wp', wp)])
            update_types.append(KnowledgeUpdateServiceRequest.ADD_GOAL)
            pred_names.append('inspect_post')
            params.append([KeyValue('wp', wp)])
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        self.goal_state[0].extend(pred_names)
        self.goal_state[1].extend(params)
        succeed = self.update_predicates(pred_names, params, update_types)
        succeed = succeed and self.home_mission()
        return succeed

    def home_mission(self):
        """
        Returning ASVs to the dock/home mission
        """
        params = list()
        pred_names = list()
        update_types = list()
        for asv in self.asvs:
            pred_names.append('at')
            params.append(
                [KeyValue('v', asv.namespace),
                 KeyValue('wp', 'asv_wp0')])
            update_types.append(KnowledgeUpdateServiceRequest.ADD_GOAL)
        self.goal_state[0].extend(pred_names)
        self.goal_state[1].extend(params)
        succeed = self.update_predicates(pred_names, params, update_types)
        return succeed

    def update_predicates(self, pred_names, parameters, update_types):
        """
        Add / remove first order facts or goals
        """
        self.mutex.acquire()
        success = True
        for idx, pred_name in enumerate(pred_names):
            req = KnowledgeUpdateServiceRequest()
            req.knowledge.knowledge_type = KnowledgeItem.FACT
            req.knowledge.attribute_name = pred_name
            req.knowledge.values.extend(parameters[idx])
            req.update_type = update_types[idx]
            success = success and self._knowledge_update_proxy(req).success
        self.mutex.release()
        return success

    def update_functions(self, func_names, params, func_values, update_types):
        """
        Add / remove functions
        """
        self.mutex.acquire()
        success = True
        for idx, func_name in enumerate(func_names):
            req = KnowledgeUpdateServiceRequest()
            req.knowledge.knowledge_type = KnowledgeItem.FUNCTION
            req.knowledge.attribute_name = func_name
            req.knowledge.values = params[idx]
            req.knowledge.function_value = func_values[idx]
            req.update_type = update_types[idx]
            success = success and self._knowledge_update_proxy(req).success
        self.mutex.release()
        return success

    def resume_plan(self):
        """
        Function to resume ongoing plan
        """
        self.action_sequence = 0
        for asv in self.asvs:
            asv.previous_mode = asv.current_mode
            asv.current_mode = asv.state.mode
            asv.external_intervened = False
            asv._cancel_action = False

    def cancel_plan(self):
        """
        Function to cancel current plan
        """
        self.action_sequence = 0
        for asv in self.asvs:
            asv._cancel_action = True

    def update_instances(self, ins_types, ins_names, update_types):
        """
        Add / remove instances
        """
        success = True
        for idx, ins_type in enumerate(ins_types):
            req = KnowledgeUpdateServiceRequest()
            req.knowledge.knowledge_type = KnowledgeItem.INSTANCE
            req.knowledge.instance_type = ins_type
            req.knowledge.instance_name = ins_names[idx]
            req.update_type = update_types[idx]
            success = success and self._knowledge_update_proxy(req).success
        return success

    def goto_waypoint(self, asv, params, dur=rospy.Duration(60, 0)):
        """
        Go to waypoint action for ASV
        """
        waypoint = -1
        for param in params:
            if param.key == 'to':
                waypoint = int(re.findall(r'\d+', param.value)[0])
                break
        if waypoint == 0:
            response = asv.return_to_launch(dur)
        elif waypoint == -1:
            response = ActionExecutor.ACTION_FAIL
        else:
            response = asv.goto(waypoint, dur, asv.low_fuel)
        return response

    def clear_mission(self):
        """
        Clear ASV related goals
        """
        if self.goal_state[0] != list():
            update_types = [
                KnowledgeUpdateServiceRequest.REMOVE_GOAL
                for _ in self.goal_state[0]
            ]
            self.update_predicates(self.goal_state[0], self.goal_state[1],
                                   update_types)
        self.goal_state = [list(), list()]

    def _dispatch_cb(self, msg):
        """
        Function for action_dispatch callback
        rosplan_dispatch sends unordered action sequences sometimes
        """
        self.dispatch_actions.append(msg)

    def _execute_action(self, event=True):
        """
        Function to execute the action when the action sequence is done
        """
        msg_executed = False
        for idx, msg in enumerate(self.dispatch_actions):
            if msg.action_id != self.action_sequence:
                continue
            # parse action message
            asv_names = [asv.namespace for asv in self.asvs]
            asv_name = [
                parm.value for parm in msg.parameters
                if parm.value in asv_names
            ]
            action_executed = False
            if len(asv_name):
                asv = [i for i in self.asvs if i.namespace == asv_name[0]][0]
                duration = self.get_max_action_duration(
                    msg.parameters, msg.duration)
                start = rospy.Time.now()
                if msg.name == 'asv_navigate':
                    action_executed = self._action(
                        msg, self.goto_waypoint,
                        [asv, msg.parameters, duration])
                elif msg.name == 'asv_inspect_wt':
                    action_executed = self._action(msg, asv.inspect_wt,
                                                   [duration])
                if action_executed:
                    self.update_action_duration(rospy.Time.now() - start,
                                                msg.parameters)
            msg_executed = True
            break
        if msg_executed:
            del self.dispatch_actions[idx]

    def get_max_action_duration(self, parameters, duration):
        """
        Get maximum action duration
        """
        wps = [
            int(re.findall(r'\d+', param.value)[0]) for param in parameters
            if 'asv' in param.value
        ]
        wps = [wps[0], wps[0]] if len(wps) == 1 else wps
        try:
            conn = [
                i[-2:] for i in self.connections if set(i[:2]) == set(wps)
            ][-1]
            max_duration = rospy.Duration(
                norm.ppf(0.95, loc=float(conn[0]), scale=float(conn[1])))
        except IndexError:
            max_duration = rospy.Duration(secs=int(duration))
        return max_duration

    def update_action_duration(self,
                               duration,
                               parameters,
                               std_dev_likelihood=15.0):
        """
        Update average and variance of the action duration
        """
        x = float("%d.%d" % (duration.secs, duration.nsecs))
        wps = [
            int(re.findall(r'\d+', param.value)[0]) for param in parameters
            if 'asv' in param.value
        ]
        wps = [wps[0], wps[0]] if len(wps) == 1 else wps
        connections = [(idx, i) for idx, i in enumerate(self.connections)
                       if set(i[:2]) == set(wps)]
        for idx, conn in connections:
            new_std = 1. / ((1. / conn[3]**2) + (1. / std_dev_likelihood**2))
            new_mean = ((float(conn[2]) / conn[3]**2) +
                        (x / std_dev_likelihood**2)) * new_std
            self.connections[idx] = [wps[0], wps[1], new_mean, new_std]
