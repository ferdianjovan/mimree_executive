#!/usr/bin/env python

# 3rd Party Packages
import re
from threading import Lock

import numpy as np
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


class PlannerInterface(object):

    mutex = Lock()

    def __init__(self,
                 names,
                 asv_waypoints,
                 uavs=list(),
                 uav_carrier='',
                 update_frequency=10.):
        """
        A Class that interfaces with ROSPlan for dispatching asv actions and
        updating asv knowledge during a mission
        """
        self.goal_state = list()
        self.uavs = uavs
        self.uav_carrier = uav_carrier
        self.waypoints = asv_waypoints
        self._rate = rospy.Rate(update_frequency)
        self.lowfuel = {name: False for name in names}
        self.asvs = [ActionExecutor(name, self.waypoints) for name in names]

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
        self._feedback_publisher = rospy.Publisher(
            '/rosplan_plan_dispatcher/action_feedback',
            ActionFeedback,
            queue_size=10)

        self.instances_set = self.set_instances()
        self.init_set = False
        if not self.instances_set:
            rospy.logerr('ASV instances in PDDL can\'t be set!')
        self.init_set = self.set_init()
        if not self.init_set:
            rospy.logerr('ASV initial state can\'t be set!')
        # Auto call functions
        rospy.Timer(self._rate.sleep_dur, self.fact_update)
        rospy.Timer(50 * self._rate.sleep_dur, self.function_update)

    def low_fuel_return_mission(self):
        """
        ASV return to launch due to low fuel voltage
        """
        if len(self.asvs) == 1:
            self._rate.sleep()
            # Remove current goal
            update_types = [
                KnowledgeUpdateServiceRequest.REMOVE_GOAL
                for _ in self.goal_state[0]
            ]
            self.update_predicates(self.goal_state[0], self.goal_state[1],
                                   update_types)
            self._rate.sleep()
            # Add new goal
            pred_names = ['landed', 'at']
            params = [
                [KeyValue('v', self.asvs[0].namespace)],
                [
                    KeyValue('v', self.asvs[0].namespace),
                    KeyValue('wp', 'asv_wp0')
                ],
            ]
            self.goal_state = (pred_names, params)
            update_types = [
                KnowledgeUpdateServiceRequest.ADD_GOAL,
                KnowledgeUpdateServiceRequest.ADD_GOAL
            ]
            self.update_predicates(pred_names, params, update_types)
            self._rate.sleep()

    def inspection_mission(self):
        """
        ASV mission visiting all waypoints and coming back to launch pod
        """
        # 'all waypoints must be visited' goals
        pred_names = ['visited' for _ in self.waypoints]
        params = [[KeyValue('wp', 'asv_wp%d' % (idx + 1))]
                  for idx, _ in enumerate(self.waypoints)]
        update_types = [
            KnowledgeUpdateServiceRequest.ADD_GOAL for _ in self.waypoints
        ]
        # 'all asvs must be back at home' goals
        pred_names.extend(['at' for _ in self.asvs])
        params.extend(
            [[KeyValue('v', asv.namespace),
              KeyValue('wp', 'asv_wp0')] for asv in self.asvs])
        update_types.extend(
            [KnowledgeUpdateServiceRequest.ADD_GOAL for _ in self.asvs])
        self.goal_state = (pred_names, params)
        succeed = self.update_predicates(pred_names, params, update_types)
        self._rate.sleep()
        return succeed

    def deploy_retrieve_mission(self):
        """
        ASV mission visiting all waypoints and coming back to launch pod
        """
        # 'all asvs must be back at home' goals
        pred_names = ['at' for _ in self.asvs]
        params = [[KeyValue('v', asv.namespace),
                   KeyValue('wp', 'asv_wp0')] for asv in self.asvs]
        update_types = [
            KnowledgeUpdateServiceRequest.ADD_GOAL for _ in self.asvs
        ]
        self.goal_state = (pred_names, params)
        succeed = self.update_predicates(pred_names, params, update_types)
        self._rate.sleep()
        return succeed

    def fact_update(self, event):
        """
        Add or remove facts related to ASVs at ROSPlan knowledge base
        """
        self._wp_update()
        self._arm_update()

    def function_update(self, event):
        """
        Update functions related to ASVs at ROSPlan knowledge base
        """
        self._fuel_update()

    def resume_plan(self):
        """
        Function to flag down human intervention
        """
        for asv in self.asvs:
            asv.previous_mode = asv.current_mode
            asv.current_mode = asv.state.mode
            asv.external_intervened = False

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
        for idx in range(len(self.waypoints) + 1):
            ins_types.append('asv_waypoint')
            ins_names.append('asv_wp' + str(idx))
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        for asv in self.asvs:
            ins_types.append('asv')
            ins_names.append(asv.namespace)
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        return self.update_instances(ins_types, ins_names, update_types)

    def set_init(self):
        """
        Adding facts to the initial state
        """
        # add wp connection assuming first and last wp can go to wp0 / home
        connections = [(i, i + 1) for i, _ in enumerate(self.waypoints)]
        connections.extend([(0, len(self.waypoints))])
        connections.extend([(j, i) for i, j in connections])
        # add wp connection from takeoff waypoint to home and last wp
        connections.extend([(i, 'asv_wp0') for i in self.takeoff_wps
                            if ('asv' in i)])
        connections.extend([(i, 'asv_wp%d' + len(self.waypoints))
                            for i in self.takeoff_wps if ('asv' in i)])
        succeed = self.update_predicates(
            ['connected' for _ in connections], [[
                KeyValue('wp1', 'asv_wp' + str(i[0])),
                KeyValue('wp2', 'asv_wp' + str(i[1]))
            ] for i in connections],
            [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE for _ in connections])
        # add home wp position
        succeed = succeed and self.update_predicates(
            ['home'], [[KeyValue('wp', 'asv_wp0')]],
            [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
        # add carrier asv
        if self.uav_carrier != '':
            succeed = succeed and self.update_predicates(
                ['carrier'], [[KeyValue('v', self.uav_carrier)]],
                [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
        # add minimum-fuel condition
        succeed = succeed and self.update_functions(
            ['minimum-fuel'
             for _ in self.asvs], [[KeyValue('v', asv.namespace)]
                                   for asv in self.asvs],
            [ActionExecutor.MINIMUM_FUEL for _ in self.asvs],
            [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE for _ in self.asvs])
        # add initial fuel-percentage condition
        succeed = succeed and self.update_functions(
            ['fuel-percentage'
             for _ in self.asvs], [[KeyValue('v', asv.namespace)]
                                   for asv in self.asvs],
            [ActionExecutor.INIT_FUEL for _ in self.asvs],
            [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE for _ in self.asvs])
        return succeed

    def goto_waypoint(self, asv, params, dur=rospy.Duration(60, 0)):
        """
        Go to waypoint action for ASV
        """
        waypoint = -1
        for param in params:
            if param.key == 'to':
                waypoint = int(re.findall(r'\d+', param.value)[0])
                break
        response = asv.goto(
            waypoint, dur) if waypoint != -1 else ActionExecutor.ACTION_FAIL
        return response

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
        Template asv action to respond to the dispatched action
        """
        self.publish_feedback(action_dispatch.action_id, 'action enabled')
        start_time = rospy.Time(action_dispatch.dispatch_time)
        duration = rospy.Duration(action_dispatch.duration)
        self._rate.sleep()
        rospy.loginfo('Dispatching %s action at %s with duration %s ...' %
                      (action_dispatch.name, str(
                          start_time.secs), str(duration.to_sec())))
        if action_func(*action_params) == ActionExecutor.ACTION_SUCCESS:
            if self._apply_operator_effect(action_dispatch.name,
                                           action_dispatch.parameters):
                self.publish_feedback(action_dispatch.action_id,
                                      'action achieved')
            else:
                self.publish_feedback(action_dispatch.action_id,
                                      'action failed')
        else:
            self.publish_feedback(action_dispatch.action_id, 'action failed')

    def _dispatch_cb(self, msg):
        """
        Function for action_dispatch callback
        """
        duration = rospy.Duration(msg.duration)
        # parse action message
        nme = [parm.value for parm in msg.parameters if parm.key == 'v'][0]
        asv = [i for i in self.asvs if i.namespace == nme]
        if msg.name == 'asv_request_arm':
            self._action(msg, asv[0].arm, [duration])
        elif (msg.name == 'asv_goto_waypoint') or (
                msg.name == 'asv_goto_waypoint_with_uav'):
            self._action(msg, self.goto_waypoint,
                         [asv[0], msg.parameters, duration])
        elif msg.name == 'asv_rtl' or msg.name == 'asv_lowfuel_return':
            self._action(msg, asv[0].return_to_launch, [duration])
        elif (msg.name == 'asv_rtl_with_uav') or (
                msg.name == 'asv_lowfuel_return_with_uav'):
            self._action(msg, asv[0].return_to_launch, [duration])

    def _arm_update(self):
        """
        Add or remove arm facts on ROSPlan knowledge base
        """
        armed_asv = list()
        params = list()
        pred_names = list()
        update_types = list()
        attributes = self._proposition_proxy('armed').attributes
        for attribute in attributes:
            name = attribute.values[0].value
            asv = [i for i in self.asvs if i.namespace == name]
            if len(asv):
                armed_asv.append(name)
                if not asv[0].state.armed:
                    pred_names.extend(['armed'])
                    params.extend([[KeyValue('v', asv[0].namespace)]])
                    update_types.extend([
                        KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE,
                    ])
        for asv in self.asvs:
            if not (asv.namespace in armed_asv) and asv.state.armed:
                pred_names.extend(['armed'])
                params.extend([[KeyValue('v', asv.namespace)]])
                update_types.extend([
                    KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE,
                ])
        if pred_names != list():
            self.update_predicates(pred_names, params, update_types)

    def _uav_wp_update(self, cur_wp, prev_wp):
        """
        Add UAV asv-waypoint facts on ROSPlan knowledge base
        when UAV is at a ASV
        """
        params = list()
        pred_names = list()
        update_types = list()
        attributes = self._proposition_proxy('at').attributes
        for attribute in attributes:
            name = attribute.values[0].value
            uav = [i for i in self.uavs if i.namespace == name]
            if len(uav) and attribute.values[1].value == 'uav_wp0':
                attributes = self._proposition_proxy('landed').attributes
                uav2 = [i for i in self.uavs if i.namespace == name]
                if len(uav2) and uav2[0].namespace == uav[0].namespace:
                    # add current wp that asv resides
                    pred_names.append('at')
                    params.append([
                        KeyValue('v', uav[0].namespace),
                        KeyValue('wp', 'asv_wp%d' % cur_wp)
                    ])
                    update_types.append(
                        KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
                    # remove previous wp that asv resided
                    pred_names.append('at')
                    params.append([
                        KeyValue('v', uav[0].namespace),
                        KeyValue('wp', 'asv_wp%d' % prev_wp)
                    ])
                    update_types.append(
                        KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE)
        return pred_names, params, update_types

    def _wp_update(self):
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
                    # update visited state
                    pred_names.append('visited')
                    params.append(
                        [KeyValue('wp', 'asv_wp%d' % asv[0]._current_wp)])
                    update_types.append(
                        KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
                    if self.uav_carrier == asv[0].namespace:
                        result = self._uav_wp_update(asv[0]._current_wp, at)
                        pred_names.extend(result[0])
                        params.extend(result[1])
                        update_types.extend(result[2])
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
                # update visited state
                pred_names.append('visited')
                params.append([KeyValue('wp', 'asv_wp%d' % asv._current_wp)])
                update_types.append(
                    KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
                if self.uav_carrier == asv[0].namespace:
                    result = self._uav_wp_update(asv[0]._current_wp, at)
                    pred_names.extend(result[0])
                    params.extend(result[1])
                    update_types.extend(result[2])
        if pred_names != list():
            self.update_predicates(pred_names, params, update_types)

    def _fuel_update(self):
        """
        Update fuel value on ROSPlan knowledge base
        """
        for asv in self.asvs:
            # fuel status update
            self.update_functions(
                ['fuel-percentage', 'minimum-fuel'],
                [[KeyValue('v', asv.namespace)],
                 [KeyValue('v', asv.namespace)]],
                [np.mean(asv.fuels), asv.MINIMUM_FUEL], [
                    KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE,
                    KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE
                ])
            self.lowfuel[asv.namespace] = asv.low_fuel
