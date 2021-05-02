#!/usr/bin/env python

# 3rd Party Packages
import re
from threading import Lock

import numpy as np
# ROS Packages
import rospy
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv import (GetAttributeService,
                                        GetDomainOperatorDetailsService,
                                        GetDomainPredicateDetailsService,
                                        KnowledgeUpdateService,
                                        KnowledgeUpdateServiceRequest)
from scipy.stats import norm
from uav_executive.executor import ActionExecutor
from uav_executive.preflightcheck import PreFlightCheck


class PlannerInterface(object):
    mutex = Lock()

    def __init__(self,
                 names,
                 uav_waypoints,
                 connections,
                 wt_to_wp,
                 update_frequency=10.):
        """
        A Class that interfaces with ROSPlan for dispatching uav actions and
        updating uav knowledge during a mission
        """
        self.action_sequence = 0
        self.wt_to_wp = wt_to_wp
        self.connections = connections
        self.dispatch_actions = list()
        self.total_wp = len(uav_waypoints)
        self.goal_state = [list(), list()]
        self._rate = rospy.Rate(update_frequency)
        self.lowbat = {name['name']: False for name in names}
        self.uavs = [ActionExecutor(name, uav_waypoints) for name in names]

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
            rospy.logerr('UAV instances in PDDL can\'t be set!')
        if not self.set_init(names, connections):
            rospy.logerr('UAV initial state can\'t be set!')
        # Auto call functions
        rospy.Timer(self._rate.sleep_dur, self.fact_update)
        rospy.Timer(10 * self._rate.sleep_dur, self._battery_update)
        rospy.Timer(self._rate.sleep_dur, self._execute_action)
        rospy.sleep(20 * self._rate.sleep_dur)

    def _feedback_cb(self, msg):
        """
        Function to make sure this executes actions in the right order
        """
        self.action_sequence = msg.action_id + 1 if (
            msg.status == 'action achieved') else self.action_sequence
        if msg.status in ['OLAM prepared', 'OLAM failed']:
            for idx, dispatch in enumerate(self.dispatch_actions):
                if msg.action_id != dispatch.action_id:
                    continue
                uav_names = [uav.namespace for uav in self.uavs]
                uav_name = [
                    parm.value for parm in dispatch.parameters
                    if parm.value in uav_names
                ]
                if len(uav_name):
                    uav = [i for i in self.uavs
                           if i.namespace == uav_name[0]][0]
                    uav._irr_ready_to_be_picked = 1 if (
                        msg.status == 'OLAM prepared') else -1
                break

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

    def _battery_update(self, event):
        """
        Update battery value on ROSPlan knowledge base
        """
        for uav in self.uavs:
            # battery status update
            self.update_functions(
                ['fuel'], [[KeyValue('v', uav.namespace)]],
                [np.mean(uav.battery_voltages)], [
                    KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE,
                ])
            self.lowbat[uav.namespace] = uav.low_battery

    def _landing_update(self):
        """
        Add or remove landing/airborne facts on ROSPlan knowledge base
        """
        landed_uav = list()
        params = list()
        pred_names = list()
        update_types = list()
        attributes = self._proposition_proxy('ground').attributes
        for attribute in attributes:
            name = attribute.values[0].value
            uav = [i for i in self.uavs if i.namespace == name]
            if len(uav):
                landed_uav.append(name)
                if not uav[0].landed:
                    pred_names.extend(['ground', 'airborne'])
                    params.extend([[KeyValue('v', uav[0].namespace)],
                                   [KeyValue('v', uav[0].namespace)]])
                    update_types.extend([
                        KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE,
                        KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE
                    ])
        for uav in self.uavs:
            if (uav.namespace not in landed_uav) and uav.landed:
                pred_names.extend(['ground', 'airborne'])
                params.extend([[KeyValue('v', uav.namespace)],
                               [KeyValue('v', uav.namespace)]])
                update_types.extend([
                    KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE,
                    KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE
                ])
        if pred_names != list():
            self.update_predicates(pred_names, params, update_types)

    def _wp_update(self):
        """
        Add or remove waypoint facts on ROSPlan knowledge base
        """
        wp_uav = list()
        params = list()
        pred_names = list()
        update_types = list()
        attributes = self._proposition_proxy('at').attributes
        for attribute in attributes:
            name = attribute.values[0].value
            uav = [i for i in self.uavs if i.namespace == name]
            if len(uav):
                wp_uav.append(name)
                at = int(re.findall(r'\d+', attribute.values[1].value)[0])
                if uav[0]._current_wp != -1 and at != uav[0]._current_wp:
                    # add current wp that uav resides
                    pred_names.append('at')
                    params.append([
                        KeyValue('v', uav[0].namespace),
                        KeyValue('wp', 'uav_wp%d' % uav[0]._current_wp)
                    ])
                    update_types.append(
                        KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
                    # remove previous wp that uav resided
                    pred_names.append('at')
                    params.append([
                        KeyValue('v', uav[0].namespace),
                        KeyValue('wp', 'uav_wp%d' % at)
                    ])
                    update_types.append(
                        KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE)
        for uav in self.uavs:
            if not (uav.namespace in wp_uav) and uav._current_wp != -1:
                # add current wp that uav resides
                pred_names.append('at')
                params.append([
                    KeyValue('v', uav.namespace),
                    KeyValue('wp', 'uav_wp%d' % uav._current_wp)
                ])
                update_types.append(
                    KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        if pred_names != list():
            self.update_predicates(pred_names, params, update_types)

    def fact_update(self, event):
        """
        Add or remove facts related to UAVs at ROSPlan knowledge base
        """
        self._wp_update()
        self._landing_update()

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
        Set initial instances for UAVs to ROSPlan
        """
        ins_types = list()
        ins_names = list()
        update_types = list()
        for idx in range(self.total_wp + 1):
            ins_types.append('waypoint')
            ins_names.append('uav_wp' + str(idx))
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        for uav in self.uavs:
            ins_types.append('uav')
            ins_names.append(uav.namespace)
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        return self.update_instances(ins_types, ins_names, update_types)

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

    def resume_plan(self):
        """
        Function to flag down human intervention
        """
        self.action_sequence = 0
        for uav in self.uavs:
            uav.previous_mode = uav.current_mode
            uav.current_mode = uav.state.mode
            uav.external_intervened = False
            uav._cancel_action = False

    def cancel_plan(self):
        """
        Function to cancel current plan
        """
        self.action_sequence = 0
        for uav in self.uavs:
            uav._cancel_action = True

    def goto_waypoint(self, uav, params, dur=rospy.Duration(60, 0)):
        """
        Go to waypoint action for UAV
        """
        waypoint = -1
        for param in params:
            if param.key == 'to':
                waypoint = int(re.findall(r'\d+', param.value)[0])
                break
        response = uav.goto(
            waypoint, dur,
            uav.low_battery) if waypoint != -1 else ActionExecutor.ACTION_FAIL
        return response

    def home_mission(self):
        """
        UAV return to launch due to low battery voltage
        """
        params = list()
        pred_names = list()
        update_types = list()
        for uav in self.uavs:
            pred_names.extend(['ground', 'at'])
            params.extend([
                [KeyValue('v', self.uavs[0].namespace)],
                [
                    KeyValue('v', self.uavs[0].namespace),
                    KeyValue('wp', 'uav_wp0')
                ],
            ])
            update_types.extend([
                KnowledgeUpdateServiceRequest.ADD_GOAL,
                KnowledgeUpdateServiceRequest.ADD_GOAL
            ])
        self.goal_state[0].extend(pred_names)
        self.goal_state[1].extend(params)
        succeed = self.update_predicates(pred_names, params, update_types)
        return succeed

    def clear_mission(self):
        """
        Clear UAV related goals
        """
        if self.goal_state[0] != list():
            update_types = [
                KnowledgeUpdateServiceRequest.REMOVE_GOAL
                for _ in self.goal_state[0]
            ]
            self.update_predicates(self.goal_state[0], self.goal_state[1],
                                   update_types)
        self.goal_state = [list(), list()]

    def add_mission(self, WTs, irr_WTs):
        """
        Add (inspection) missionas for UAVs to do
        """
        params = list()
        pred_names = list()
        update_types = list()
        inspect_wps = ['uav_wp' + str(self.wt_to_wp[wt]) for wt in WTs]
        for wp in inspect_wps:
            pred_names.extend(['turbine_inspected_at', 'inspect_post'])
            params.extend([[KeyValue('wp', wp)], [KeyValue('wp', wp)]])
            update_types.extend([
                KnowledgeUpdateServiceRequest.ADD_GOAL,
                KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE
            ])
        deploy_wps = ['uav_wp' + str(self.wt_to_wp[wt]) for wt in irr_WTs]
        for wp in deploy_wps:
            pred_names.append('deploy_retrieve_post')
            params.append([KeyValue('wp', wp)])
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        self.goal_state[0].extend(pred_names)
        self.goal_state[1].extend(params)
        succeed = self.update_predicates(pred_names, params, update_types)
        succeed = succeed and self.home_mission()
        return succeed

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
            duration = self.get_max_action_duration(msg.parameters,
                                                    msg.duration)
            # parse action message
            uav_names = [uav.namespace for uav in self.uavs]
            uav_name = [
                parm.value for parm in msg.parameters
                if parm.value in uav_names
            ]
            if len(uav_name):
                uav = [i for i in self.uavs if i.namespace == uav_name[0]][0]
                start = rospy.Time.now()
                if msg.name == 'uav_takeoff':
                    self._action(
                        msg, uav.takeoff,
                        [rospy.get_param('~takeoff_altitude', 10.), duration])
                elif msg.name == 'uav_land':
                    self._action(msg, uav.return_to_launch, [True, duration])
                elif msg.name == 'uav_navigate':
                    self._action(msg, self.goto_waypoint,
                                 [uav, msg.parameters, duration])
                elif msg.name == 'uav_inspect_blade':
                    self._action(msg, uav.inspect_wt, [duration])
                elif msg.name == 'uav_retrieve_irr':
                    self._action(msg, uav.retrieve_irr,
                                 [msg.parameters[1].value, duration])
                elif msg.name == 'uav_deploy_irr':
                    self._action(msg, uav.deploy_irr, [duration])
                elif msg.name == 'uav_refuelling':
                    self._action(msg, uav.refuelling, [duration])
                self.update_action_duration(rospy.Time.now() - start,
                                            msg.parameters)
                self.update_predicates(
                    ['idle'], [[KeyValue('v', uav.namespace)]],
                    [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            msg_executed = True
            break
        if msg_executed:
            del self.dispatch_actions[idx]

    def preflightcheck(self, uav, duration=rospy.Duration(600, 0)):
        """
        Preflight check action for UAV
        """
        start = rospy.Time.now()
        # preflight process
        preflightcheck = PreFlightCheck(uav)
        try:
            init_batt = float(preflightcheck._init_batt.get())
            min_batt = float(preflightcheck._low_batt.get())
            uav.set_battery(init_batt, min_batt)
            # add minimum-battery condition
            succeed = self.update_functions(
                ['minimum-battery'], [[KeyValue('v', uav.namespace)]],
                [min_batt - 0.15],
                [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            resp = ActionExecutor.ACTION_SUCCESS if (
                succeed and self._preflightcheck(preflightcheck)
            ) else ActionExecutor.ACTION_FAIL
        except ValueError:
            rospy.logwarn(
                'Pre-flight check for %s doesn\'t have correct inputs!' %
                uav.namespace)
            resp = ActionExecutor.ACTION_FAIL
        if (rospy.Time.now() - start) > duration:
            resp = ActionExecutor.OUT_OF_DURATION
        return resp

    def _preflightcheck(self, preflightcheck):
        pilot = preflightcheck._pilot.get() != ''
        gc = preflightcheck._gc.get() != ''
        location = preflightcheck._location.get() != ''
        takeoff_amsl = ('%.2f' %
                        float(preflightcheck._takeoff_amsl.get())) > 0.
        planned_agl = ('%.2f' % float(preflightcheck._planned_agl.get())) > 0.
        filled = (pilot and gc and location and takeoff_amsl and planned_agl)

        rf_noise = preflightcheck._rf_noise.get()
        airframe = preflightcheck._airframe.get()
        hatch = preflightcheck._hatch.get()
        range_check = preflightcheck._range_check.get()
        ground_station = preflightcheck._ground_station.get()
        cl = rf_noise and airframe and hatch and range_check and ground_station

        telem = preflightcheck._telemetry_check.get()
        transmitter = preflightcheck._transmitter_check.get()
        magnetometer = preflightcheck._magnetometer_check.get()
        gps = preflightcheck._gps_check.get()
        ahrs = preflightcheck._ahrs_check.get()
        cm = preflightcheck._camera_check.get()
        cl2 = telem and transmitter and magnetometer and gps and ahrs and cm

        barometer = preflightcheck._barometer_check.get()
        motor = preflightcheck._motor_check.get()
        airtraf = preflightcheck._airtraffic_check.get()
        people = preflightcheck._people_check.get()
        pilot_ch = preflightcheck._pilot_check.get()
        gpsch = preflightcheck._gps_check.get()
        cl3 = motor and barometer and airtraf and people and pilot_ch and gpsch
        return filled and cl and cl2 and cl3

    def get_max_action_duration(self, parameters, duration):
        """
        Get maximum action duration
        """
        wps = [
            int(re.findall(r'\d+', param.value)[0]) for param in parameters
            if 'uav' in param.value
        ]
        wps = [0, 0] if len(wps) == 0 else wps
        wps = wps[:2] if len(wps) > 2 else wps
        wps = [wps[0], wps[0]] if len(wps) == 1 else wps
        try:
            conn = [
                i[-2:] for i in self.connections if set(i[:2]) == set(wps)
            ][-1]
            max_duration = rospy.Duration(
                norm.ppf(0.95, loc=float(conn[0]), scale=float(conn[1])))
        except IndexError:
            max_duration = duration
        return max_duration

    def update_action_duration(self,
                               duration,
                               parameters,
                               std_dev_likelihood=1.0):
        """
        Update average and variance of the action duration
        """
        x = float("%d.%d" % (duration.secs, duration.nsecs))
        wps = [
            int(re.findall(r'\d+', param.value)[0]) for param in parameters
            if 'uav' in param.value
        ]
        wps = [0, 0] if len(wps) == 0 else wps
        wps = wps[:2] if len(wps) > 2 else wps
        wps = [wps[0], wps[0]] if len(wps) == 1 else wps
        idx, conn = [(idx, i) for idx, i in enumerate(self.connections)
                     if set(i[:2]) == set(wps)][-1]
        new_mean = ((float(conn[2]) / conn[3]**2) +
                    (x / std_dev_likelihood**2)) / (
                        (1. / conn[3]**2) + (1. / std_dev_likelihood**2))
        new_std = 1. / ((1. / conn[3]**2) + (1. / std_dev_likelihood**2))
        self.connections[idx] = [wps[0], wps[1], new_mean, new_std]

    def set_init(self, uavs, connections):
        """
        Adding facts to the initial state
        """
        # adding landing_post and takeoff_post facts
        succeed = self.update_predicates(
            ['landing_post'], [[KeyValue('wp', 'uav_wp0')]],
            [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
        if len(self._proposition_proxy('takeoff_post').attributes) == 0:
            succeed = succeed and self.update_predicates(
                ['takeoff_post'], [[KeyValue('wp', 'uav_wp0')]],
                [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
        # add wp connection
        connections.extend([[j, i, k, l] for i, j, k, l in connections
                            if i != j])
        for i in connections:
            succeed = succeed and self.update_predicates(['connected'], [[
                KeyValue('wp1', 'uav_wp' + str(i[0])),
                KeyValue('wp2', 'uav_wp' + str(i[1]))
            ]], [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            # add min action duration on the wp connection
            succeed = succeed and self.update_functions(['min_dur'], [[
                KeyValue('wp1', 'uav_wp' + str(i[0])),
                KeyValue('wp2', 'uav_wp' + str(i[1]))
            ]], [norm.ppf(0.05, loc=float(i[2]), scale=float(i[3]))
                 ], [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            # add max action duration on the wp connection
            succeed = succeed and self.update_functions(['max_dur'], [[
                KeyValue('wp1', 'uav_wp' + str(i[0])),
                KeyValue('wp2', 'uav_wp' + str(i[1]))
            ]], [norm.ppf(0.95, loc=float(i[2]), scale=float(i[3]))
                 ], [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
        # add UAV features
        for uav in uavs:
            succeed = succeed and self.update_predicates(
                ['idle'], [[KeyValue('v', uav['name'])]],
                [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            if uav['deploy_system']:
                succeed = succeed and self.update_predicates(
                    ['has_deploy_system'], [[KeyValue('v', uav['name'])]],
                    [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            if uav['retrieve_system']:
                succeed = succeed and self.update_predicates(
                    ['has_retrieval_system'], [[KeyValue('uav', uav['name'])]],
                    [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            if uav['camera_system']:
                succeed = succeed and self.update_predicates(
                    ['has_camera'], [[KeyValue('uav', uav['name'])]],
                    [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            if uav['irr_attached'] != "":
                succeed = succeed and self.update_predicates(
                    ['attached_to'], [[
                        KeyValue('irr', uav['irr_attached']),
                        KeyValue('uav', uav['name'])
                    ]], [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            # add fuel, min, max condition
            succeed = succeed and self.update_functions(
                ['min_fuel'], [[KeyValue('v', uav['name'])]],
                [float(uav['min_fuel'])],
                [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            succeed = succeed and self.update_functions(
                ['max_fuel'], [[KeyValue('v', uav['name'])]],
                [float(uav['max_fuel'])],
                [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            succeed = succeed and self.update_functions(
                ['fuel'], [[KeyValue('v', uav['name'])]],
                [float(uav['max_fuel'])],
                [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
            succeed = succeed and self.update_functions(
                ['consumption_rate'], [[KeyValue('v', uav['name'])]], [
                    norm.ppf(0.95,
                             loc=uav['fuel_rate'][0],
                             scale=uav['fuel_rate'][1])
                ], [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
        return succeed
