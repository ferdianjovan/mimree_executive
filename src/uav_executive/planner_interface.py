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
from uav_executive.executor import ActionExecutor
from uav_executive.preflightcheck import PreFlightCheck


class PlannerInterface(object):

    mutex = Lock()

    def __init__(self,
                 names,
                 uav_waypoints,
                 asv_waypoints=list(),
                 update_frequency=10.):
        """
        A Class that interfaces with ROSPlan for dispatching uav actions and
        updating uav knowledge during a mission
        """
        self.goal_state = list()
        self.waypoints = uav_waypoints
        self.low_battery_mission = False
        self._rate = rospy.Rate(update_frequency)
        self.lowbat = {name: False for name in names}
        self.takeoff_wps = self.get_takeoff_wps(asv_waypoints)
        self.uavs = [ActionExecutor(name, self.waypoints) for name in names]

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
            rospy.logerr('UAV instances in PDDL can\'t be set!')
        self.init_set = self.set_init()
        if not self.init_set:
            rospy.logerr('UAV initial state can\'t be set!')
        # Auto call functions
        rospy.Timer(self._rate.sleep_dur, self.fact_update)
        rospy.Timer(10 * self._rate.sleep_dur, self.function_update)
        rospy.sleep(20 * self._rate.sleep_dur)

    def get_takeoff_wps(self, waypoints):
        """
        Get ASV areas where UAV can takeoff
        """
        takeoff_wps = [
            'asv_wp' + str(idx + 1) for idx, val in enumerate(waypoints)
            if bool(val['takeoff'])
        ]
        if takeoff_wps == []:
            takeoff_wps = ['uav_wp0']
        return takeoff_wps

    def low_battery_return_mission(self, all_return=False):
        """
        UAV return to launch due to low battery voltage
        """
        if len(self.uavs) == 1 or all_return:
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
                [KeyValue('v', self.uavs[0].namespace)],
                [
                    KeyValue('v', self.uavs[0].namespace),
                    KeyValue('wp', 'uav_wp0')
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
        UAV mission visiting all waypoints and coming back to launch pod
        """
        # 'a drone must do inspect action' goals
        pred_names = ['inspected' for wp in self.waypoints if wp['inspect']]
        params = [[KeyValue('wp', 'uav_wp%d' % (idx + 1))]
                  for idx, wp in enumerate(self.waypoints) if wp['inspect']]
        update_types = [
            KnowledgeUpdateServiceRequest.ADD_GOAL for wp in self.waypoints
            if wp['inspect']
        ]
        # 'all drones must be back at home' goals
        pred_names.extend(['at' for _ in self.uavs])
        params.extend(
            [[KeyValue('v', uav.namespace),
              KeyValue('wp', 'uav_wp0')] for uav in self.uavs])
        update_types.extend(
            [KnowledgeUpdateServiceRequest.ADD_GOAL for _ in self.uavs])
        # 'all drones must be landed' goals
        pred_names.extend(['landed' for _ in self.uavs])
        params.extend([[KeyValue('v', uav.namespace)] for uav in self.uavs])
        update_types.extend(
            [KnowledgeUpdateServiceRequest.ADD_GOAL for _ in self.uavs])
        self.goal_state = (pred_names, params)
        succeed = self.update_predicates(pred_names, params, update_types)
        self._rate.sleep()
        return succeed

    def fact_update(self, event):
        """
        Add or remove facts related to UAVs at ROSPlan knowledge base
        """
        self._wp_update()
        self._arm_update()
        self._landing_update()

    def function_update(self, event):
        """
        Update functions related to UAVs at ROSPlan knowledge base
        """
        self._battery_update()

    def resume_plan(self):
        """
        Function to flag down human intervention
        """
        for uav in self.uavs:
            uav.previous_mode = uav.current_mode
            uav.current_mode = uav.state.mode
            uav.external_intervened = False
            uav._cancel_action = False

    def cancel_plan(self):
        """
        Function to cancel current plan
        """
        for uav in self.uavs:
            uav._cancel_action = True

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
        Set initial instances for UAVs to ROSPlan
        """
        ins_types = list()
        ins_names = list()
        update_types = list()
        for idx in range(len(self.waypoints) + 1):
            ins_types.append('uav_waypoint')
            ins_names.append('uav_wp' + str(idx))
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        for uav in self.uavs:
            ins_types.append('uav')
            ins_names.append(uav.namespace)
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        return self.update_instances(ins_types, ins_names, update_types)

    def set_init(self):
        """
        Adding facts to the initial state
        """
        # add wp connection assuming first and last wp can go to wp0 / home
        connections = [(i, i + 1) for i, _ in enumerate(self.waypoints)]
        connections.extend([(j, i) for i, j in connections])
        succeed = self.update_predicates(
            ['connected' for _ in connections], [[
                KeyValue('wp1', 'uav_wp' + str(i[0])),
                KeyValue('wp2', 'uav_wp' + str(i[1]))
            ] for i in connections],
            [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE for _ in connections])
        # add wp connection from takeoff waypoint to home and last wp
        connections = [(i, 'uav_wp0') for i in self.takeoff_wps
                       if ('asv' in i)]
        succeed = self.update_predicates(
            ['connected' for _ in connections],
            [[KeyValue('wp1', i[0]),
              KeyValue('wp2', i[1])] for i in connections],
            [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE for _ in connections])
        # add home wp position
        succeed = succeed and self.update_predicates(
            ['home'], [[KeyValue('wp', 'uav_wp0')]],
            [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE])
        # add wp where uav can takeoff
        succeed = succeed and self.update_predicates(
            ['takeoff' for _ in self.takeoff_wps],
            [[KeyValue('wp', i)] for i in self.takeoff_wps], [
                KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE
                for _ in self.takeoff_wps
            ])
        # add wp where uav can do inspection
        succeed = succeed and self.update_predicates(
            ['inspect' for wp in self.waypoints if wp['inspect']],
            [[KeyValue('wp', 'uav_wp%d' % (i + 1))]
             for i, wp in enumerate(self.waypoints) if wp['inspect']], [
                 KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE
                 for wp in self.waypoints if wp['inspect']
             ])
        # add minimum-battery condition
        succeed = succeed and self.update_functions(
            ['minimum-battery'
             for _ in self.uavs], [[KeyValue('v', uav.namespace)]
                                   for uav in self.uavs],
            [ActionExecutor.MINIMUM_VOLTAGE - 0.15 for _ in self.uavs],
            [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE for _ in self.uavs])
        # add initial battery-amount condition
        succeed = succeed and self.update_functions(
            ['battery-amount'
             for _ in self.uavs], [[KeyValue('v', uav.namespace)]
                                   for uav in self.uavs],
            [ActionExecutor.INIT_VOLTAGE for _ in self.uavs],
            [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE for _ in self.uavs])
        return succeed

    def goto_waypoint(self, uav, params, dur=rospy.Duration(60, 0)):
        """
        Go to waypoint action for UAV
        """
        waypoint = -1
        for param in params:
            if param.key == 'to':
                waypoint = int(re.findall(r'\d+', param.value)[0])
                break
        response = uav.goto(waypoint, dur, self.low_battery_mission
                            ) if waypoint != -1 else ActionExecutor.ACTION_FAIL
        return response

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
        uav_names = [uav.namespace for uav in self.uavs]
        uav_name = [
            parm.value for parm in msg.parameters if parm.value in uav_names
        ]
        if len(uav_name):
            uav = [i for i in self.uavs if i.namespace == uav_name[0]][0]
            if msg.name == 'uav_preflightcheck':
                self._action(msg, self.preflightcheck, [uav, duration])
            elif msg.name == 'uav_request_arm':
                self._action(msg, uav.request_arm, [False, duration])
            elif msg.name == 'uav_takeoff':
                self._action(
                    msg, uav.takeoff,
                    [rospy.get_param('~takeoff_altitude', 10.), duration])
            elif msg.name == 'uav_inspect_blade':
                self._action(msg, uav.inspect_wt, [duration])
            elif msg.name == 'uav_goto_waypoint':
                self._action(msg, self.goto_waypoint,
                             [uav, msg.parameters, duration])
            elif msg.name == 'uav_rtl' or msg.name == 'uav_lowbat_return':
                self._action(msg, uav.return_to_launch, [True, duration])

    def _landing_update(self):
        """
        Add or remove landing/airborne facts on ROSPlan knowledge base
        """
        landed_uav = list()
        params = list()
        pred_names = list()
        update_types = list()
        attributes = self._proposition_proxy('landed').attributes
        for attribute in attributes:
            name = attribute.values[0].value
            uav = [i for i in self.uavs if i.namespace == name]
            if len(uav):
                landed_uav.append(name)
                if not uav[0].landed:
                    pred_names.extend(['landed', 'airborne'])
                    params.extend([[KeyValue('v', uav[0].namespace)],
                                   [KeyValue('v', uav[0].namespace)]])
                    update_types.extend([
                        KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE,
                        KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE
                    ])
        for uav in self.uavs:
            if not (uav.namespace in landed_uav) and uav.landed:
                pred_names.extend(['landed', 'airborne'])
                params.extend([[KeyValue('v', uav.namespace)],
                               [KeyValue('v', uav.namespace)]])
                update_types.extend([
                    KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE,
                    KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE
                ])
        if pred_names != list():
            self.update_predicates(pred_names, params, update_types)

    def _arm_update(self):
        """
        Add or remove arm facts on ROSPlan knowledge base
        """
        armed_uav = list()
        params = list()
        pred_names = list()
        update_types = list()
        attributes = self._proposition_proxy('armed').attributes
        for attribute in attributes:
            name = attribute.values[0].value
            uav = [i for i in self.uavs if i.namespace == name]
            if len(uav):
                armed_uav.append(name)
                if not uav[0].state.armed:
                    pred_names.extend(['armed'])
                    params.extend([[KeyValue('v', uav[0].namespace)]])
                    update_types.extend([
                        KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE,
                    ])
        for uav in self.uavs:
            if not (uav.namespace in armed_uav) and uav.state.armed:
                pred_names.extend(['armed'])
                params.extend([[KeyValue('v', uav.namespace)]])
                update_types.extend([
                    KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE,
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
                if uav[0]._current_wp != -1:
                    # add current wp that uav resides
                    pred_names.append('at')
                    params.append([
                        KeyValue('v', uav[0].namespace),
                        KeyValue('wp', 'uav_wp%d' % uav[0]._current_wp)
                    ])
                    update_types.append(
                        KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
                    # remove previous wp that uav resided
                    if at != uav[0]._current_wp:
                        pred_names.append('at')
                        params.append([
                            KeyValue('v', uav[0].namespace),
                            KeyValue('wp', 'uav_wp%d' % at)
                        ])
                        update_types.append(
                            KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE)
                    # update visited state
                    pred_names.append('visited')
                    params.append(
                        [KeyValue('wp', 'uav_wp%d' % uav[0]._current_wp)])
                    update_types.append(
                        KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
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
                # update visited state
                pred_names.append('visited')
                params.append([KeyValue('wp', 'uav_wp%d' % uav._current_wp)])
                update_types.append(
                    KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        if pred_names != list():
            self.update_predicates(pred_names, params, update_types)

    def _battery_update(self):
        """
        Update battery value on ROSPlan knowledge base
        """
        for uav in self.uavs:
            # battery status update
            self.update_functions(
                ['battery-amount'], [[KeyValue('v', uav.namespace)]],
                [np.mean(uav.battery_voltages)], [
                    KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE,
                ])
            self.lowbat[uav.namespace] = uav.low_battery
