(define (domain om_wind_turbines)

    (:requirements 
        :strips
        :typing
        :equality
        :fluents
        :durative-actions
        :continuous-effects
        :duration-inequalities
		:universal-preconditions
        :disjunctive-preconditions
    )

    (:types
        waypoint vehicle - object
        uav asv irr - vehicle
    )

    (:predicates
        ;; ==================== Gen Predicates ====================
        (idle ?v - vehicle)
        (inspect_post ?wp - waypoint)
        (at ?v - vehicle ?wp - waypoint)
        (connected ?wp1 ?wp2 - waypoint)
        (turbine_inspected_at ?wp - waypoint)
        ;; ==================== USV Predicates ====================
        (has_lr_camera ?v - asv)
        (has_uav ?asv - asv ?uav - uav)
        (has_charging_dock ?asv - asv)
        ;; ==================== UAV Predicates ====================
        (ground ?v - uav)
        (airborne ?v - uav)
        (has_camera ?v - uav)
        (has_deploy_system ?v - uav)
        (landing_post ?wp - waypoint)
        (takeoff_post ?wp - waypoint)
        (charging_post ?wp - waypoint)
        (has_retrieval_system ?v - uav)
        (tookoff_from ?v - uav ?wp - waypoint)
        ;; ==================== IRR Predicates ====================
        (detached ?v - irr)
        (attached_to ?irr - irr ?uav - uav)
        (ndt_post ?wp - waypoint)
        (has_ndt_system ?v - irr)
        (has_repair_arm ?v - irr)
        (repair_post ?wp - waypoint)
        (turbine_repaired_at ?wp - waypoint)
        (turbine_nd_tested_at ?wp - waypoint)
        (deploy_retrieve_post ?wp - waypoint)
    )

    (:functions
        ;; ==================== Gen Functions ====================
        (fuel ?v - vehicle)
        (min_fuel ?v - vehicle)
        (max_fuel ?v - vehicle)
        (min_dur ?wp1 ?wp2 - waypoint)
        (max_dur ?wp1 ?wp2 - waypoint)
        (consumption_rate ?v - vehicle)
    )

    ;; ==================== ASV Actions ====================
    (:durative-action asv_navigate
        :parameters (?v - asv ?from ?to - waypoint)
        :duration (and (>= ?duration (min_dur ?from ?to)) (<= ?duration (max_dur ?from ?to)))
        :condition (and
            (at start (at ?v ?from))
            (over all (connected ?from ?to))
            (at start (> (fuel ?v) (min_fuel ?v)))
        )
        :effect (and
            (at end (at ?v ?to))
            (at start (not (at ?v ?from)))
            (decrease (fuel ?v) (* (consumption_rate ?v) #t))
        )
    )

    (:durative-action asv_inspect_wt
        :parameters (?v - asv ?wp - waypoint)
        :duration (and (>= ?duration (min_dur ?wp ?wp)) (<= ?duration (max_dur ?wp ?wp)))
        :condition (and
            (over all (at ?v ?wp))
            (over all (inspect_post ?wp))
            (over all (has_lr_camera ?v))
        )
        :effect (and
            (at end (turbine_inspected_at ?wp))
        )
    )

    ;; ==================== UAV Actions ====================
    (:durative-action uav_takeoff
        :parameters (?v - uav ?wp - waypoint)
        :duration (and (>= ?duration (min_dur ?wp ?wp)) (<= ?duration (max_dur ?wp ?wp)))
        :condition (and 
            (at start (idle ?v))
            (at start (ground ?v))
            (over all (takeoff_post ?wp))
            (at start (> (fuel ?v) (min_fuel ?v)))
            (over all (forall (?asv - asv) (imply (has_uav ?asv ?v) (at ?asv ?wp))))
        )
        :effect (and 
            (at end (idle ?v))
            (at end (airborne ?v))
            (at start (not (idle ?v)))
            (at start (not (ground ?v)))
            (at end (tookoff_from ?v ?wp))
            (decrease (fuel ?v) (* (consumption_rate ?v) #t))
        )
    )

    (:durative-action uav_land
        :parameters (?v - uav ?from ?to - waypoint) 
        :duration (and (>= ?duration (min_dur ?from ?from)) (<= ?duration (max_dur ?from ?from)))
        :condition (and
            (at start (idle ?v))
            (at start (airborne ?v))
            (over all (at ?v ?from))
            (over all (landing_post ?from))
            (at start (tookoff_from ?v ?to))
            (at start (> (fuel ?v) (min_fuel ?v)))
            (over all (forall (?asv - asv) (imply (has_uav ?asv ?v) (at ?asv ?to))))
        )
        :effect (and
            (at end (idle ?v))
            (at end (ground ?v))
            (at start (not (idle ?v)))
            (at start (not (airborne ?v)))
            (at start (not (tookoff_from ?v ?to)))
            (decrease (fuel ?v) (* (consumption_rate ?v) #t))
        )
    )

    (:durative-action uav_navigate
        :parameters (?v - uav ?from ?to ?takeoff - waypoint)
        :duration (and (>= ?duration (min_dur ?from ?to)) (<= ?duration (max_dur ?from ?to)))
        :condition (and
            (at start (idle ?v))
            (at start (at ?v ?from))
            (over all (airborne ?v))
            (over all (connected ?from ?to))
            (over all (connected ?takeoff ?to))
            (over all (tookoff_from ?v ?takeoff))
            (at start (> (fuel ?v) (min_fuel ?v)))
        )
        :effect (and
            (at end (idle ?v))
            (at end (at ?v ?to))
            (at start (not (idle ?v)))
            (at start (not (at ?v ?from)))
            (decrease (fuel ?v) (* (consumption_rate ?v) #t))
        )
    )

    (:durative-action uav_inspect_blade
        :parameters (?v - uav ?wp - waypoint)
        :duration (and (>= ?duration (min_dur ?wp ?wp)) (<= ?duration (max_dur ?wp ?wp)))
        :condition (and
            (at start (idle ?v))
            (over all (at ?v ?wp))
            (over all (airborne ?v))
            (over all (has_camera ?v))
            (over all (inspect_post ?wp))
            (at start (> (fuel ?v) (min_fuel ?v)))
        )
        :effect (and
            (at end (idle ?v))
            (at start (not (idle ?v)))
            (at end (turbine_inspected_at ?wp))
            (decrease (fuel ?v) (* (consumption_rate ?v) #t))
        )
    )

    (:durative-action uav_deploy_irr
        :parameters (?uav - uav ?irr - irr ?uav_wp ?irr_wp - waypoint)
        :duration (and (>= ?duration (min_dur ?uav_wp ?uav_wp)) (<= ?duration (max_dur ?uav_wp ?uav_wp)))
        :condition (and
            (at start (idle ?uav))
            (over all (idle ?irr))
            (over all (airborne ?uav))
            (over all (at ?uav ?uav_wp))
            (at start (attached_to ?irr ?uav))
            (over all (has_deploy_system ?uav))
            (over all (deploy_retrieve_post ?uav_wp))
            (over all (deploy_retrieve_post ?irr_wp))
            (at start (> (fuel ?uav) (min_fuel ?uav)))
        )
        :effect (and
            (at end (idle ?uav))
            (at end (detached ?irr))
            (at end (at ?irr ?irr_wp))
            (at start (not (idle ?uav)))
            (at start (not (attached_to ?irr ?uav)))
            (decrease (fuel ?uav) (* (consumption_rate ?uav) #t))
        )
    )

    (:durative-action uav_retrieve_irr
        :parameters (?uav - uav ?irr - irr ?uav_wp ?irr_wp - waypoint)
        :duration (and (>= ?duration (min_dur ?uav_wp ?uav_wp)) (<= ?duration (max_dur ?uav_wp ?uav_wp)))
        :condition (and
            (at start (idle ?uav))
            (over all (idle ?irr))
            (at start (detached ?irr))
            (over all (airborne ?uav))
            (over all (at ?uav ?uav_wp))
            (at start (at ?irr ?irr_wp))
            (over all (has_retrieval_system ?uav))
            (over all (deploy_retrieve_post ?uav_wp))
            (over all (deploy_retrieve_post ?irr_wp))
            (at start (> (fuel ?uav) (min_fuel ?uav)))
        )
        :effect (and
            (at end (idle ?uav))
            (at start (not (idle ?uav)))
            (at end (attached_to ?irr ?uav))
            (at start (not (detached ?irr)))
            (at start (not (at ?irr ?irr_wp)))
            (decrease (fuel ?uav) (* (consumption_rate ?uav) #t))
        )
    )

    (:durative-action uav_refuelling
        :parameters (?uav - uav ?asv - asv ?wp - waypoint)
        :duration (= ?duration (/ (- (max_fuel ?uav) (fuel ?uav)) (* 10 (consumption_rate ?uav))))
        :condition (and
            (at start (idle ?uav))
            (over all (ground ?uav))
            (over all (at ?uav ?wp))
            (over all (charging_post ?wp))
            (over all (has_uav ?asv ?uav))
            (over all (has_charging_dock ?asv))
            (at start (< (fuel ?uav) (max_fuel ?uav)))
        )
        :effect (and
            (at end (idle ?uav))
            (at start (not (idle ?uav)))
            (at end (assign (fuel ?uav) (max_fuel ?uav))) 
        )
    )

    (:durative-action uav_refuelling_home
        :parameters (?uav - uav ?wp - waypoint)
        :duration (= ?duration (/ (- (max_fuel ?uav) (fuel ?uav)) (* 10 (consumption_rate ?uav))))
        :condition (and
            (at start (idle ?uav))
            (over all (ground ?uav))
            (over all (at ?uav ?wp))
            (over all (charging_post ?wp))
            (at start (< (fuel ?uav) (max_fuel ?uav)))
        )
        :effect (and
            (at end (idle ?uav))
            (at start (not (idle ?uav)))
            (at end (assign (fuel ?uav) (max_fuel ?uav))) 
        )
    )
        
    ;; ==================== IRR Actions ====================
    (:durative-action irr_navigate
        :parameters (?v - irr ?from ?to - waypoint)
        :duration (and (>= ?duration (min_dur ?from ?to)) (<= ?duration (max_dur ?from ?to)))
        :condition (and
            (at start (at ?v ?from))
            (over all (connected ?from ?to))
            (at start (> (fuel ?v) (min_fuel ?v)))
        )
        :effect (and
            (at end (at ?v ?to))
            (at start (not (at ?v ?from)))
            (decrease (fuel ?v) (* (consumption_rate ?v) #t))
        )
    )

    (:durative-action irr_ndt_inspect
        :parameters (?v - irr ?wp - waypoint)
        :duration (and (>= ?duration (min_dur ?wp ?wp)) (<= ?duration (max_dur ?wp ?wp)))
        :condition (and
            (at start (idle ?v))
            (over all (at ?v ?wp))
            (over all (ndt_post ?wp))
            (over all (has_ndt_system ?v))
            (at start (> (fuel ?v) (min_fuel ?v)))
        )
        :effect (and
            (at end (idle ?v))
            (at start (not (idle ?v)))
            (at end (turbine_nd_tested_at ?wp))
            (decrease (fuel ?v) (* (consumption_rate ?v) #t))
        )
    )

    (:durative-action irr_repair_wt
        :parameters (?v - irr ?wp - waypoint)
        :duration (and (>= ?duration (min_dur ?wp ?wp)) (<= ?duration (max_dur ?wp ?wp)))
        :condition (and
            (at start (idle ?v))
            (over all (at ?v ?wp))
            (over all (repair_post ?wp))
            (over all (has_repair_arm ?v))
            (at start (> (fuel ?v) (min_fuel ?v)))
        )
        :effect (and
            (at end (idle ?v))
            (at start (not (idle ?v)))
            (at end (turbine_repaired_at ?wp))
            (decrease (fuel ?v) (* (consumption_rate ?v) #t))
        )
    )

)
