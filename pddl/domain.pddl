(define (domain mimree_mission)

    (:requirements 
        :strips
        :typing
        :equality
        :fluents
        :durative-actions
        :continuous-effects
		:universal-preconditions
    )

    (:types
        waypoint vehicle - object
        uav asv - vehicle
        uav_waypoint asv_waypoint - waypoint
    )

    (:functions
        (minimum-fuel ?v - vehicle)
        (fuel-percentage ?v - vehicle)
        (battery-amount ?v - vehicle)
        (minimum-battery ?v - vehicle)
    )

    (:predicates
        (carrier ?v - asv)
        (landed ?v - uav)
        (airborne ?v - uav)
        (armed ?v - vehicle)
        (home ?wp - waypoint)
        (takeoff ?wp - waypoint)
        (inspect ?wp - waypoint)
        (inspected ?wp - waypoint)
        (preflightchecked ?v - uav)
        (at ?v - vehicle ?wp - waypoint)
        (visited ?wp - waypoint)
        (connected ?wp1 - waypoint ?wp2 - waypoint)
        (tracked ?v - vehicle)
    )

    ;; ASV ACTIONS
    (:durative-action asv_request_arm
        :parameters (?v - asv)
        :duration (= ?duration 180)
        :condition (and
            (at start (> (fuel-percentage ?v) (minimum-fuel ?v)))
        )
        :effect (and (at end (armed ?v)))
    )

    (:durative-action asv_goto_waypoint
        :parameters (?v - asv ?from ?to - asv_waypoint)
        :duration (= ?duration 600)
        :condition (and
            (at start (at ?v ?from))
			(over all (forall (?drone - uav) (airborne ?drone)))
            (over all (connected ?from ?to))
            (over all (armed ?v))
        )
        :effect (and
            (at start (not (at ?v ?from)))
            (at end (at ?v ?to))
            (at end (visited ?to))
            (decrease (fuel-percentage ?v) (* 0.01 #t))
        )
    )

    (:durative-action asv_goto_waypoint_with_uav
        :parameters (?boat - asv ?drone - uav ?from ?to - asv_waypoint)
        :duration (= ?duration 600)
        :condition (and
            (at start (at ?boat ?from))
            (at start (at ?drone ?from))
            (over all (connected ?from ?to))
            (over all (armed ?boat))
            (over all (landed ?drone))
            (over all (carrier ?boat))
        )
        :effect (and
            (at start (not (at ?boat ?from)))
            (at start (not (at ?drone ?from)))
            (at end (at ?boat ?to))
            (at end (at ?drone ?to))
            (at end (visited ?to))
            (decrease (fuel-percentage ?boat) (* 0.01 #t))
        )
    )

    (:durative-action asv_rtl
        :parameters (?v - asv ?from ?to - asv_waypoint)
        :duration (= ?duration 540)
        :condition (and
            (at start (at ?v ?from))
            (over all (home ?to))
            (over all (armed ?v))
            (over all (connected ?from ?to))
        )
        :effect (and 
            (at end (at ?v ?to))
            (at end (visited ?to))
            (at start (not (at ?v ?from)))
            (decrease (fuel-percentage ?v) (* 0.01 #t))
        )
    )

    (:durative-action asv_rtl_with_uav
        :parameters (?boat - asv ?drone - uav ?from ?to - asv_waypoint)
        :duration (= ?duration 540)
        :condition (and
            (at start (at ?boat ?from))
            (at start (at ?drone ?from))
            (over all (landed ?drone))
            (over all (home ?to))
            (over all (armed ?boat))
            (over all (carrier ?boat))
            (over all (connected ?from ?to))
        )
        :effect (and 
            (at end (at ?boat ?to))
            (at end (at ?drone ?to))
            (at end (visited ?to))
            (at start (not (at ?boat ?from)))
            (at start (not (at ?drone ?from)))
            (decrease (fuel-percentage ?boat) (* 0.01 #t))
        )
    )

    ;; UAV ACTIONS
    (:durative-action uav_preflightcheck
        :parameters  (?v - uav ?wp - waypoint)
        :duration (= ?duration 180)
        :condition (and
            (over all (landed ?v))
            (over all (at ?v ?wp))
            (over all (takeoff ?wp))
        )
        :effect (and (at end (preflightchecked ?v)))
    )

    (:durative-action uav_request_arm
        :parameters (?v - uav ?wp - waypoint)
        :duration (= ?duration 180)
        :condition (and 
            (over all (at ?v ?wp))
            (over all (takeoff ?wp))
            (over all (landed ?v))
            (over all (preflightchecked ?v))
        )
        :effect (and (at end (armed ?v)))
    )   

    (:durative-action uav_takeoff
        :parameters (?v - uav ?from - waypoint)
        :duration (= ?duration 120)
        :condition (and 
            (at start (landed ?v))
            (over all (at ?v ?from))
            (over all (armed ?v))
            (over all (takeoff ?from))
            (over all (preflightchecked ?v))
        )
        :effect (and 
            (at start (not (landed ?v)))
            (at end (airborne ?v))
            (decrease (battery-amount ?v) (* 0.0001 #t))
        )
    )

    (:durative-action uav_goto_waypoint
        :parameters (?v - uav ?from - waypoint ?to - uav_waypoint)
        :duration (= ?duration 600)
        :condition (and
            (at start (at ?v ?from))
            (over all (armed ?v))
            (over all (airborne ?v))
            (over all (connected ?from ?to))
            (over all (preflightchecked ?v))
        )
        :effect (and
            (at start (not (at ?v ?from)))
            (at end (at ?v ?to))
            (at end (visited ?to))
            (decrease (battery-amount ?v) (* 0.0001 #t))
        )
    )

    (:durative-action uav_tracking
        :parameters (?v - uav ?vo - vehicle)
        :duration (= ?duration 600)
        :condition (and
            (over all (armed ?v))
            (over all (airborne ?v))
            (over all (preflightchecked ?v))
        )
        :effect (and
            (at end (tracked ?vo))
            (decrease (battery-amount ?v) (* 0.0001 #t))
        )
    )

    (:durative-action uav_inspect_blade
        :parameters (?v - uav ?from - waypoint)
        :duration (= ?duration 900)
        :condition (and
            (at start (at ?v ?from))
            (at start (> (battery-amount ?v) (minimum-battery ?v)))
            (over all (armed ?v))
            (over all (airborne ?v))
            (over all (inspect ?from))
            (over all (preflightchecked ?v))
        )
        :effect (and
            (at start (not (at ?v ?from)))
            (at end (at ?v ?from))
            (at end (inspected ?from))
            (decrease (battery-amount ?v) (* 0.0001 #t))
        )
    )

    (:durative-action uav_rtl
        :parameters (?v - uav ?from ?to - uav_waypoint)
        :duration (= ?duration 540)
        :condition (and
            (at start (airborne ?v))
            (at start (at ?v ?from))
            (over all (home ?to))
            (over all (armed ?v))
            (over all (connected ?from ?to))
            (over all (preflightchecked ?v))
        )
        :effect (and 
            (at end (landed ?v))
            (at end (at ?v ?to))
            (at end (visited ?to))
            (at start (not (at ?v ?from)))
            (at start (not (airborne ?v)))
            (decrease (battery-amount ?v) (* 0.0001 #t))
        )
    )
)
