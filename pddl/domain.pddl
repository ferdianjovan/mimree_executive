(define (domain mimree_mission)

    (:requirements 
        :strips
        :typing
        :equality
        :fluents
        :durative-actions
        :continuous-effects
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
        (guided ?v - uav)
        (landed ?v - uav)
        (airborne ?v - uav)
        (armed ?v - vehicle)
        (home ?wp - waypoint)
        (takeoff ?wp - waypoint)
        (preflightchecked ?v - uav)
        (at ?v - vehicle ?wp - waypoint)
        (visited ?wp - waypoint)
        (connected ?wp1 - waypoint ?wp2 - waypoint)
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
            (at start (> (fuel-percentage ?v) (minimum-fuel ?v)))
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
            (at start (> (fuel-percentage ?boat) (minimum-fuel ?boat)))
            (over all (connected ?from ?to))
            (over all (armed ?boat))
            (over all (landed ?drone))
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
        :duration (= ?duration 420)
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
        :duration (= ?duration 420)
        :condition (and
            (at start (at ?boat ?from))
            (at start (at ?drone ?from))
            (over all (landed ?drone))
            (over all (home ?to))
            (over all (armed ?boat))
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

    (:durative-action asv_lowbat_return
        :parameters (?v - asv ?from ?to - asv_waypoint)
        :duration (= ?duration 420)
        :condition (and
            (over all (connected ?from ?to))
            (over all (home ?to))
            (over all (armed ?v))
            (at start (at ?v ?from))
            (at start (<= (fuel-percentage ?v) (minimum-fuel ?v)))
        )
        :effect (and 
            (at end (at ?v ?to))
            (at end (visited ?to))
            (at start (not (at ?v ?from)))
            (decrease (fuel-percentage ?v) (* 0.01 #t))
        )
    )

    (:durative-action asv_lowbat_return_with_uav
        :parameters (?boat - asv ?drone - uav ?from ?to - asv_waypoint)
        :duration (= ?duration 420)
        :condition (and
            (over all (connected ?from ?to))
            (over all (landed ?drone))
            (over all (home ?to))
            (over all (armed ?boat))
            (at start (at ?boat ?from))
            (at start (at ?drone ?from))
            (at start (<= (fuel-percentage ?boat) (minimum-fuel ?boat)))
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

    (:durative-action uav_guide
        :parameters (?v - uav ?wp - waypoint)
        :duration (= ?duration 60)
        :condition (and 
            (over all (at ?v ?wp))
            (over all (takeoff ?wp))
            (over all (preflightchecked ?v))
        )
        :effect (and (at end (guided ?v)))
    )

    (:durative-action uav_request_arm
        :parameters (?v - uav ?wp - waypoint)
        :duration (= ?duration 180)
        :condition (and 
            (over all (at ?v ?wp))
            (over all (takeoff ?wp))
            (at start (guided ?v))
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
            (at start (> (battery-amount ?v) (minimum-battery ?v)))
            (over all (at ?v ?from))
            (over all (armed ?v))
            (over all (guided ?v))
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
        :duration (= ?duration 300)
        :condition (and
            (at start (at ?v ?from))
            (at start (> (battery-amount ?v) (minimum-battery ?v)))
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

    (:durative-action uav_rtl
        :parameters (?v - uav ?from ?to - uav_waypoint)
        :duration (= ?duration 420)
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

    (:durative-action uav_lowbat_return
        :parameters (?v - uav ?from ?to - uav_waypoint)
        :duration (= ?duration 420)
        :condition (and
            (over all (home ?to))
            (over all (armed ?v))
            (at start (airborne ?v))
            (at start (at ?v ?from))
            (over all (connected ?from ?to))
            (at start (<= (battery-amount ?v) (minimum-battery ?v)))
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
