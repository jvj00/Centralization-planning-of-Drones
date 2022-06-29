(define (domain planner)
  (:requirements :strips :action-costs :typing :durative-actions)
  
  (:types
    drone point - object
  )

  (:predicates
    (link ?p1 - point ?p2 - point)
    (link_agv ?p1 - point ?p2 - point)
    (drone_pos ?d - drone ?p - point)
    (agv_pos ?p - point)
    (agv_has ?d - drone)
    (picture ?p - point)
    (busy ?p - point)
    (empty ?p - point)
  )
  
  (:functions
    (distance ?from - point ?to - point)
    (distance_agv ?from - point ?to - point)
    (cost)
  )

  (:durative-action move_drone
    :parameters (?d - drone ?from - point ?to - point)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
                  (drone_pos ?d ?from) 
                  (link ?from ?to)
                )
        )
        (at end (and
                  (empty ?to)
              )
        )
    )
    :effect (and 
        (at start (and
            (not (drone_pos ?d ?from))
            (empty ?from)
            (not (busy ?from))
        ))
        (at end (and 
            (drone_pos ?d ?to) 
            (busy ?to)
            (not (empty ?to))
            (increase (cost) (distance ?from ?to)) 
        ))
    )
  )

  (:durative-action take_picture
    :parameters (?d - drone ?p - point)
    :duration (= ?duration 0.001)
    :condition (and
              (at start (and
                    (drone_pos ?d ?p)
              ))
    )
    :effect (and
            (at end (picture ?p))
    )
  )

  (:durative-action takeoff
    :parameters (?d - drone ?pagv - point ?pdrone - point)
    :duration (= ?duration 0.3)
    :condition (and
              (at start (and
                    (agv_pos ?pagv)
                    (link ?pdrone ?pagv)
                    (agv_has ?d)
              ))
              (at end (and
                    (empty ?pdrone)
              ))
    )
    :effect (and
            (at start (and
                    (not (agv_has ?d))
            ))
            (at end (and
                  (drone_pos ?d ?pdrone)
                  (busy ?pdrone)
                  (not (empty ?pdrone))
                  (increase (cost) 0.1)
            ))
    )
  )

  (:durative-action move_agv
    :parameters (?from - point ?to - point)
    :duration (= ?duration 10)
    :condition (and
              (at start (and
                  (agv_pos ?from)
                  (link_agv ?from ?to)
              ))
    )
    :effect (and
            (at start
                  (not(agv_pos ?from))
            )
            (at end (and
                  (agv_pos ?to)
                  (increase (cost) (distance_agv ?from ?to)) 
            ))
    )
  )

)