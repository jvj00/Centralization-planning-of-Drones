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
    (agv_hasnot ?d - drone)
    (agv_has ?d - drone)
    (picture ?p - point)
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
                  (agv_hasnot ?d)
                )
        )
    )
    :effect (and 
        (at start (not (drone_pos ?d ?from)) )
        (at end (and 
            (drone_pos ?d ?to) 
            (increase (cost) (distance ?from ?to)) 
          )
        )
    )
  )

  (:durative-action take_picture
    :parameters (?d - drone ?p - point)
    :duration (= ?duration 0.001)
    :condition (and
              (at start (and
                    (drone_pos ?d ?p)
                    (agv_hasnot ?d)
              ))
    )
    :effect (and
            (at end (picture ?p))
    )
  )

  (:durative-action landing
    :parameters (?d - drone ?p - point)
    :duration (= ?duration 0.3)
    :condition (and
              (at start (and
                    (drone_pos ?d ?p)
                    (agv_pos ?p)
                    (agv_hasnot ?d)
              ))
    )
    :effect (and
            (at end (and
                  (agv_has ?d)
                  (not(agv_hasnot ?d))
                  (increase (cost) 0.1) 
            ))
    )
  )

  (:durative-action takeoff
    :parameters (?d - drone ?p - point)
    :duration (= ?duration 0.3)
    :condition (and
              (at start (and
                    (agv_pos ?p)
                    (agv_has ?d)
              ))
    )
    :effect (and
            (at end (and
                  (agv_hasnot ?d)
                  (not(agv_has ?d))
                  (drone_pos ?d ?p)
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