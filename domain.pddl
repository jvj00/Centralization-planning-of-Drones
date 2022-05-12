(define (domain planner)
  (:requirements :strips :action-costs :typing :durative-actions)
  
  (:types
    drone point - object
  )

  (:predicates
    (link ?p1 - point ?p2 - point)
    (drone_pos ?d - drone ?p - point)
    (picture ?p - point)
  )
  
  (:functions
    (distance ?from - point ?to - point)
    (cost)
  )

  (:durative-action move
    :parameters (?d - drone ?from - point ?to - point)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
                  (drone_pos ?d ?from) 
                  (link ?from ?to)
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
              ))
    )
    :effect (and
            (at end (picture ?p))
    )
  )
  
)