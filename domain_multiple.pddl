(define (domain planner)
  (:requirements :strips :action-costs :typing)
  
  (:types
    drone point - object
  )

  (:predicates
    (link ?p1 - point ?p2 - point)
    (drone_pos ?d - drone ?p - point)
    (visited ?p - point)
  )
  
  (:functions
    (distance ?from - point ?to - point)
    (cost)
  )

  (:action move
    :parameters (?d - drone ?from - point ?to - point)
    :precondition (and 
                    (drone_pos ?d ?from) 
                    (link ?from ?to)
                  )
    :effect (and 
              (drone_pos ?d ?to) 
              (not (drone_pos ?d ?from)) 
              (visited ?to) 
              (increase (cost) (distance ?from ?to)) 
            )
  )
)