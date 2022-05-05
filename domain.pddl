(define (domain planner)
  (:requirements :strips :action-costs :typing :durative-actions)
  
  (:types
    drone point direction - object
  )

  (:predicates
    (link ?p1 - point ?p2 - point)
    (rotation ?dir_from - direction ?dir_to - direction)
    (rotation_point ?p - point)
    (drone_pos ?d - drone ?p - point)
    (drone_dir ?d - drone ?dir - direction)
    (picture ?p - point ?dir - direction)
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

  (:durative-action rotate_45
    :parameters (?d - drone ?rp - point ?dir_from - direction ?dir_to - direction)
    :duration (= ?duration 0.1)
    :condition  (and
            (at start (and
              (drone_pos ?d ?rp)
              (rotation_point ?rp)
              (drone_dir ?d ?dir_from)
              (rotation ?dir_from ?dir_to)
            ))
    )
    :effect (and
            (at start (not (drone_dir ?d ?dir_from)))
            (at end (and 
              (drone_dir ?d ?dir_to) 
              (increase (cost) 0.1)
            ))
    )
  )

  (:durative-action take_picture
    :parameters (?d - drone ?p - point ?dir - direction)
    :duration (= ?duration 0.001)
    :condition (and
              (at start (and
                    (drone_pos ?d ?p)
                    (drone_dir ?d ?dir)
              ))
    )
    :effect (and
            (at end (picture ?p ?dir))
    )
  )
  
)