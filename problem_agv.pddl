(define (problem planning_map)
  (:domain planner)
  (:objects
     p0 p1 p2 p3 - point
     d1 d2 - drone
     mir - agv
  )
  
  (:init
    (= (cost) 0)
    (drone_pos d1 p0)
    (drone_pos d2 p0)
    (agv_pos p0)
    (agv_has d1)
    (agv_has d2)

    (link_agv p0 p1)
    (link p0 p1)
    (link p1 p2)
    (link p2 p3)
    (= (distance p0 p1) 1)
    (= (distance p1 p2) 1)
    (= (distance p2 p3) 1)
    (= (distance_agv p0 p1) 0.5)
  )
  (:goal
    (and 
      (picture p3)
    )
  )
  (:metric minimize
    (cost)
  )
)
