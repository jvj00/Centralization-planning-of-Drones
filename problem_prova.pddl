(define (problem planning_map)
  (:domain planner)
  (:objects
     p0 p1 pagv0 pagv1  - point
     d1 d2 - drone
  )
  
  (:init
    (= (cost) 0)
    (agv_pos pagv0)

    (empty p0)
    (empty p1)

    (link p0 p1)
    (link p1 p0)
    (link p0 pagv0)
    (link p1 pagv1)
    (= (distance p0 p1) 10.000000)
    (= (distance p1 p0) 10.000000)
    (= (distance p0 pagv0) 30.000000)
    (= (distance p1 pagv1) 30.000000)
    (link_agv pagv0 pagv1)
    (link_agv pagv1 pagv0)
    (= (distance_agv pagv0 pagv1) 10.000000)
    (= (distance_agv pagv1 pagv0) 10.000000)
  )
  (:goal
    (and 
      (picture p1)
    )
  )
  (:metric minimize
    (cost)
  )
)
