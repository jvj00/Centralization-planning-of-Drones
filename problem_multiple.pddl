(define (problem planning_map)
  (:domain planner)
  (:objects
     p0 p1 p2 p3 p4 p5 p6 p7 p8 p9 p10 p11 p12 p13 p14 p15  - point
     d1 d2 - drone
     dir0 dir45 dir90 dir135 dir180 dir225 dir270 dir315 - direction
  )
  
  (:init
    (= (cost) 0)
    (drone_pos d1 p0)
    (drone_pos d2 p15)
    (drone_dir d1 dir0)
    (drone_dir d2 dir0)

    (rotation dir0 dir45)
    (rotation dir45 dir0)
    (rotation dir45 dir90)
    (rotation dir90 dir45)
    (rotation dir90 dir135)
    (rotation dir135 dir90)
    (rotation dir135 dir180)
    (rotation dir180 dir135)
    (rotation dir180 dir225)
    (rotation dir225 dir180)
    (rotation dir225 dir270)
    (rotation dir270 dir225)
    (rotation dir270 dir315)
    (rotation dir315 dir270)
    (rotation dir315 dir0)
    (rotation dir0 dir315)
    (link p0 p1)
    (link p0 p5)
    (link p0 p4)
    (link p1 p2)
    (link p1 p6)
    (link p1 p5)
    (link p1 p4)
    (link p1 p0)
    (link p2 p3)
    (link p2 p7)
    (link p2 p6)
    (link p2 p5)
    (link p2 p1)
    (link p3 p7)
    (link p3 p6)
    (link p3 p2)
    (link p4 p0)
    (link p4 p1)
    (link p4 p5)
    (link p4 p9)
    (link p4 p8)
    (link p5 p0)
    (link p5 p1)
    (link p5 p2)
    (link p5 p6)
    (link p5 p10)
    (link p5 p9)
    (link p5 p8)
    (link p5 p4)
    (link p6 p1)
    (link p6 p2)
    (link p6 p3)
    (link p6 p7)
    (link p6 p11)
    (link p6 p10)
    (link p6 p9)
    (link p6 p5)
    (link p7 p2)
    (link p7 p3)
    (link p7 p11)
    (link p7 p10)
    (link p7 p6)
    (link p8 p4)
    (link p8 p5)
    (link p8 p9)
    (link p8 p13)
    (link p8 p12)
    (link p9 p4)
    (link p9 p5)
    (link p9 p6)
    (link p9 p10)
    (link p9 p14)
    (link p9 p13)
    (link p9 p12)
    (link p9 p8)
    (link p10 p5)
    (link p10 p6)
    (link p10 p7)
    (link p10 p11)
    (link p10 p15)
    (link p10 p14)
    (link p10 p13)
    (link p10 p9)
    (link p11 p6)
    (link p11 p7)
    (link p11 p15)
    (link p11 p14)
    (link p11 p10)
    (link p12 p8)
    (link p12 p9)
    (link p12 p13)
    (link p13 p8)
    (link p13 p9)
    (link p13 p10)
    (link p13 p14)
    (link p13 p12)
    (link p14 p9)
    (link p14 p10)
    (link p14 p11)
    (link p14 p15)
    (link p14 p13)
    (link p15 p10)
    (link p15 p11)
    (link p15 p14)
    (= (distance p0 p1) 1.000000)
    (= (distance p0 p5) 1.414200)
    (= (distance p0 p4) 1.000000)
    (= (distance p1 p2) 1.000000)
    (= (distance p1 p6) 1.414200)
    (= (distance p1 p5) 1.000000)
    (= (distance p1 p4) 1.414200)
    (= (distance p1 p0) 1.000000)
    (= (distance p2 p3) 1.000000)
    (= (distance p2 p7) 1.414200)
    (= (distance p2 p6) 1.000000)
    (= (distance p2 p5) 1.414200)
    (= (distance p2 p1) 1.000000)
    (= (distance p3 p7) 1.000000)
    (= (distance p3 p6) 1.414200)
    (= (distance p3 p2) 1.000000)
    (= (distance p4 p0) 1.000000)
    (= (distance p4 p1) 1.414200)
    (= (distance p4 p5) 1.000000)
    (= (distance p4 p9) 1.414200)
    (= (distance p4 p8) 1.000000)
    (= (distance p5 p0) 1.414200)
    (= (distance p5 p1) 1.000000)
    (= (distance p5 p2) 1.414200)
    (= (distance p5 p6) 1.000000)
    (= (distance p5 p10) 1.414200)
    (= (distance p5 p9) 1.000000)
    (= (distance p5 p8) 1.414200)
    (= (distance p5 p4) 1.000000)
    (= (distance p6 p1) 1.414200)
    (= (distance p6 p2) 1.000000)
    (= (distance p6 p3) 1.414200)
    (= (distance p6 p7) 1.000000)
    (= (distance p6 p11) 1.414200)
    (= (distance p6 p10) 1.000000)
    (= (distance p6 p9) 1.414200)
    (= (distance p6 p5) 1.000000)
    (= (distance p7 p2) 1.414200)
    (= (distance p7 p3) 1.000000)
    (= (distance p7 p11) 1.000000)
    (= (distance p7 p10) 1.414200)
    (= (distance p7 p6) 1.000000)
    (= (distance p8 p4) 1.000000)
    (= (distance p8 p5) 1.414200)
    (= (distance p8 p9) 1.000000)
    (= (distance p8 p13) 1.414200)
    (= (distance p8 p12) 1.000000)
    (= (distance p9 p4) 1.414200)
    (= (distance p9 p5) 1.000000)
    (= (distance p9 p6) 1.414200)
    (= (distance p9 p10) 1.000000)
    (= (distance p9 p14) 1.414200)
    (= (distance p9 p13) 1.000000)
    (= (distance p9 p12) 1.414200)
    (= (distance p9 p8) 1.000000)
    (= (distance p10 p5) 1.414200)
    (= (distance p10 p6) 1.000000)
    (= (distance p10 p7) 1.414200)
    (= (distance p10 p11) 1.000000)
    (= (distance p10 p15) 1.414200)
    (= (distance p10 p14) 1.000000)
    (= (distance p10 p13) 1.414200)
    (= (distance p10 p9) 1.000000)
    (= (distance p11 p6) 1.414200)
    (= (distance p11 p7) 1.000000)
    (= (distance p11 p15) 1.000000)
    (= (distance p11 p14) 1.414200)
    (= (distance p11 p10) 1.000000)
    (= (distance p12 p8) 1.000000)
    (= (distance p12 p9) 1.414200)
    (= (distance p12 p13) 1.000000)
    (= (distance p13 p8) 1.414200)
    (= (distance p13 p9) 1.000000)
    (= (distance p13 p10) 1.414200)
    (= (distance p13 p14) 1.000000)
    (= (distance p13 p12) 1.000000)
    (= (distance p14 p9) 1.414200)
    (= (distance p14 p10) 1.000000)
    (= (distance p14 p11) 1.414200)
    (= (distance p14 p15) 1.000000)
    (= (distance p14 p13) 1.000000)
    (= (distance p15 p10) 1.414200)
    (= (distance p15 p11) 1.000000)
    (= (distance p15 p14) 1.000000)
  )
  (:goal
    (and (picture p8 dir270) (picture p9 dir180) )
  )
  (:metric minimize
    (+ (total-time) (cost))
  )
)
