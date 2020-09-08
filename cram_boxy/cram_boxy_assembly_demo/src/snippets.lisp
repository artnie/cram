(defun tare ()
  (boxy-ll::zero-wrench-sensor))

(defun g-open ()
  (with-giskard-controlled-robot
    (boxy-ll::move-gripper-joint :action-type-or-position :open :left-or-right :left)))

(defun g-close ()
  (with-giskard-controlled-robot
    (boxy-ll::move-gripper-joint :action-type-or-position :close :left-or-right :left)))

#+good-base-pose
(cl-tf:make-pose-stamped
  "map" 0.0
  (-2.6880709033373353d0 2.076750906430788d0 0.0d0)
  (0.0d0 0.0d0 -0.38311944677109644d0 0.9236988088688916d0))

#+bring-ee-into-operating-pose
(let* ((?pose (cl-tf:make-pose-stamped 
                     "base_footprint" 0.0 
                     (cl-tf:make-3d-vector 0.6389242467818544d0 0.9949709508577872d0 1.1983385610582362d0)
                     (cl-tf:make-quaternion 0.4196480192106036d0 0.9076867839317774d0 -2.377563681542072d-4 4.309525652191615d-4)))
             (?constraint '("odom_x_joint" 
                            "odom_y_joint"
                            ;; "odom_z_joint"
                            "triangle_base_joint"
                            "left_gripper_joint")))
        (with-giskard-controlled-robot
          (home-torso) ;; at -0.2
          (perform
           (a motion
              (type moving-tcp)
              (left-pose ?pose)
              (constraints ?constraint)))))

#+MOVING-TCP-with-base-to-chassis-over-holder
(let* ((?pose (cl-tf:make-pose-stamped 
                     "map" 0.0 
                     (cl-tf:make-3d-vector -1.3014965705262884d0 2.0017630857174618d0 1.2503926173201693d0)
                     (cl-tf:make-quaternion 0.7071067811865476d0 0.7071067811865475d0 0.0 0.0)))
             (?constraint '("left_gripper_joint")))
        (with-giskard-controlled-robot
          (perform
           (a motion
              (type moving-tcp)
              (left-pose ?pose)
              (constraints ?constraint)))))

#+GOING-somewhere
(with-giskard-controlled-robot
  (let* ((?nav-goal `((-2.8 1.0 0) (0 0 0 1)))
         (?pose (cl-transforms-stamped:pose->pose-stamped
                 cram-tf:*fixed-frame*
                 0.0
                 (btr:ensure-pose ?nav-goal))))
    (exe:perform
     (desig:an action
               (type going)
               (target (desig:a location
                                (pose ?pose)))))))

#+CHASSIS-ON-HOLDER-assembly-plan
(with-giskard-controlled-robot
  (let ((?object
          (exe:perform
           (desig:a motion
                    (type world-state-detecting)
                    (object (desig:an object (type :chassis))))))
        (?other-object
            (exe:perform
             (desig:a motion
                      (type world-state-detecting)
                      (object (desig:an object (type :holder-plane-horizontal))))))
        (?constraints '("triangle_base_joint" "odom_x_joint" "odom_y_joint" "odom_z_joint")))
    (exe:perform
     (desig:an action
                 (type assembling)
                 (arm left)
                 (object ?object)
                 (target (desig:a location
                                  (on ?other-object)
                                  (for ?object)
                                  (attachment :horizontal-attachment)))
                 (constraints ?constraints)))))

#+P1-CHASSIS-ON-HOLDER
(let* ((xo 0.0)
       (yo 0.0)
       (?pose (cl-tf:make-pose-stamped 
               "map" 0.0 
                (cl-tf:make-3d-vector 
                 -1.249474078373198d0 1.9502198955368117d0 1.0763608837966459d0)
                (cl-tf:make-quaternion 
                 0.9999981795850676d0 8.229773285967079d-4 0.0010119637044441272d0 0.001392646519700876d0)))
       (?retr (cram-tf:translate-pose 
               ?pose
               :x-offset xo
               :y-offset yo
               :z-offset 0.0))
       (?push (cram-tf:translate-pose 
               ?pose
               :x-offset xo
               :y-offset yo
               :z-offset -0.015))
       (?constraint '("odom_x_joint" 
                      "odom_y_joint"
                      "odom_z_joint" 
                      "triangle_base_joint"
                      "left_gripper_joint" )))
  (with-giskard-controlled-robot
    (perform
     (a motion
        (type moving-tcp)
        (left-pose ?retr)
        (constraints ?constraint)
        (COLLISION-MODE allow-all)))
    (boxy-ll::zero-wrench-sensor)
    (perform
     (a motion
        (type moving-tcp)
        (left-pose ?push)
        (constraints ?constraint)
        (COLLISION-MODE allow-all)))
    (break "Waiting for retract")
    (perform
     (a motion
        (type moving-tcp)
        (left-pose ?retr)
        (constraints ?constraint)
        (COLLISION-MODE allow-all)))
    (boxy-ll::zero-wrench-sensor)))

#+P2-BOTTOM_WING_ON_CHASSIS
(let* ((xo 0.0)
       (yo 0.0)
       (?pose (cl-tf:make-pose-stamped 
                "map" 0.0 
                (cl-tf:make-3d-vector 
                 -1.2794965705262884d0 2.0387630857174618d0 1.083926173201693d0)
                (cl-tf:make-quaternion 
                 -0.4920123344121077d0 -0.49965631707217406d0 0.5202339136815843d0 -0.4874670272457019d0)))
       (?retr (cram-tf:translate-pose 
               ?pose
               :x-offset xo
               :y-offset yo
               :z-offset -0.0))
       (?push (cram-tf:translate-pose 
               ?pose
               :x-offset xo
               :y-offset yo
               :z-offset -0.03))
       (?constraint '("odom_x_joint" 
                      "odom_y_joint"
                      "odom_z_joint" 
                      "triangle_base_joint"
                      "left_gripper_joint" )))
  (with-giskard-controlled-robot
    (perform
     (a motion
        (type moving-tcp)
        (left-pose ?retr)
        (constraints ?constraint)
        (COLLISION-MODE allow-all)))
    (boxy-ll::zero-wrench-sensor)
    (perform
     (a motion
        (type moving-tcp)
        (left-pose ?push)
        (constraints ?constraint)
        (COLLISION-MODE allow-all)))
    (break "Waiting for retract")
    (perform
     (a motion
        (type moving-tcp)
        (left-pose ?retr)
        (constraints ?constraint)
        (COLLISION-MODE allow-all)))
    (boxy-ll::zero-wrench-sensor)))

#+underb-on-bw-joint-state
(("left_arm_0_joint" "left_arm_1_joint" "left_arm_2_joint" "left_arm_3_joint"
  "left_arm_4_joint" "left_arm_5_joint" "left_arm_6_joint")
 (0.15639362415680255d0 -0.6006752430834015d0 -0.16731306719453742d0
  -1.406309471279534d0 0.3648613955500662d0 1.3833355982480102d0
                        -1.1898572754079715d0))

#+P3-UNDERBODY-ON-BOTTOMWING
(let* ((xo 0.0)
       (yo 0.0)
       (?pose (cl-tf:make-pose-stamped 
                "map" 0.0 
                (cl-tf:make-3d-vector 
                 -1.2909990475669384d0 1.9599989208515338d0 1.110199981657524d0)
                (cl-tf:make-quaternion 
                 0.7071069689450484d0 0.707106593427997d0 -4.605990059482497d-11 2.0230786829737605d-9)))
       (?retr (cram-tf:translate-pose 
               ?pose
               :x-offset xo
               :y-offset yo
               :z-offset -0.0))
       (?push (cram-tf:translate-pose 
               ?pose
               :x-offset xo
               :y-offset yo
               :z-offset -0.03))
       (?constraint '("odom_x_joint" 
                      "odom_y_joint"
                      "odom_z_joint" 
                      "triangle_base_joint"
                      "left_gripper_joint" )))
  (with-giskard-controlled-robot
    (perform
     (a motion
        (type moving-tcp)
        (left-pose ?retr)
        (constraints ?constraint)
        (COLLISION-MODE allow-all)))
    (boxy-ll::zero-wrench-sensor)
    (perform
     (a motion
        (type moving-tcp)
        (left-pose ?push)
        (constraints ?constraint)
        (COLLISION-MODE allow-all)))
    (break "Waiting for retract")
    (perform
     (a motion
        (type moving-tcp)
        (left-pose ?retr)
        (constraints ?constraint)
        (COLLISION-MODE allow-all)))
    (boxy-ll::zero-wrench-sensor)))

#+P4-UPPERBODY-ON-UNDERBODY-rotate-just-a-bit
(let* ((xo -0.02)
       (yo 0.0)
       (?pose (cl-tf:make-pose-stamped 
               "map" 0.0 
               (cl-tf:make-3d-vector 
                -1.350000396426472d0 1.949002728387602d0 1.088d0)
               (cl-tf:make-quaternion 
                0.6708824717752359d0 -0.7415636918463241d0 0.0d0 0.0d0)))
       (?retr (cram-tf:translate-pose 
               ?pose
               :x-offset xo
               :y-offset yo
               :z-offset -0.0))
       (?push (cram-tf:translate-pose 
               ?pose
               :x-offset xo
               :y-offset yo
               :z-offset -0.03))
       (?constraint '("odom_x_joint" 
                      "odom_y_joint"
                      "odom_z_joint"
                      "triangle_base_joint"
                      "left_gripper_joint" )))
  (with-giskard-controlled-robot
    (perform
     (a motion
        (type moving-tcp)
        (left-pose ?retr)
        (constraints ?constraint)
        (COLLISION-MODE allow-all)))
    (boxy-ll::zero-wrench-sensor)
    (perform
     (a motion
        (type moving-tcp)
        (left-pose ?push)
        (constraints ?constraint)
        (COLLISION-MODE allow-all)))
    (break "Waiting for retract")
    (perform
     (a motion
        (type moving-tcp)
        (left-pose ?retr)
        (constraints ?constraint)
        (COLLISION-MODE allow-all)))
    (boxy-ll::zero-wrench-sensor)))

#+P5-BOLT-IN-REAR-UPPERBODY
(let* ((xo 0.0)
       (yo 0.0)
       (?pose (cl-tf:make-pose-stamped 
                "map" 0.0 
                (cl-tf:make-3d-vector 
                 -1.3685446144091171d0 1.9500139897951687d0 1.1606120733034709d0)
                (cl-tf:make-quaternion 
                 0.8660256905066502d0 -0.4999995033552162d0 4.952901774606716d-6 1.57676586885885d-6)))
       (?retr (cram-tf:translate-pose 
               ?pose
               :x-offset xo
               :y-offset yo
               :z-offset -0.0))
       (?push (cram-tf:translate-pose 
               ?pose
               :x-offset xo
               :y-offset yo
               :z-offset -0.03))
       (?constraint '("odom_x_joint" 
                      "odom_y_joint"
                      "odom_z_joint" 
                      "triangle_base_joint"
                      "left_gripper_joint" )))
  (with-giskard-controlled-robot
    (perform
     (a motion
        (type moving-tcp)
        (left-pose ?retr)
        (constraints ?constraint)
        (COLLISION-MODE allow-all)))
    (boxy-ll::zero-wrench-sensor)
    (perform
     (a motion
        (type moving-tcp)
        (left-pose ?push)
        (constraints ?constraint)
        (COLLISION-MODE allow-all)))
    (break "Waiting for retract")
    (perform
     (a motion
        (type moving-tcp)
        (left-pose ?retr)
        (constraints ?constraint)
        (COLLISION-MODE allow-all)))
    (boxy-ll::zero-wrench-sensor)))

#+touching-js
'(("left_arm_0_joint" -0.9119245409965515d0)
  ("left_arm_1_joint" 0.6476925611495972d0)
  ("left_arm_2_joint"  1.1286091804504395d0)
  ("left_arm_3_joint" -1.3791991472244263d0)
  ("left_arm_4_joint" -0.026325132697820663d0)
  ("left_arm_5_joint" 0.5115517377853394d0)
  ("left_arm_6_joint" -1.454879879951477d0))

#+TOUCH-board-front
(let* ((?constraint '("left_gripper_joint" "triangle_base_joint")))
        (multiple-value-bind (?pose ?dir) (touch-trajectory :big-wooden-plate :from :front :offset '(0 0.3 0))
           (viz-debug-pose ?pose)
          (with-giskard-controlled-robot
            (let ((?object (exe:perform
                            (desig:a motion
                                     (type world-state-detecting)
                                     (object (desig:an object (type :big-wooden-plate)))))))
              
              (touch :object ?object
                     :arm :left
                     :pose ?pose
                     :direction ?dir)
              ))))

#+TOUCH-board-left
(let* ((?constraint '("left_gripper_joint" "triangle_base_joint")))
        (multiple-value-bind (?pose ?dir) (touch-trajectory :big-wooden-plate :from :left :offset '(-0.33 0 0.025))
           (viz-debug-pose ?pose)
          (with-giskard-controlled-robot
            (let ((?object (exe:perform
                            (desig:a motion
                                     (type world-state-detecting)
                                     (object (desig:an object (type :big-wooden-plate)))))))
              
              (touch :object ?object
                     :arm :left
                     :pose ?pose
                     :direction ?dir)
              ))))


#+TOUCH-board-top
(let* ((?constraint '("left_gripper_joint" "triangle_base_joint")))
        (multiple-value-bind (?pose ?dir) (touch-trajectory :big-wooden-plate :from :top :offset '(-0.3 0 0))
           (viz-debug-pose ?pose)
          (with-giskard-controlled-robot
            (let ((?object (exe:perform
                            (desig:a motion
                                     (type world-state-detecting)
                                     (object (desig:an object (type :big-wooden-plate)))))))
              
              (touch :object ?object
                     :arm :left
                     :pose ?pose
                     :direction ?dir)
              ))))




#+SCREWING-example
(with-giskard-controlled-robot
  (let* ((?other-nav-goal *base-left-side-left-hand-pose*) 
         (?object (exe:perform 
                   (desig:a motion
                            (type world-state-detecting)
                            (object (desig:an object (name :bolt-1))))))
         (?other-object
           (go-perceive :upper-body ?other-nav-goal)))
    (exe:perform
     (desig:an action
               (type screwing)
               (arm left)
               (object ?object)
               (target (desig:a location
                                (on ?other-object)
                                (for ?object)
                                (attachment :rear-thread)))))))
