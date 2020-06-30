(defun tare ()
  (boxy-ll::zero-wrench-sensor))

(defun g-open ()
  (with-giskard-controlled-robot
    (boxy-ll::move-gripper-joint :action-type-or-position :open :left-or-right :left)))

(defun g-close ()
  (with-giskard-controlled-robot
    (boxy-ll::move-gripper-joint :action-type-or-position :close :left-or-right :left)))

#+MOVING-TCP-somewhere
(let* ((?pose (cl-tf:make-pose-stamped 
                     "map" 0.0 
                     (cl-tf:make-3d-vector -1.574166711171468d0 1.6d0 1.066000722249348d0)
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

#+PUSH-AND-RETRACT-bw-on-chassis
(let* ((xo 0.0)
       (yo 0.0)
       (?pose (cl-tf:make-pose-stamped 
                "map" 0.0 
                (cl-tf:make-3d-vector 
                 -1.3014965705262884d0 2.0017630857174618d0 1.1503926173201693d0)
                (cl-tf:make-quaternion 
                 -0.4920123344121077d0 -0.49965631707217406d0 0.5202339136815843d0 -0.4874670272457019d0)))
       (?retr (cram-tf:translate-pose 
               ?pose
               :x-offset xo
               :y-offset yo
               :z-offset -0.06))
       (?push (cram-tf:translate-pose 
               ?pose
               :x-offset xo
               :y-offset yo
               :z-offset -0.08))
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

#+TOUCH-board-front
(let* ((?constraint '("left_gripper_joint" "triangle_base_joint")))
        (multiple-value-bind (?pose ?dir) (touch-trajectory :big-wooden-plate :from :front :offset '(0 0 0))
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


#+ASSEMBLE-example----put-chassis-in-gripper-before
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
