;;;
;;; Copyright (c) 2020, Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
(in-package :demo)

(defparameter *gripper-tip-margin* (cl-tf:make-3d-vector 0.011 0.009 0.008))

(defun viz-debug-pose (pose)
  (cram-tf:visualize-marker pose
                            :scale-list `(0.15 0.05 0.05)
                            :id 987
                            :marker-type :arrow
                            :topic "testing/visualization_marker"
                            :r-g-b-list '(0.2 0.2 0.2 1.0)))

(defparameter *force-threshold* 0.5d0 "Force detected above this threshold indicates contact")
(defparameter *force-timeout* 10.0d0 "Timeout in seconds for waiting for force.")

(defun force-aggregated (msg)
  (roslisp:with-fields ((fx (x force wrench))
                        (fy (y force wrench))
                        (fz (z force wrench))
                        (tx (x torque wrench))
                        (ty (y torque wrench))
                        (tz (z torque wrench))) msg
    (apply #'+ (mapcar #'abs (list fx fy fz tx ty tz)))))

(defun force-on-axis (msg axis)
  (roslisp:with-fields ((fx (x force wrench))
                        (fy (y force wrench))
                        (fz (z force wrench))
                        (tx (x torque wrench))
                        (ty (y torque wrench))
                        (tz (z torque wrench))) msg
    (nth (position axis '(fx fy fz tx ty tz))
         (list fx fy fz tx ty tz))))

(defun force-mean (msg)
  (roslisp:with-fields ((fx (x force wrench))
                        (fy (y force wrench))) msg
    (list fx fy)))

;;;;;;;;;;;;;;
;; TOUCHING ;;
(defun touch-trajectory (object-name &key (from :front) (for-gripper T) (dist 0.1d0) (offset '(0 0 0)))
  "Returns a pose nearby object `object-name' to approach the object from,
   with x pointing towards the object.
`from' describes the direction from where to approach the object w.r.t. map.
       One of :front :back :left :right :top.
`for-gripper' if T, make pose as grippers ee-frame goal (rotate around y for pi, goal towards y).
`dist' is the distance to the object's bonding box.
`offset' adds an offset to the resulting pose in x y z direction."
  (declare (type symbol object-name)
           (type keyword from)
           (type (or null float) dist)
           (type (or null list) offset))
  (unless (member from '(:front :back :left :right :top))
    (error "[touch-trajectory] Parameter FROM is neither of :front :back :left :right or :top but ~a." from))
  (let* ((object (or (btr:object btr:*current-bullet-world* object-name)
                     (error "[touch-trajectory] Object ~a does not exist." object-name)))
         (object-pose (btr:object-pose object-name))
         (bb-vector-in-map (cl-tf:origin
                            (cl-tf:transform 
                             (cl-tf:make-transform (cl-tf:make-identity-vector)
                                                   (cl-tf:orientation object-pose))
                             (cl-tf:make-pose 
                              (btr:calculate-bb-dims object) 
                              (cl-tf:make-identity-rotation)))))
         (approach-orientation (case from   ;; offset-dir orientation
                                 (:back     `(( 1  0  0) ,(cl-tf:euler->quaternion :az pi)))
                                 (:left     `(( 0  1  0) ,(cl-tf:euler->quaternion :az (/ pi -2))))
                                 (:right    `(( 0 -1  0) ,(cl-tf:euler->quaternion :az (/ pi 2))))
                                 (:top      `(( 0  0  1) ,(cl-tf:euler->quaternion :ay (/ pi 2))))
                                 (otherwise `((-1  0  0) ,(cl-tf:make-identity-rotation)))))
         pose)
    (with-slots ((x cl-tf:x) (y cl-tf:y) (z cl-tf:z)) bb-vector-in-map
      (let ((bb-edge-point-in-origin (apply #'cl-tf:make-3d-vector
                                            (mapcar (lambda (val id off)
                                                      (+ (* (abs (/ val 2)) id)
                                                         (* dist id)
                                                         off))
                                                    (list x y z)
                                                    (first approach-orientation)
                                                    offset))))
        (setf pose (cl-tf:make-pose-stamped
                    "map" 0.0 (cl-tf:v+ (cl-tf:origin object-pose) bb-edge-point-in-origin)
                    (if for-gripper
                        (cl-tf:q* (if (eq from :top)
                                      (cl-tf:make-identity-rotation)
                                      (second approach-orientation))
                                  (cl-tf:euler->quaternion :ax pi :az (/ pi 2)))
                        (second approach-orientation))))
        (cram-tf:visualize-marker pose
                                  :scale-list `(,dist 0.005 0.005)
                                  :id 666
                                  :marker-type :arrow
                                  :topic "palpate/visualization_marker"
                                  :r-g-b-list '(1.0 0.0 0.5))
        (values pose (mapcar (lambda (id) (* (- id) dist))
                                     (first approach-orientation)))))))

(defun update-object-pose-from-touch (object-name direction)
  "Calculate position of the gripper tip edge touching the object and update the pose of
the touched object on observed axis given by `direction'.
`direction' is a list of 3 values resembling a 3D base vector, meaning it has only zeroes except
for one value. The vector will be normalized to length 1."
  (let* ((touched-object (btr:object btr:*current-bullet-world* object-name))
           ;; BB dimensions in respective orientation, assuming everything in within 90 degree rotation
           (touched-object-bb-expansion (cl-tf:v* (cl-bullet:bounding-box-dimensions (btr:aabb touched-object))
                                                  0.5d0))
           (old-object-pose (btr:pose touched-object))
           
           (current-gripper-tool-frame-pose
             (cram-tf:strip-transform-stamped (cl-tf:lookup-transform cram-tf:*transformer*  
                                                                      "map" 
                                                                      "left_gripper_tool_frame")))
           ;; Base vector of length 1 with one non-zero entry among zeroes.
           ;; Examples given for touch from :top.
           ;; (0 0 -1) direction of gripper approaching the object.
           (direction-gripper-approach-vector (cl-tf:normalize-vector
                                               (apply 'cl-tf:make-3d-vector direction)))
           ;; (0 0 1) direction of bb expansion towards approaching gripper. 
           (direction-bb-expansion-vector (cl-tf:normalize-vector
                                           (apply 'cl-tf:make-3d-vector (mapcar #'- direction))))
           ;; (0 0 1) non-negative base vector for extracting the observed axis from position vectors.
           (axis-indicator (cl-tf:normalize-vector
                            (apply 'cl-tf:make-3d-vector (mapcar #'abs direction))))
           ;; (0 0 z_g) position of the gripper tip reduced to direction vector
           (gripper-tip-pos-reduced-to-base-vector-on-axis
             (cl-tf:transform (cl-tf:make-transform (cl-tf:v*-pairwise direction-gripper-approach-vector
                                                                       *gripper-tip-margin*)
                                                    (cl-tf:make-identity-rotation))
                              (cl-tf:v*-pairwise axis-indicator
                                                 (cl-tf:origin current-gripper-tool-frame-pose))))
           ;; (0 0 z_o) old position of the object edge on observed axis
           (old-object-connection-pos-reduced-to-base-vector-on-axis
             (cl-tf:transform (cl-tf:make-transform (cl-tf:v*-pairwise direction-bb-expansion-vector
                                                                       touched-object-bb-expansion)
                                                    (cl-tf:make-identity-rotation))
                              (cl-tf:v*-pairwise axis-indicator
                                                 (cl-tf:origin old-object-pose))))
           ;; (0 0 z_t) transformation to update old pose along observed axis
           (update-transform
             (cl-tf:make-transform (cl-tf:v- gripper-tip-pos-reduced-to-base-vector-on-axis
                                             old-object-connection-pos-reduced-to-base-vector-on-axis)
                                   (cl-tf:make-identity-rotation)))
           (updated-object-pose (cl-tf:transform update-transform old-object-pose)))

    (setf (btr:pose touched-object) updated-object-pose)
    (with-giskard-controlled-robot (review-all-objects))))

;; Touch plan: close gripper, move near the object, approach the object until force response
(defun touch (&key
                ((:object ?object))
                ((:arm ?arm))
                ((:pose ?pose))
                ((:direction ?direction))
              &allow-other-keys)
  (unless (listp ?pose)
    (setf ?pose (list ?pose)))
  (let* ((?object-name (desig:desig-prop-value ?object :name))
         (?gripper-joint (list
                          (cut:var-value
                           '?joint
                           (car
                            (prolog `(and (rob-int:robot ?robot)
                                          (rob-int:gripper-joint ?robot :left ?joint)))))))
         (?constraints (append ?gripper-joint '("triangle_base_joint"
                                                "odom_x_joint"
                                                "odom_y_joint"
                                                "odom_z_joint")))
         (?1st-touch-pose (list (destructuring-bind (x-off y-off z-off) ?direction
                                  (cram-tf:translate-pose (car (last ?pose))
                                                          :x-offset x-off
                                                          :y-offset y-off
                                                          :z-offset  z-off))))
         (?2nd-touch-pose (list (destructuring-bind (x-off y-off z-off) ?direction
                                  (cram-tf:translate-pose (car (last ?pose))
                                                          :x-offset (* x-off 1.5)
                                                          :y-offset (* y-off 1.5)
                                                          :z-offset (* z-off 1.5))))))
    ;;Go home
    ;; (home-torso)
    
    ;; (home-arms)
    ;;Move to suitable position to touch the board
    (break "moving base to table")
    (let* ((?nav-goal *base-touch-pose*)
           (?pose (cl-transforms-stamped:pose->pose-stamped
                   cram-tf:*fixed-frame* 0.0
                   (cram-tf:translate-pose (btr:ensure-pose ?nav-goal)
                                           :x-offset (- (first ?direction))
                                           :y-offset (- (second ?direction))
                                           :z-offset (- (third ?direction))))))
          (exe:perform
           (desig:an action
                     (type going)
                     (target (desig:a location
                                      (pose ?pose))))))
    (break "closing the gripper")
    (setf giskard::*max-velocity* 0.1)
    (cpl:par
      (roslisp:ros-info (palpating) "Closing gripper")
      ;;(boxy-ll::move-gripper-joint :action-type-or-position :close :left-or-right :left)
      (exe:perform
       (desig:an action
                 (type closing-gripper)
                 (gripper ?arm)))
      (roslisp:ros-info (palpating) "Reaching pre-touch pose")
      (cpl:with-retry-counters ((touch-retries 0))
         (cpl:with-failure-handling
             ((common-fail:manipulation-goal-not-reached (e)
                (roslisp:ros-warn (palpating reach-pre-touch)
                                  "Can't reach pre-touch pose: ~a"
                                  e)
                (cpl:do-retry touch-retries
                  (break "Reaching pre touch caught manipulation-goal-not-reached fail.~%Retrying")
                  ;; (roslisp:ros-warn (pick-and-place move-torso) "Retrying with slightly off pose...")
                  ;; (push (cl-tf:make-pose-stamped
                  ;;        (cl-tf:frame-id (car ?pose)) (cl-tf:stamp (car ?pose))
                  ;;        (cl-tf:v+ (cl-tf:make-3d-vector 0 0.1 0) (cl-tf:origin (car ?pose)))
                  ;;        (cl-tf:orientation (car ?pose)))
                  ;;       ?pose)
                  (cpl:retry))))
           (break "reaching for pre-touch")
           (exe:perform
            (desig:an action
                      (type reaching)
                      (desig:when (eql ?arm :left)
                        (left-poses ?pose))
                      (desig:when (eql ?arm :right)
                        (right-poses ?pose))
                      (constraints ?constraints))))))

    
    ;; Move gripper until force sensed
    (roslisp:ros-info (palpating) "Moving gripper to touch object.")
    (break "zero wrench")
    (boxy-ll::zero-wrench-sensor)
    (break "set impedance loose and velocity very-slow")
    (roslisp:set-param "joint_impedance" "loose")
    (roslisp:set-param "max_joint_velocity" "very-slow")
    ;; (break "set max-velocity to 0.01")
    ;; (setf giskard::*max-velocity* 0.01)
    (break "touch until sum of force is greater than 2")
    (roslisp:ros-info (palpating) "Moving close to object.")
    (let (touched)
      (cpl:pursue
        (and (cpl:wait-for (cpl:> (cpl:fl-funcall #'force-aggregated boxy-ll:*wrench-state-fluent*) 4.0))
             (roslisp:ros-info (palpating) "Object touched.")
             (setf touched T))
        (exe:perform
         (desig:an action
                   (type pushing)
                   (desig:when (eql ?arm :left)
                     (left-poses ?2nd-touch-pose))
                   (desig:when (eql ?arm :right)
                     (right-poses ?2nd-touch-pose))
                   (constraints ?constraints)))))
    (roslisp:set-param "joint_impedance" "regular")
    (roslisp:set-param "max_joint_velocity" "regular")
    (break "[touch] Touching plan finished. Is the motion done jet?")
    
    ;; Adjust object pose based on gripper position.
    (setf giskard::*max-velocity* 0.1)

    (update-object-pose-from-touch ?object-name ?direction)))
;; TOUCHING ;;
;;;;;;;;;;;;;;

;;;;;;;;;;;;;;
;; ASSEMBLY ;;
(defun assemble (&key
                   ((:object ?object-designator))
                   ((:other-object ?other-object-designator))
                   ((:arm ?arm))
                   ((:gripper-opening ?gripper-opening))
                   ((:attachment-type ?placing-location-name))
                   ((:left-reach-poses ?left-reach-poses))
                   ((:right-reach-poses ?right-reach-poses))
                   ((:left-put-poses ?left-put-poses))
                   ((:right-put-poses ?right-put-poses))
                   ((:left-retract-poses ?left-retract-poses))
                   ((:right-retract-poses ?right-retract-poses))
                   ((:constraints ?constraints))
                 &allow-other-keys)
  (declare (type desig:object-designator ?object-designator)
           (type (or desig:object-designator null) ?other-object-designator)
           (type keyword ?arm)
           (type (or null keyword) ?placing-location-name)
           (type number ?gripper-opening)
           (type (or null list) ; yes, null is also list, but this is better reachability
                 ?left-reach-poses ?right-reach-poses
                 ?left-put-poses ?right-put-poses
                 ?left-retract-poses ?right-retract-poses))
  "Reach, put, assert assemblage if given, open gripper, retract grasp event, retract arm."
  (roslisp:ros-info (assembly assemble) "Reaching")
  (roslisp:set-param "joint_impedance" "regular")
  (roslisp:set-param "max_joint_velocity" "regular")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up) "Manipulation messed up: ~a~%Ignoring." e)))
    (exe:perform
     (desig:an action
               (type reaching)
               (left-poses ?left-reach-poses)
               (right-poses ?right-reach-poses)
               (constraints ?constraints))))
  (roslisp:ros-info (assembly assemble) "Putting")
  (boxy-ll::zero-wrench-sensor)
  (roslisp:set-param "joint_impedance" "loose-arm")
  (roslisp:set-param "max_joint_velocity" "very-slow")
  (setf giskard::*max-velocity* 0.01)
  (cpl:with-retry-counters ((clicking-retries 50))
    (cpl:with-failure-handling
        (((or common-fail:manipulation-low-level-failure
              common-fail:manipulation-goal-not-reached) (e)
           (roslisp:ros-warn (assembly assemble)
                             "Manipulation messed up: ~a~%Ignoring." e)
           (cpl:do-retry clicking-retries
             ;; getting force info
             ;; TODO: Manipulate putting pose towards force ;;;;;;;;>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
             (sleep 1) ;; waiting for force to stabilize
             (let ((xy-force (cpl:wait-for (cpl:fl-funcall #'force-mean boxy-ll:*wrench-state-fluent*)))
                   (gripper-pose (cl-tf:lookup-transform CRAM-TF:*TRANSFORMER* "map" "left_gripper_tool_frame")))
               (viz-debug-pose 
                (cram-tf:translate-pose (cram-tf:strip-transform-stamped gripper-pose)
                                        :x-offset (first xy-force)
                                        :y-offset (second xy-force))))
             ;; retract
             (break "[assembly] Putting object failed, retrying to previous pose.")
               (let ((?left-prepare-retry-pose (last ?left-reach-poses))
                     (?right-prepare-retry-pose (last ?right-reach-poses)))
                 (roslisp:ros-info (assembly-demo assembling) "Retracting for retry.")
                 (exe:perform
                  (desig:an action
                            (type reaching)
                            (left-poses ?left-prepare-retry-pose)
                            (right-poses ?right-prepare-retry-pose)
                            (constraints ?constraints)))
                 (boxy-ll::zero-wrench-sensor)
                 ;; expand put-poses with multiple wiggles
               ))
             (cpl:retry)))
      (break "[assembly] Putting object & force detect in parallel.")
      (cpl:pursue
        (and (cpl:wait-for (cpl:> (cpl:fl-funcall #'force-aggregated boxy-ll:*wrench-state-fluent*) 1.5))
             (roslisp:ros-info (palpating) "Object touched.")
             (cpl:fail 'common-fail:manipulation-goal-not-reached
              :description "Object was touched!"))
        (exe:perform
         (desig:an action
                   (type putting)
                   (object ?object-designator)
                   (desig:when ?other-object-designator
                     (supporting-object ?other-object-designator))
                   (left-poses ?left-put-poses)
                   (right-poses ?right-put-poses)
                   (constraints ?constraints))))))
  (setf giskard::*max-velocity* 0.1)
  (roslisp:set-param "joint_impedance" "regular")
  (roslisp:set-param "max_joint_velocity" "regular")
  (break "[assembly] Assembling done. Has the arm stopped moving?")

  (when ?placing-location-name
    (roslisp:ros-info (assembly assemble) "Asserting assemblage connection in knowledge base")
    (cram-occasions-events:on-event
     (make-instance 'cpoe:object-attached-object
       :object-name (desig:desig-prop-value ?object-designator :name)
       :other-object-name (desig:desig-prop-value ?other-object-designator :name)
       :attachment-type ?placing-location-name)))
  (roslisp:ros-info (assembly assemble) "Opening gripper")
  (exe:perform
   (desig:an action
             (type setting-gripper)
             (gripper ?arm)
             (position ?gripper-opening)))
  (review-all-objects)
  (roslisp:ros-info (assembly assemble) "Retract grasp in knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
     :arm ?arm
     :object-name (desig:desig-prop-value ?object-designator :name)))
  (roslisp:ros-info (assembly assemble) "Retracting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (assembly assemble)
                           "Manipulation messed up: ~a~%Ignoring." e)
         (return)))
    (let ((?without-first-left-retracting-pose (cdr ?left-retract-poses))
          (?without-first-right-retracting-pose (cdr ?right-retract-poses)))
      (exe:perform
       (desig:an action
                 (type retracting)
                 (left-poses ?without-first-left-retracting-pose)
                 (right-poses ?without-first-right-retracting-pose)
                 (constraints ?constraints)))))
  (roslisp:ros-info (assembly assemble) "Parking")
  (home-torso)
  (home-arms))


(defun screw (&key
                ((:object ?object-designator))
                ((:other-object ?other-object-designator))
                ((:arm ?arm))
                ((:gripper-opening ?gripper-opening))
                ((:attachment-type ?placing-location-name))
                ((:left-reach-poses ?left-reach-poses))
                ((:right-reach-poses ?right-reach-poses))
                ((:left-put-poses ?left-put-poses))
                ((:right-put-poses ?right-put-poses))
                ((:left-retract-poses ?left-retract-poses))
                ((:right-retract-poses ?right-retract-poses))
                ((:constraints ?constraints))
              &allow-other-keys)
  (declare (type desig:object-designator ?object-designator)
           (type (or desig:object-designator null) ?other-object-designator)
           (type keyword ?arm)
           (type (or null keyword) ?placing-location-name)
           (type number ?gripper-opening)
           (type (or null list) ; yes, null is also list, but this is better reachability
                 ?left-reach-poses ?right-reach-poses
                 ?left-put-poses ?right-put-poses
                 ?left-retract-poses ?right-retract-poses))
  "Reach, put, assert assemblage if given, open gripper, retract grasp event, retract arm."
  (perform (an action (type closing-gripper) (gripper left)))
  (unless (find "left_gripper_joint" (alexandria:flatten ?constraints) :test #'string=)
    (let ((gripper-constraint (if (and (car ?constraints)
                                       (listp (car ?constraints)))
                                  (append '("left_gripper_joint")
                                          (joints:joint-positions '("left_gripper_joint")))
                                  "left_gripper_joint")))
      (push gripper-constraint ?constraints)))
  (roslisp:ros-info (assembly screw) "Reaching")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (assembly screw) "Manipulation messed up: ~a~%Ignoring." e)))
    (exe:perform
     (desig:an action
               (type reaching)
               (left-poses ?left-reach-poses)
               (right-poses ?right-reach-poses)
               (constraints ?constraints))))

  
  (roslisp:ros-info (assembly assemble) "Positioning driver")
  (boxy-ll::zero-wrench-sensor)
  (roslisp:set-param "joint_impedance" "loose")
  (setf giskard::*max-velocity* 0.015)
  (cpl:with-retry-counters ((position-driver-retries 3))
    (cpl:with-failure-handling
        ((common-fail:manipulation-goal-not-reached (e)
           (roslisp:ros-warn (assembly position-driver)
                             "Positioning driver messed up: ~a~%Ignoring." e)
           (cpl:do-retry position-driver-retries
             (let ((?reposition-pose (list (cram-tf:rotate-pose (car (last ?left-reach-poses))
                                                                :z (/ pi -3.0)))))
               (cpl:pursue
                 ;; release tension by moving up 
                 (cpl:wait-for (cpl-impl:fl< (cpl:fl-funcall #'force-on-axis boxy-ll:*wrench-state-fluent* 'fz)
                                             *force-threshold*)
                               :timeout *force-timeout*)
                 (perform
                  (an action
                      (type reaching)
                      (left-poses ?reposition-pose)
                      (constraints ?constraints)))))
             (cpl:retry)
             )))
      (cpl:pursue
        (and (cpl:wait-for (cpl-impl:fl> (cpl:fl-funcall #'force-on-axis boxy-ll:*wrench-state-fluent* 'fz)
                                         *force-threshold*)
                           :timeout *force-timeout*)
             (cpl:fail 'common-fail:manipulation-goal-not-reached
                       :description "Touched something while positioning driver."))
        (perform
         (an action
             (type pushing)
             (left-poses ?left-put-poses)
             (collision-mode :allow-all)
             (constraints ?constraints))))))
  (roslisp:ros-info (assembly assemble) "Screwing")
  (boxy-ll::zero-wrench-sensor)
  (roslisp:set-param "joint_impedance" "loose-gripper")
  (setf giskard::*max-velocity* 0.01)
  (cpl:with-retry-counters ((screwing-retries 5))
    (cpl:with-failure-handling
        (((or common-fail:manipulation-low-level-failure
              common-fail:manipulation-goal-not-reached) (e)
           (roslisp:ros-warn (assembly assemble)
                             "Manipulation messed up: ~a~%Ignoring." e)
           (cpl:do-retry screwing-retries
             ;; retract
             (let ((?left-prepare-retry-pose (last ?left-reach-poses))
                   (?right-prepare-retry-pose (last ?right-reach-poses)))
               (roslisp:ros-info (assembly-demo assembling) "Retracting for retry.")
               (exe:perform
                (desig:an action
                          (type reaching)
                          (left-poses ?left-prepare-retry-pose)
                          (right-poses ?right-prepare-retry-pose)
                          (constraints ?constraints)))
               (boxy-ll::zero-wrench-sensor)
               ;; expand put-poses with multiple wiggles
               )
             (cpl:retry))))
      (cpl:pursue
        (and (cpl:wait-for (cpl:> (cpl:fl-funcall #'force-aggregated boxy-ll:*wrench-state-fluent*) 0.5))
             (roslisp:ros-info (palpating) "Object touched."))
        (exe:perform
         (desig:an action
                   (type putting)
                   (object ?object-designator)
                   (desig:when ?other-object-designator
                     (supporting-object ?other-object-designator))
                   (left-poses ?left-put-poses)
                   (right-poses ?right-put-poses)
                   (constraints ?constraints))))))
  
  (setf giskard::*max-velocity* 0.1)
  (roslisp:set-param "joint_impedance" "regular")
  (roslisp:ros-info (assembly assemble) "Retracting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (assembly screw)
                           "Manipulation messed up: ~a~%Ignoring." e)
         (return)))
    (exe:perform
     (desig:an action
               (type retracting)
               (left-poses ?left-retract-poses)
               (right-poses ?right-retract-poses)
               (constraints ?constraints))))
  (roslisp:ros-info (assembly screw) "Parking")
  ;; (home-torso)
  ;; (home-arms)
  )
