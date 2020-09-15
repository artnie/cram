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

(defparameter *gripper-tip-margin* (cl-tf:make-3d-vector 0.016 0.009 0.008))

(defun viz-debug-pose (&rest poses)
  (let ((id 987))
    (loop for pose in poses
          do (cram-tf:visualize-marker pose
                                       :scale-list `(0.15 0.05 0.05)
                                       :id (incf id)
                                       :marker-type :arrow
                                       :topic "testing/visualization_marker"
                                       :r-g-b-list '(0.2 0.2 0.2 1.0)))))

(defun viz-trajectory-split (&rest poses)
  (let ((id 987))
    (loop for pose in poses
          do (cram-tf:visualize-marker pose
                                       :scale-list `(0.1 0.005 0.005)
                                       :id (incf id)
                                       :marker-type :arrow
                                       :topic "testing/visualization_marker"
                                       :r-g-b-list '(0.5 0.2 0.2 1.0)))))

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
    (nth (position axis '(:fx :fy :fz :tx :ty :tz))
         (list fx fy fz tx ty tz))))

(defun force-mean (msg)
  (roslisp:with-fields ((fx (x force wrench))
                        (fy (y force wrench))) msg
    (list fx fy)))

(defun chassis-heuristic (msg)
  (roslisp:with-fields ((fx (x force wrench))
                        (fy (y force wrench))
                        (fz (z force wrench))
                        (tx (x torque wrench))
                        (ty (y torque wrench))
                        (tz (z torque wrench))) msg
    (if (< fz -1.5)
        (if ;; (and (> fy 1.0) 
         (> (abs ty) (abs tx))
            ;; +-x
            (if (> 0 ty)
                '(-1 0 :+x-off)
                (if (> ty 0)
                    '(1 0 :-x-off)
                    '(0 0 :unknown)))
            ;; +-y
            (if (> 0 tx);; (or (> tz tx ty))
                '(0 -1 :+y-off)
                (if (> tx 0)
                    '(0 1 :-y-off)
                    '(0 0 :unknown))))
        '(0 0 :success))))

(defparameter *fl-default-sample-rate* 1.0)

(defgeneric fl-active (fluent &optional sample-rate)
  (:documentation "Waits double the `sample-rate' in Hz for `fluent' to be pulsed.
                   On the first `cpl:whenever' the body is always executed when
                   the `fluent' was pulsed at least once after creation, so the test
                   passes on the next observed pulse, and returns NIL otherwise.")
  (:method ((fluent cpl:value-fluent) &optional (sample-rate *fl-default-sample-rate*))
    (let ((init-pulse-passed nil)) ;; not necessary to set nil, but for better readability 
      (cpl:pursue
        (cpl:whenever ((cpl:pulsed fluent))
          (if init-pulse-passed
              (return T)
              (setf init-pulse-passed T)))
        (progn (sleep (/ 2.0 sample-rate))
               (roslisp:ros-info (fl-active) "Fluent ~a is inactive." (cpl-impl:name fluent))
               nil)))))

(defgeneric fl-gate (fluent &optional sample-rate)
  (:documentation "Throws error when `fluent' is not active.")
  (:method ((fluent cpl:value-fluent) &optional (sample-rate *fl-default-sample-rate*))
    (unless (fl-active fluent sample-rate)
      (error "[touch plan] The fluent ~a is inactive." fluent))))

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
    (review-all-objects)))

;; Touch plan: close gripper, move near the object, approach the object until force response
(defun touch (&key
                ((:object ?object))
                ((:arm ?arm))
                ((:pose ?pose))
                ((:direction ?direction))
              &allow-other-keys)
  (fl-gate boxy-ll:*wrench-state-fluent*)
  (unless (listp ?pose)
    (setf ?pose (list ?pose)))
  (let* ((?object-name (desig:desig-prop-value ?object :name))
         (?gripper-joint (list
                          (cut:var-value
                           '?joint
                           (car
                            (prolog `(and (rob-int:robot ?robot)
                                          (rob-int:gripper-joint ?robot :left ?joint)))))))
         (?constraints (append
                        ?gripper-joint
                        '("triangle_base_joint")))
         (?constraints-wo-base (append
                                ?gripper-joint
                                '("odom_x_joint"
                                  "odom_y_joint"
                                  "odom_z_joint")))
         (?constraints-wo-all (append
                                ?gripper-joint
                                '("triangle_base_joint"
                                  "odom_x_joint"
                                  "odom_y_joint"
                                  "odom_z_joint")))
         (?2nd-touch-pose (destructuring-bind (x-off y-off z-off) ?direction
                            (list (cram-tf:translate-pose (car (last ?pose))
                                                          :x-offset (* x-off 0.9)
                                                          :y-offset (* y-off 0.9)
                                                          :z-offset (* z-off 0.9))
                                  (cram-tf:translate-pose (car (last ?pose))
                                                          :x-offset (* x-off 1.2)
                                                          :y-offset (* y-off 1.2)
                                                          :z-offset (* z-off 1.2))))))
    ;;Go home
    (home-ee T)
    (home-torso)
    (home-ee nil)
    ;; (home-arms)
    ;; Move to suitable position to touch the board
    (break "moving base to table")
    (let* ((?base-pose *good-base-pose*))
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location
                                  (pose ?base-pose))))))
 
    (roslisp:ros-info (palpating) "Closing gripper")
    (exe:perform
     (desig:an action
               (type closing-gripper)
               (gripper ?arm)))

    (break "preparing gripper orientation")
    (let* ((current-ee-pose (cl-tf:lookup-transform cram-tf:*transformer*
                                                    "map"
                                                    "left_gripper_tool_frame"))
           (origin-above-goal (cl-tf:origin (cram-tf:translate-pose
                                             (car ?pose)
                                             :z-offset 0.1)))
           (?above-goal-pose (cl-tf:make-pose-stamped
                                         (cl-tf:frame-id current-ee-pose) 0.0
                                         origin-above-goal
                                         (cl-tf:orientation (car ?pose))))
           (?adjusted-gripper-orientation-pose
             (list (cl-tf:make-pose-stamped
                    (cl-tf:frame-id current-ee-pose) 0.0
                    (cl-tf:translation current-ee-pose)
                    (cl-tf:orientation (car ?pose)))))
           (?gripper-positioning-constraints
             (append ?constraints-wo-all 
                     '("left_arm_0_joint" "left_arm_1_joint"
                       "left_arm_2_joint" "left_arm_3_joint"
                       "left_arm_4_joint" "left_arm_5_joint"))))
      (exe:perform
       (desig:an action
                 (type reaching)
                 (desig:when (eql ?arm :left)
                   (left-poses ?adjusted-gripper-orientation-pose))
                 (constraints ?gripper-positioning-constraints)))
      (push ?above-goal-pose ?pose))
    
    (break "Reaching pre-touch pose")
    (exe:perform
     (desig:an action
               (type reaching)
               (desig:when (eql ?arm :left)
                 (left-poses ?pose))
               (desig:when (eql ?arm :right)
                 (right-poses ?pose))
               (constraints ?constraints-wo-base)))

    (break "Goal reached?")
    (let (touched
          (?left-touch-trajectory (split-trajectory-between
                              (cram-tf:strip-transform-stamped 
                               (cl-tf:lookup-transform cram-tf:*transformer*
                                                       "map" "left_gripper_tool_frame"))
                              (car (last ?2nd-touch-pose))))
          ;; ?right-touch-trajectory
          (time-touched 0))
      (apply #'viz-trajectory-split ?left-touch-trajectory)
      (break "Following touch trajectory until force detected")
      (fl-gate boxy-ll:*wrench-state-fluent*)
      (boxy-ll::zero-wrench-sensor)

      (labels ((push-further (?trajectory &optional ?last-pose)
               (break "pushing further ~a" ?trajectory)
               (let ((?pose (list (pop ?trajectory))))
                 (fl-gate boxy-ll:*wrench-state-fluent*)
                 (when ?pose
                   (cpl:pursue
                    (and (cpl:wait-for
                           (cpl-impl:fl>
                            (cpl:fl-funcall #'force-aggregated boxy-ll:*wrench-state-fluent*)
                            4.0))
                          (roslisp:ros-info (palpating) "Object touched.")
                          (setf touched T))
                     (and (exe:perform
                           (desig:an action
                                     (type pushing)
                                     (left-poses ?pose)
                                     (constraints ?constraints-wo-all)))
                          (sleep 1)))
                   (if touched
                       (progn
                         (break "touched")
                         (update-object-pose-from-touch ?object-name ?direction)
                         (incf time-touched)
                         (setf touched nil)
                         (when (and ?last-pose (< time-touched 3))
                           (exe:perform
                           (desig:an action
                                     (type pushing)
                                     (left-poses ?last-pose)
                                     (constraints ?constraints-wo-all)))
                           (push-further (append ?pose ?trajectory))))
                       (push-further ?trajectory ?pose))))))
        (push-further (cdr (copy-list ?left-touch-trajectory)) (list (car ?left-touch-trajectory)))))     

    (break "[touch] Touching plan finished. Is the motion done jet?")

    (update-object-pose-from-touch ?object-name ?direction)
    
    (break "Reaching back to pre-touch pose")
    (let ((?pre-grasp-pose (list (car ?pose))))
      (exe:perform
       (desig:an action
                 (type reaching)
                 (desig:when (eql ?arm :left)
                   (left-poses ?pre-grasp-pose))
                 (desig:when (eql ?arm :right)
                   (right-poses ?pre-grasp-pose))
                 (constraints ?constraints-wo-all))))))
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
  (fl-gate boxy-ll:*wrench-state-fluent*)
  "Reach, put, assert assemblage if given, open gripper, retract grasp event, retract arm."

  ;; (home-ee t)
  ;; ;; #+prepare-gripper-orientation
  ;; (let* ((current-ee-pose (cl-tf:lookup-transform cram-tf:*transformer* "map" "left_gripper_tool_frame"))
  ;;        (current-ee-origin (cl-tf:translation current-ee-pose))
  ;;        (goal-ee-orientation (cl-tf:orientation (car (last ?left-reach-poses))))
  ;;        (?prepare-gripper-pose (list 
  ;;                                (cl-tf:make-pose-stamped
  ;;                                 (cl-tf:frame-id current-ee-pose) 0.0
  ;;                                 current-ee-origin
  ;;                                 goal-ee-orientation)))
  ;;        (?constraints (remove-duplicates
  ;;                       (append ?constraints 
  ;;                               '("left_arm_0_joint" "left_arm_1_joint"
  ;;                                 "left_arm_2_joint" "left_arm_3_joint"
  ;;                                 "left_arm_4_joint" "left_arm_5_joint"))
  ;;                       :test #'string=)))
  ;;   (exe:perform
  ;;    (desig:an action
  ;;              (type reaching)
  ;;              (desig:when (eql ?arm :left)
  ;;                (left-poses ?prepare-gripper-pose))
  ;;              (constraints ?constraints))))
  
  
  ;; #+reaching
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up) "Manipulation messed up: ~a~%Ignoring." e)))
    (roslisp:ros-info (assembly assemble) "Reaching")
    (exe:perform
     (desig:an action
               (type reaching)
               (left-poses ?left-reach-poses)
               (right-poses ?right-reach-poses)
               (constraints ?constraints))))

  (roslisp:ros-info (assembly assemble) "Putting")
  (boxy-ll::zero-wrench-sensor)

  (break "Ready for heuristic magic?")
  (let* ((tool-frame (cut:var-value 
                      '?tool-frame
                      (cut:lazy-car 
                       (prolog `(rob-int:robot-tool-frame ,(rob-int:current-robot-symbol)
                                                          :left
                                                          ?tool-frame)))))
         (?left-trajectory (split-trajectory-between
                           (cram-tf:strip-transform-stamped 
                            (cl-tf:lookup-transform cram-tf:*transformer*
                                                    "map" "left_gripper_tool_frame"))
                           (car (last ?left-put-poses))))
         (?left-trajectory-copy (copy-list ?left-trajectory))
         touched
         (heuristic-results (make-hash-table :test #'equal))
         (min-heuristic-data 4)
         (max-heuristic-data 9)
         (reposition-factor 0.01))
    (apply #'viz-trajectory-split ?left-trajectory)
    (labels ((translate-pose-with-heuristic (pose heuristic-result)
               (cram-tf:strip-transform-stamped
                (cram-tf:apply-transform 
                 (cram-tf:pose-stamped->transform-stamped pose
                                                          tool-frame)
                 (cl-tf:make-transform-stamped  tool-frame "offset_frame" 0.0
                                                (cl-tf:make-3d-vector (* reposition-factor  
                                                                         (first heuristic-result))
                                                                      (* reposition-factor  
                                                                         (second heuristic-result)) 
                                                                      0.0)
                                                (cl-tf:make-identity-rotation)))))
             (generate-trajectory (start-pose end-pose)
               (split-trajectory-between
                (cram-tf:translate-pose start-pose
                                        :z-offset (+ 0.01 (random 0.005)))
                end-pose))
             (heuristic-data-sum ()
               (apply #'+ (alexandria:hash-table-values heuristic-results)))
             (dominant-result-found (key)
               (< (ceiling (/ (heuristic-data-sum) 2.0))
                  (gethash key heuristic-results) ))
             (enough-data ()
               (>= (heuristic-data-sum) min-heuristic-data))
             (retract-from-touch (?previous-pose)
               (break "Retract to re-touch.")
               (exe:perform
                (desig:an action
                          (type putting)
                          (object ?object-designator)
                          (desig:when ?other-object-designator
                            (supporting-object ?other-object-designator))
                          (left-poses ?previous-pose)
                          (right-poses nil)
                          (constraints ?constraints))))
             (follow-trajectory (?poses &key retract)
               ;; terminates when last pose is reached
               ;; without invoking z-axis force
               (apply #'viz-trajectory-split ?poses)
               (when retract
                 (retract-from-touch (list (pop ?poses))))
               (break "Move towards collision.")
               (let ((?current-pose (list (pop ?poses))))
                 (when ?current-pose
                   (fl-gate boxy-ll:*wrench-state-fluent*)
                   (boxy-ll::zero-wrench-sensor)
                   (cpl:pursue
                     (and (exe:perform
                           (desig:an action
                                     (type putting)
                                     (object ?object-designator)
                                     (desig:when ?other-object-designator
                                       (supporting-object ?other-object-designator))
                                     (left-poses ?current-pose)
                                     (right-poses nil)
                                     (constraints ?constraints)))
                          (sleep 1))
                     (and (cpl:wait-for (cpl-impl:fl<
                                         (cpl:fl-funcall #'force-on-axis
                                                         boxy-ll:*wrench-state-fluent*
                                                         :fz)
                                         -4.0))
                          (roslisp:ros-info (palpating) "Object touched.")
                          (setf touched T)))
                   (if touched
                       (let ((heuristic-result
                               (or (sleep 1) ;; wait for force to stabilize
                                   (cpl:value (cpl:fl-funcall #'chassis-heuristic
                                                              boxy-ll:*wrench-state-fluent*)))))
                         (break "Touched! Heuristic result ~a, list is ~a"
                                heuristic-result
                                (format nil "~{~%~a~}"
                                        (alexandria:hash-table-alist heuristic-results)))
                         (setf touched nil)
                         (if (gethash heuristic-result heuristic-results)
                             (incf (gethash heuristic-result heuristic-results))
                             (setf (gethash heuristic-result heuristic-results) 1))
                         (if (enough-data)
                             ;; if enough data is found, check if oe is dominant,
                             ;; otherwise search on
                             ;; (dominant-result-found heuristic-result))
                             (if (dominant-result-found heuristic-result)
                                 ;; suitable result found
                                 ;; apply suggested offset on remaining poses
                                 ;; then go on trying
                                 (progn (break "DOMINANT RESULT FOUND: ~a" heuristic-result)
                                        (setf heuristic-results (make-hash-table :test #'equal))
                                        (follow-trajectory
                                         (mapcar
                                          (alexandria:rcurry #'translate-pose-with-heuristic
                                                             heuristic-result)
                                          (generate-trajectory
                                           (car ?current-pose)
                                           (car (last ?poses))))
                                         :retract T))
                                 (progn (break "No dominant result found. Retract and continue.")
                                        (follow-trajectory (generate-trajectory
                                                            (car ?current-pose)
                                                            (car (last ?poses)))
                                                           :retract T)))
                             ;; else get more data
                             (if (< (heuristic-data-sum) max-heuristic-data)
                                 (progn
                                   (break "~a~%~a~%~a"
                                          "Not enough data."
                                          "Max results not reached, yet."
                                          "Retract and continue.")
                                   (follow-trajectory (generate-trajectory
                                                       (car ?current-pose)
                                                       (car (last ?poses)))
                                                      :retract T))
                                 (break "Max results reached. No dominant found. Cancelling."))))
                         (follow-trajectory ?poses))))))
      (boxy-ll::zero-wrench-sensor)
      (follow-trajectory ?left-trajectory-copy)
      heuristic-results
      ))
      
  #+old-heuristic-repositioning-code
  (let ((?flex-left-reach-poses (last ?left-reach-poses))
        (?flex-right-reach-poses (last ?right-reach-poses))
        (?flex-left-put-poses ?left-put-poses)
        (?flex-right-put-poses ?right-put-poses)
        (heuristic-response '(0 0 :init))
        (reposition-factor 0.01))
    (cpl:with-retry-counters ((clicking-retries 50))
      (cpl:with-failure-handling
          (((or common-fail:manipulation-low-level-failure
                common-fail:manipulation-goal-not-reached) (e)
             (roslisp:ros-warn (assembly assemble)
                               "Manipulation messed up: ~a~%Ignoring." e)
             (cpl:do-retry clicking-retries
               (sleep 1) ;; waiting for force to stabilize#

               (break "Executing heurstic classfication.")
               ;; (roslisp:ros-info (assembly-demo assembling)
               ;;                   "Heruistic tries for adjustment in ~a"
               ;;                   heuristic-response)
               (setf heuristic-response
                     (cpl:wait-for (cpl:fl-funcall #'chassis-heuristic boxy-ll:*wrench-state-fluent*)))
               (break "Heruistic says ~a" heuristic-response)
               (unless (eq (third heuristic-response) :success)
                 (roslisp:ros-info (assembly-demo assembling) "Retracting for retry.")
                 (exe:perform
                  (desig:an action
                            (type reaching)
                            (left-poses ?flex-left-reach-poses)
                            (right-poses ?flex-right-reach-poses)
                            (collision-mode :avoid-all)
                            (constraints ?constraints)))
                 (setf ?flex-left-reach-poses
                       (mapcar (lambda (pose)
                                 (cram-tf:translate-pose
                                  pose
                                  :x-offset (* reposition-factor (first heuristic-response))
                                  :y-offset (* reposition-factor (second heuristic-response))))
                               ?flex-left-reach-poses))
                 (setf ?flex-right-reach-poses
                       (mapcar (lambda (pose)
                                 (cram-tf:translate-pose
                                  pose
                                  :x-offset (* reposition-factor (first heuristic-response))
                                  :y-offset (* reposition-factor (second heuristic-response))))
                               ?flex-right-reach-poses))
                 (setf ?flex-left-put-poses
                       (mapcar (lambda (pose)
                                 (cram-tf:translate-pose
                                  pose
                                  :x-offset (* reposition-factor (first heuristic-response))
                                  :y-offset (* reposition-factor (second heuristic-response))))
                               ?flex-left-put-poses))
                 (setf ?flex-right-put-poses
                       (mapcar (lambda (pose)
                                 (cram-tf:translate-pose
                                  pose
                                  :x-offset (* reposition-factor (first heuristic-response))
                                  :y-offset (* reposition-factor (second heuristic-response))))
                               ?flex-right-put-poses))
                 
                 (viz-debug-pose (car (last ?flex-left-reach-poses)))
                 
                 (roslisp:ros-info (assembly-demo assembling) "Repositioning to adjusted reach pose.")
                 (exe:perform
                  (desig:an action
                            (type reaching)
                            (left-poses ?flex-left-reach-poses)
                            (right-poses ?flex-right-reach-poses)
                            (collision-mode :allow-all)
                            (constraints ?constraints)))))
             (if (eq (third heuristic-response) :success)
                 (cpl:continue)
                 (cpl:retry))))
        (break "[assembly] Putting object & force detect in parallel.")
        (cpl:pursue
          (and (cpl:wait-for (cpl:> (cpl:fl-funcall #'force-aggregated boxy-ll:*wrench-state-fluent*) 3))
               (roslisp:ros-info (assembling) "Object touched.")
               (unless (eq (third heuristic-response) :success)
                 (cpl:fail 'common-fail:manipulation-low-level-failure
                           :description "Object was touched!")))
          (exe:perform
           (desig:an action
                     (type putting)
                     (object ?object-designator)
                     (desig:when ?other-object-designator
                       (supporting-object ?other-object-designator))
                     (left-poses ?flex-left-put-poses)
                     (right-poses ?flex-right-put-poses)
                     (constraints ?constraints)))))))

  #+after-assembly
  (
  (setf giskard::*max-velocity* 0.1)
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
  )

  ;; (roslisp:ros-info (assembly assemble) "Parking")
  ;; (home-torso)
  ;; (home-arms)
  )


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
                 ?left-retract-poses ?right-retract-poses)
           (ignore ?arm  ?gripper-opening ?placing-location-name))
  "Reach, put, assert assemblage if given, open gripper, retract grasp event, retract arm."
  (fl-gate boxy-ll:*wrench-state-fluent*)
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
                 (cpl:wait-for (cpl-impl:fl< (cpl:fl-funcall #'force-on-axis boxy-ll:*wrench-state-fluent* :fz)
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
        (and (cpl:wait-for (cpl-impl:fl> (cpl:fl-funcall #'force-on-axis boxy-ll:*wrench-state-fluent* :fz)
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
