;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defun spawn-objects-on-plate (&optional (spawning-poses *object-spawning-data*))
  ;; (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (mapcar (alexandria:compose #'btr:detach-all-objects #'car) spawning-poses)
  ;; let ((object-types '(:breakfast-cereal :cup :bowl :spoon :milk)))
  ;; spawn objects at default poses
  (let ((objects (mapcar (lambda (object-name-type-pose-list)
                           (destructuring-bind (object-name object-type object-color
                                                object-pose-list)
                               object-name-type-pose-list
                             (roslisp:ros-info (assembly-setup) "Spawning ~a" object-name)
                             (let* ((object-relative-pose
                                      (cram-tf:list->pose object-pose-list))
                                    (spawned-obj (btr-utils:spawn-object
                                                  object-name
                                                  object-type
                                                  :mass 0.0
                                                  :color object-color
                                                  :pose (cram-tf:pose->list
                                                         (cl-tf:make-pose
                                                          (cl-transforms:v+
                                                           (cl-transforms:make-3d-vector
                                                            (- *plate-x* *plate-rad-x*)
                                                            (- *plate-y* *plate-rad-y*)
                                                            (+ *plate-z* *plate-rad-z*))
                                                           (cl-transforms:origin
                                                            object-relative-pose))
                                                          (cl-transforms:orientation
                                                           object-relative-pose))))))
                               (visualize-part object-name)
                               (add-to-collision-scene object-name)
                               spawned-obj)))
                         spawning-poses)))
    objects))


(defmethod exe:generic-perform :before (designator)
  (format t "~%PERFORMING~%~A~%~%" designator))


(defun exec (phase &key (only t) reset)
  (when reset (reset))
  (if (<= 0 phase (length *assembly-steps*))
      (let ((assembly-step (nth phase *assembly-steps*)))
        (roslisp:ros-info (assembly exec)
                          "Executing phase ~a:~%Putting ~a on ~a."
                          phase (nth 1 assembly-step) (nth 3 assembly-step))
        (with-giskard-controlled-robot
          (if only
            (eval (nth phase *assembly-steps*))
            (loop for step to phase
                  do (eval (nth step *assembly-steps*))))))
      (roslisp:ros-warn (assembly exec) "There is no phase ~a." phase)))

(defun demo (&optional (until-phase 13))
  ;;(setf cram-robosherlock::*no-robosherlock-mode* t)
  ;; (spawn-objects-on-plate)
  (reset)
  (exec until-phase))

(defun initialize-attachments ()
  (btr:attach-object :motor-grill :underbody)
  (loop for holder in '(:holder-bolt :holder-upper-body :holder-bottom-wing :holder-underbody
                        :holder-plane-horizontal :holder-window :holder-plane-vertical :holder-top-wing)
        do (btr:attach-object :big-wooden-plate holder))
  (btr:attach-object :big-wooden-plate :chassis :loose t)
  (btr:attach-object :holder-bottom-wing :bottom-wing)
  (btr:attach-object :holder-underbody :underbody)
  (btr:attach-object :holder-upper-body :upper-body)
  (btr:attach-object :holder-top-wing :top-wing)
  (btr:attach-object :holder-plane-horizontal :rear-wing))

(defun home ()
  (with-giskard-controlled-robot
    (home-torso)
    (home-arms)))

(defun home-arms ()
  (exe:perform
   (desig:an action
             (type positioning-arm)
             (left-configuration park)
             ;; (right-configuration park)
             )))

(defun home-torso ()
  (exe:perform
   (desig:a motion (type moving-torso)
            (joint-angle -0.2))))

(defun go-perceive (?object-type ?nav-goal)
  ;; park arms
  ;; (home-torso)
  ;; (home-arms)
  
  ;; drive to right location
  (let ((?pose (cl-transforms-stamped:pose->pose-stamped
                cram-tf:*fixed-frame*
                0.0
                (btr:ensure-pose ?nav-goal))))
    (exe:perform
     (desig:an action
               (type going)
               (target (desig:a location
                                (pose ?pose))))))

  (exe:perform
       (desig:a motion
                (type world-state-detecting)
                (object (desig:an object (type ?object-type)))))
  
  ;; ;; Ignore head movement since we do world-state-detection
  ;; ;; look down
  ;; (exe:perform
  ;;  (desig:an action
  ;;            (type looking)
  ;;            (direction down-left)))
  ;; ;; perceive object
  ;; (let ((?object
  ;;         (exe:perform
  ;;          (desig:an action
  ;;                    (type detecting)
  ;;                    (object (desig:an object (type ?object-type)))))))
  ;;   ;; look away
  ;;   (exe:perform
  ;;    (desig:an action
  ;;              (type looking)
  ;;              (direction away)))
  ;;   ?object)
  )

(defun go-pick (?object-type ?nav-goal)
  ;; go and perceive object
  (let ((?object
          (go-perceive ?object-type ?nav-goal))
        (?constraints '(;; "odom_x_joint"
                        ;; "odom_y_joint"
                        ;; "odom_z_joint"
                        ;; "triangle_base_joint" 
                        )))
    ;; (home-torso)
    ;; (home-arms)

    (btr:detach-all-objects (btr:object btr:*current-bullet-world* (desig:desig-prop-value ?object :name)))
    ;; pick object
    (exe:perform
     (desig:an action
               (type picking-up)
               (arm left)
               (object ?object)
               (constraints ?constraints)))
    ?object))

(defun go-pick-place (?object-type ?nav-goal)
  ;; go and pick up object
  (let ((?object
          (go-pick ?object-type ?nav-goal)))
    ;; put the cookie down
    (exe:perform
     (desig:an action
               (type placing)
               (object ?object)))))

(defun go-connect (?object-type ?nav-goal ?other-object-type ?other-nav-goal ?attachment-type)
  ;; go and pick up object
  (let ((?object
          (go-pick ?object-type ?nav-goal)))
    ;; go and perceive other object
    (let ((?other-object
            (go-perceive ?other-object-type ?other-nav-goal)))
      (exe:perform
       (desig:an action
                 (type assembling)
                 (arm left)
                 (object ?object)
                 (target (desig:a location
                                  (on ?other-object)
                                  (for ?object)
                                  (attachment ?attachment-type))))
       ;; (desig:an action
       ;;           (type placing)
       ;;           (arm left)
       ;;           (object ?object)
       ;;           ;; this location designator is resolved in
       ;;           ;; cram_boxy_plans/src/action-designators.lisp
       ;;           (target (desig:a location
       ;;                            (on ?other-object)
       ;;                            (for ?object)
       ;;                            (attachment ?attachment-type))))
       )
      (values ?object ?other-object))))

(defun palpate-board ()
  (let ((?object (exe:perform
                  (desig:an action
                            (type detecting)
                            (object (desig:an object (name :big-wooden-plate)))))))
    (multiple-value-bind (pose dir) (touch-trajectory :big-wooden-plate :from :top :offset '(-0.3 0 0))
      (touch :object ?object
             :arm :left
             :pose pose
             :direction dir))
    (multiple-value-bind (pose dir) (touch-trajectory :big-wooden-plate :from :front)
      (touch :object ?object
             :arm :left
             :pose pose
             :direction dir))
    (multiple-value-bind (pose dir) (touch-trajectory :big-wooden-plate :from :left)
      (touch :object ?object
             :arm :left
             :pose pose
             :direction dir))))

#+examples
(
 (boxy-proj:with-projected-robot
    (cram-executive:perform
     (desig:an action
               (type looking)
               (direction down))))

 (boxy-proj:with-projected-robot
     (cram-executive:perform
      (desig:an action
                (type detecting)
                (object (desig:an object (type chassis))))))

 (boxy-proj:with-simulated-robot
  (exe:perform
   (desig:an action
            (type opening-gripper)
            (gripper left))))

 (boxy-proj:with-projected-robot
    (cram-executive:perform
     (desig:an action
               (type placing)
               (arm left))))
 )

#+everything-below-is-pr2-s-stuff-so-need-new-things-for-boxy
(
(defun demo-hard-coded ()
  (spawn-objects-on-plate)

  (boxy-proj:with-simulated-robot

    (dolist (object-type '(:breakfast-cereal :cup :bowl :spoon :milk))

      (let ((placing-target
              (cl-transforms-stamped:pose->pose-stamped
               "map" 0.0
               (cram-bullet-reasoning:ensure-pose
                (cdr (assoc object-type *object-placing-poses*)))))
            (arm-to-use
              (cdr (assoc object-type *object-grasping-arms*))))

        (pick-object object-type arm-to-use)
        (place-object placing-target arm-to-use)))))
)
