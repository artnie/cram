;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :giskard)

(defparameter *giskard-convergence-delta-joint* 0.17 "in radiants, about 10 degrees")

(defun make-giskard-joint-action-goal (joint-state-left joint-state-right)
  (declare (type list joint-state-left joint-state-right))
  (roslisp:make-message
   'giskard_msgs-msg:MoveGoal
   :type (roslisp:symbol-code 'giskard_msgs-msg:MoveGoal :plan_and_execute)
   :cmd_seq (vector
             (roslisp:make-message
              'giskard_msgs-msg:movecmd
              :joint_constraints (vector (roslisp:make-message
                                          'giskard_msgs-msg:jointconstraint
                                          :type (roslisp:symbol-code
                                                 'giskard_msgs-msg:jointconstraint
                                                 :joint)
                                          :goal_state (roslisp:make-message
                                                       'sensor_msgs-msg:jointstate
                                                       :name (apply #'vector
                                                                    (first
                                                                     joint-state-left))
                                                       :position (apply #'vector
                                                                        (second
                                                                         joint-state-left))))
                                         (roslisp:make-message
                                          'giskard_msgs-msg:jointconstraint
                                          :type (roslisp:symbol-code
                                                 'giskard_msgs-msg:jointconstraint
                                                 :joint)
                                          :goal_state (roslisp:make-message
                                                       'sensor_msgs-msg:jointstate
                                                       :name (apply #'vector
                                                                    (first
                                                                     joint-state-right))
                                                       :position (apply #'vector
                                                                        (second
                                                                         joint-state-right)))))
              :collisions (vector (roslisp:make-message
                                   'giskard_msgs-msg:collisionentry
                                   :type (roslisp:symbol-code
                                          'giskard_msgs-msg:collisionentry
                                          :avoid_all_collisions)))
              ))))

(defun get-arm-joint-names-and-positions-list (arm &optional joint-states)
  (if joint-states
      (list (mapcar #'first joint-states)
            (mapcar #'second joint-states))
      (let ((joint-names
              (cut:var-value '?joints
                             (cut:lazy-car
                              (prolog:prolog
                               `(cram-robot-interfaces:arm-joints
                                 ,(rob-int:current-robot-symbol)
                                 ,arm ?joints))))))
        (list joint-names
              (joints:joint-positions joint-names)))))

(defun ensure-giskard-joint-input-parameters (left-goal right-goal)
  (flet ((ensure-giskard-joint-goal (goal arm)
           (if (and (listp goal) (= (length goal) 7))
               (get-arm-joint-names-and-positions-list arm goal)
               (and (if goal
                        (roslisp:ros-warn (low-level giskard)
                                          "No joint goal for ~a arm given. Ignoring." arm)
                        (roslisp:ros-warn (low-level giskard)
                                          "Joint goal ~a for ~a arm was not a list of 7. Ignoring."
                                          goal arm))
                    (get-arm-joint-names-and-positions-list arm)))))
   (values (ensure-giskard-joint-goal left-goal :left)
           (ensure-giskard-joint-goal right-goal :right))))

(defun ensure-giskard-joint-arm-goal-reached (arm goal-configuration convergence-delta-joint)
  (if (and (listp goal-configuration) (car goal-configuration))
      (let ((configuration (second (get-arm-joint-names-and-positions-list arm))))
        (values (cram-tf:values-converged (joints:normalize-joint-angles
                                           configuration)
                                          (joints:normalize-joint-angles
                                           (mapcar #'second goal-configuration))
                                          convergence-delta-joint)
                configuration))
      (progn (roslisp:ros-warn (giskard arm-joints)
                               "Empty joint goal for ~a arm. Assuming there's no position to ensure." arm)
             T)))

(defun ensure-giskard-joint-goal-reached (status
                                          goal-configuration-left goal-configuration-right
                                          convergence-delta-joint)
  (when (eql status :preempted)
    (roslisp:ros-warn (giskard arm-joints) "Giskard action preempted.")
    (return-from ensure-giskard-joint-goal-reached))
  (when (eql status :timeout)
    (roslisp:ros-warn (giskard arm-joints) "Giskard action timed out."))
  (flet ((throw-failure (arm goal-configuration current-configuration)
           (cpl:fail 'common-fail:manipulation-goal-not-reached
                     :description (format nil "Giskard did not converge to goal:~%~
                                                   ~a (~a)~%should have been at~%~a~%~
                                                   with delta-joint of ~a."
                                                arm
                                                (joints:normalize-joint-angles
                                                 current-configuration)
                                                (joints:normalize-joint-angles
                                                 (mapcar #'second goal-configuration))
                                                convergence-delta-joint))))
    (multiple-value-bind (reached current-configuration)
        (ensure-giskard-joint-arm-goal-reached :left goal-configuration-left convergence-delta-joint)
      (unless reached (throw-failure :left goal-configuration-left current-configuration)))
    (multiple-value-bind (reached current-configuration)
        (ensure-giskard-joint-arm-goal-reached :right goal-configuration-right convergence-delta-joint)
      (unless reached (throw-failure :right goal-configuration-right current-configuration)))))

(defun call-giskard-joint-action (&key
                                    goal-configuration-left goal-configuration-right action-timeout
                                    (convergence-delta-joint *giskard-convergence-delta-joint*))
  (declare (type list goal-configuration-left goal-configuration-right)
           (type (or null number) action-timeout convergence-delta-joint))
  (multiple-value-bind (joint-state-left joint-state-right)
      (ensure-giskard-joint-input-parameters goal-configuration-left goal-configuration-right)
    
    (if (and (ensure-giskard-joint-arm-goal-reached :left goal-configuration-left convergence-delta-joint)
             (ensure-giskard-joint-arm-goal-reached :right goal-configuration-right convergence-delta-joint))
        (roslisp:ros-info (low-level giskard) "Arms are already in goal-configuration.")
        (multiple-value-bind (result status)
            (actionlib-client:call-simple-action-client
             'giskard-action
             :action-goal (make-giskard-joint-action-goal joint-state-left joint-state-right)
             :action-timeout action-timeout)
          (ensure-giskard-joint-goal-reached status goal-configuration-left goal-configuration-right
                                             convergence-delta-joint)
          (values result status)
          ;; return the joint state, which is our observation
          (joints:full-joint-states-as-hash-table)))))
