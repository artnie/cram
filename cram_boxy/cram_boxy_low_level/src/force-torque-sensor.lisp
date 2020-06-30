;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :boxy-ll)

(defparameter *wrench-too-high-limit* 3.0 "In N/m.")

(defparameter *wrench-zeroing-service-retries* 3 "how many times")

(defvar *wrench-state-sub* nil
  "Subscriber for robot's 6dof force-torque wrist sensor.")

(defvar *wrench-state-fluent* (cpl:make-fluent :name :wrench-state)
  "ROS message containing robot's left gripper state ROS message.")

(defvar *wrench-state-filtered-sub* nil
  "Filtered subscriber.")

(defvar *wrench-state-filtered-fluent* (cpl:make-fluent :name :wrench-state)
  "Filtered fluent.")

(defun reset-wrench-state ()
  (setf *wrench-zeroing-service-retries* 3)
  (setf *wrench-state-sub* nil)
  (setf *wrench-state-filtered-sub* nil)
  (setf *wrench-state-fluent* (cpl:make-fluent :name :wrench-state))
  (setf *wrench-state-filtered-sub* (cpl:make-fluent :name :wrench-state-filtered))
  (init-wrench-state-sub))

(defun init-wrench-state-sub ()
  "Initializes *wrench-state-sub*"
  (flet ((wrench-state-sub-cb (wrench-state-msg)
           (setf (cpl:value *wrench-state-fluent*) wrench-state-msg))
         (wrench-state-filtered-sub-cb (wrench-state-msg)
           (setf (cpl:value *wrench-state-filtered-fluent*) wrench-state-msg)))
    (setf *wrench-state-sub*
          (roslisp:subscribe "left_arm_kms40/wrench"
                             "geometry_msgs/WrenchStamped"
                             #'wrench-state-sub-cb))
    (setf *wrench-state-filtered-sub*
          (roslisp:subscribe "left_arm_kms40/wrench_filtered"
                             "geometry_msgs/WrenchStamped"
                             #'wrench-state-filtered-sub-cb))))

(defun destroy-wrench-state-sub ()
  (setf *wrench-state-sub* nil))

(roslisp-utilities:register-ros-init-function init-wrench-state-sub)
(roslisp-utilities:register-ros-cleanup-function destroy-wrench-state-sub)

(defun zero-wrench-sensor ()
  (loop for i from 1 to *wrench-zeroing-service-retries*
        until (roslisp:wait-for-service "left_arm_kms40/set_tare" 5.0)
        do (roslisp:ros-info (force-torque-sensor zero-sensor) "Waiting for zeroing service...")
        finally (if (> i *wrench-zeroing-service-retries*)
                    (progn
                      (roslisp:ros-warn (force-torque-sensor zero-sensor)
                                        "Service unreachable. Stop using force-torque sensor.")
                      (setf *wrench-zeroing-service-retries* 0))
                    (roslisp:call-service "left_arm_kms40/set_tare"
                                          "std_srvs/SetBool" 
                                          (roslisp:make-request 'std_srvs-srv:setbool :data T)))))
