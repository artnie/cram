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

(defparameter *object-cad-models*
  '(;; (:cup . "cup_eco_orange")
    ;; (:bowl . "edeka_red_bowl")
    ))

(defparameter *object-colors*
  '((:spoon . "Black")
    ;; (:spoon . "Blue")
    ;; (:breakfast-cereal . "Yellow")
    ;; (:milk . "Blue")
    (:bowl . "Red")
    (:cup . "Red")))

(defparameter *object-materials*
  '(;; (:spoon . "Steel")
    ))

(defparameter *object-grasps*
  '((:spoon . :top)
    (:breakfast-cereal . :front)
    (:milk . :front)
    (:cup . :top)
    (:bowl . :top)))

(defparameter *object-arms*
  '((:milk . :left)))

(defun park-robot ()
  (cpl:with-failure-handling
      ((cpl:plan-failure (e)
         (declare (ignore e))
         (return)))
    (exe:perform
     (desig:an action
               (type moving-torso)
               (joint-angle upper-limit)))
    (cpl:par
      (exe:perform
       (desig:an action
                 (type positioning-arm)
                 (left-configuration park)
                 (right-configuration park)))
      (exe:perform (desig:an action (type opening-gripper) (gripper (left right))))
      (exe:perform (desig:an action (type looking) (direction forward))))
    (let ((?pose (cl-transforms-stamped:make-pose-stamped
                  cram-tf:*fixed-frame*
                  0.0
                  (cl-transforms:make-identity-vector)
                  (cl-transforms:make-identity-rotation))))
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location (pose ?pose))))))))

(defun start-logging ()
  (ccl::start-episode))

(defun stop-logging ()
  (ccl::stop-episode))

(defun initialize ()
  (sb-ext:gc :full t)

  ;; (setf proj-reasoning::*projection-checks-enabled* t)

  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))
  (btr-utils:kill-all-objects)
  (setf (btr:joint-state (btr:get-environment-object)
                         "sink_area_left_upper_drawer_main_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "sink_area_left_middle_drawer_main_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "sink_area_left_bottom_drawer_main_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "iai_fridge_door_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "sink_area_dish_washer_door_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "sink_area_dish_washer_tray_main")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "oven_area_area_right_drawer_main_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "sink_area_trash_drawer_main_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "kitchen_island_left_upper_drawer_main_joint")
        0.0)
  (btr-belief::publish-environment-joint-state
   (btr:joint-states (btr:get-environment-object)))

  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))

  (coe:clear-belief)

  (btr:clear-costmap-vis-object))

(defun finalize ()
  ;; (setf proj-reasoning::*projection-reasoning-enabled* nil)

  (sb-ext:gc :full t))


(cpl:def-cram-function demo-random (&optional
                                    (random
                                     nil)
                                    (list-of-objects
                                     '(:bowl :spoon :cup :milk :breakfast-cereal)))

  (initialize)
  (when cram-projection:*projection-environment*
    (spawn-objects-on-sink-counter :random random))

  (park-robot)

  (dolist (?object-type list-of-objects)
    (let* ((?arm-to-use
             (cdr (assoc ?object-type *object-grasping-arms*)))
           (?cad-model
             (cdr (assoc ?object-type *object-cad-models*)))
           (?color
             (cdr (assoc ?object-type *object-colors*)))
           (?material
             (cdr (assoc ?object-type *object-materials*)))
           (?object-to-fetch
             (desig:an object
                       (type ?object-type)
                       (desig:when ?cad-model
                         (cad-model ?cad-model))
                       (desig:when ?color
                         (color ?color))
                       (desig:when ?material
                         (material ?material)))))

      (cpl:with-failure-handling
          ((common-fail:high-level-failure (e)
             (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
             (return)))

        (exe:perform
         (desig:an action
                   (type transporting)
                   (context table-setting-counter)
                   (object ?object-to-fetch)
                   ;; (desig:when ?arm-to-use
                   ;;   (arms (?arm-to-use)))
                   )))

      ;; (setf proj-reasoning::*projection-reasoning-enabled* nil)
      ))

  ;; (setf proj-reasoning::*projection-reasoning-enabled* nil)

  (park-robot)

  (finalize)

  cpl:*current-path*)
