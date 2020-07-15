(in-package :demo)


(defun grasp-when-force (&optional open)
  (with-giskard-controlled-robot
    (roslisp:ros-info (debug-grasp) "Zeroing...")
    (boxy-ll::zero-wrench-sensor)
    (sleep 2)
    (roslisp:ros-info (debug-grasp) "Done zeroing.")
    (when open
      (boxy-ll::move-gripper-joint :action-type-or-position :open :left-or-right :left))
    (roslisp:ros-info (debug-grasp) "Waiting for force to close gripper.")
    (cpl:wait-for (cpl:> (cpl:fl-funcall #'force-aggregated boxy-ll:*wrench-state-fluent*) 4.0))
    (roslisp:ros-info (debug-grasp) "Force detected, closing gripper.")
    (boxy-ll::move-gripper-joint :action-type-or-position :grip :left-or-right :left :effort 50)))

(defun reset-tf-client ()
  ;; (cram-tf::destroy-tf)
  ;; (cram-tf::init-tf)
  (setf cram-tf:*transformer* nil)
  (setf cram-tf:*transformer* (make-instance 'cl-tf2:buffer-client))
  )

(defun remove-obj-from-giskard (name)
  (declare (type symbol name))
  (values
   (giskard::call-giskard-environment-service
    :detach :name (roslisp-utilities:rosify-underscores-lisp-name name))
   (giskard::call-giskard-environment-service
    :remove :name (roslisp-utilities:rosify-underscores-lisp-name name))))

(defun reset (&optional noise)
  ;; (coe:clear-belief)
  (mapcar (alexandria:compose #'remove-obj-from-giskard #'first) *object-spawning-data*)
  (giskard::make-giskard-action-client)
  (add-kitchen-to-giskard)
  (spawn-objects-on-plate)
  (initialize-attachments)
  (when noise (add-noise)))

(defun add-noise (&optional (noise 0.1))
  (let ((plate-pose (btr:object-pose :big-wooden-plate))
        (noise-transform (cl-tf:make-transform
                          (cl-tf:make-3d-vector (- (* 2 (random noise)) noise) (- (* 2 (random noise)) noise) 0)
                          (cl-tf:make-identity-rotation))))
    (setf (btr:pose (btr:object btr:*current-bullet-world* :big-wooden-plate))
          (cl-tf:transform noise-transform plate-pose))
    (with-giskard-controlled-robot
      (review-all-objects))))

(defun review-all-objects ()
  (loop for ?object-name in (mapcar #'car *object-spawning-data*)
            do (exe:perform
                (desig:an action 
                          (type detecting)
                          (object (desig:an object (name ?object-name)))))))

(defun print-phases ())
  

(defun window ()
  (prolog `(and (btr:bullet-world ?w)
                    (btr:debug-window ?w))))  

(defgeneric pose*->btr-pose (pose)
  (:documentation "Returns pose or pose-stamped as btr-style list of orig and rot.")
  (:method ((pose cl-tf:pose))
    (with-slots ((o cl-tf:origin) (r cl-tf:orientation)) pose
        (with-slots ((x cl-tf:x) (y cl-tf:y) (z cl-tf:z)) o
           (with-slots ((ax cl-tf:x) (ay cl-tf:y) (az cl-tf:z) (w cl-tf:w)) r
             `(,(list x y z) ,(list ax ay az w))))))
  (:method ((pose cl-tf:pose-stamped))
    (pose*->btr-pose (cl-tf:pose-stamped->pose pose))))

(defgeneric pose-between-objects (object-target object-source)
  (:documentation "Pose of 'source' in 'target' regarding btr objects.")
  (:method ((object-target btr:object) (object-source btr:object))
    (cl-tf:transform->pose
     (cl-tf:transform* (cl-tf:transform-inv (cl-tf:pose->transform (btr:pose object-target))) 
                       (cl-tf:pose->transform (btr:pose object-source)))))
  (:method ((object-target symbol) (object-source symbol))
    (pose-between-objects (btr:object btr:*current-bullet-world* object-target)
                          (btr:object btr:*current-bullet-world* object-source))))


;;;;;;;;;;;;;;;;;;;
;; VISUALIZATION ;;

(defparameter *visualization-map* (make-hash-table))

(let ((vis-id 9000))
  (defun visualize-part (object-name &optional parent-frame)
    (declare (type symbol object-name))
    (let* ((mesh-key (car (btr:item-types (btr:object btr:*current-bullet-world* object-name))))
           (mesh-path (second (find mesh-key btr::*mesh-files* :key #'first)))
           (obj-pose (cl-tf:ensure-pose-stamped (btr-utils:object-pose object-name)
                                                cram-tf:*fixed-frame* 0.0))
           (pose (if parent-frame
                     (cram-tf:ensure-pose-in-frame obj-pose parent-frame)
                     obj-pose))
           (id (or (gethash object-name *visualization-map*)
                   (setf (gethash object-name *visualization-map*) (incf vis-id))))
           (color (third (assoc object-name *object-spawning-data*))))
      (cram-tf:visualize-marker
       pose
       :mesh-path mesh-path
       :id id
       :marker-type :MESH_RESOURCE
       :scale-list '(1 1 1)
       :r-g-b-list (cond ((eq 3 (length color)) (append color '(1.0)))
                         ((eq 4 (length color)) color)
                         ((alexandria:starts-with-subseq "HOLDER" (string object-name))
                          '(0.9 0.9 0.9 1))
                         ((eq 'BIG-WOODEN-PLATE object-name)
                          '(0.92156863 0.65882355 0.20392157 1))
                         (t '(0.3 0.7 0.3 1)))
       ;; :in-frame parent-frame
       :frame-locked T
       :topic "assembly_items/visualization_marker"))))

(defmethod coe:on-event visualize-attach-object ((event cpoe:object-attached-robot))
  "Attach marker to gripper link."
  (let* ((object-name
           (cpoe:event-object-name event))
         (link (cut:var-value
                '?ee-link
                (car (prolog:prolog
                      `(and (cram-robot-interfaces:robot ?robot)
                            (cram-robot-interfaces:end-effector-link
                             ?robot ,(cpoe:event-arm event)
                             ?ee-link)))))))
    (when (cut:is-var link)
      (error "[Assembly Vis] Couldn't find robot's EE link."))
    (unless (btr:object btr:*current-bullet-world* object-name)
      (error "[Assembly Vis] there was no corresponding btr object."))
    (visualize-part object-name link)))

(defmethod coe:on-event visualize-detach-object ((event cpoe:object-detached-robot))
  "Attach marker to gripper link."
  (let* ((object-name (cpoe:event-object-name event)))
    (unless (btr:object btr:*current-bullet-world* object-name)
      (error "[Assembly Vis] there was no corresponding btr object."))
    (visualize-part object-name)))

;; VISUALIZATION ;;
;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;
;; COLLISION ;;
(defun add-to-collision-scene (object-name)
  (declare (type symbol object-name))
  (giskard::call-giskard-environment-service
   :add
   :name (roslisp-utilities:rosify-underscores-lisp-name object-name)
   :pose (cl-tf:pose->pose-stamped "map" 0.0
                                   (btr:object-pose object-name))
   :dimensions (with-slots (cl-tf:x cl-tf:y cl-tf:z)
                   (btr:calculate-bb-dims (btr:object btr:*current-bullet-world* object-name))
                 (list cl-tf:x cl-tf:y cl-tf:z))))

(defun update-collision-scene (object-name)
  (let ((giskard-name (roslisp-utilities:rosify-underscores-lisp-name object-name)))
    (giskard::call-giskard-environment-service
     :remove
     :name giskard-name)
    (add-to-collision-scene object-name)))
;; COLLISION ;;
;;;;;;;;;;;;;;;
