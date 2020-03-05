(in-package :demo)

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

(defun reset ()
  (mapcar (alexandria:compose #'remove-obj-from-giskard #'first) *object-spawning-data*)
  (giskard::make-giskard-action-client)
  (spawn-objects-on-plate))

(defun chassis-in-hand ()
  (let* ((pose-in-map
           (cram-tf:ensure-pose-in-frame
            (cl-tf:pose->pose-stamped
             "left_arm_7_link" 0.0
             (btr:ensure-pose `((0.001d0 -0.001d0 0.298d0)
                                (0.71405d0 0.7d0 0.006d0 0.005d0))))
            "map"))
         (btr-pose (pose-stamped->btr-pose pose-in-map)))
    (when (btr:attached-objects (btr:get-robot-object))
      (cram-occasions-events:on-event
       (make-instance 'cpoe:object-detached-robot
                      :arm :left
                      :object-name 'chassis)))
    (btr-utils:spawn-object 'chassis :chassis :pose btr-pose)
    (cram-occasions-events:on-event
     (make-instance 'cpoe:object-attached-robot
                    :arm :left
                    :object-name 'chassis))))
    

(defgeneric pose-stamped->btr-pose (pose)
  (:method ((pose cl-tf:pose-stamped))
    (with-slots ((o cl-tf:origin) (r cl-tf:orientation)) pose
        (with-slots ((x cl-tf:x) (y cl-tf:y) (z cl-tf:z)) o
           (with-slots ((ax cl-tf:x) (ay cl-tf:y) (az cl-tf:z) (w cl-tf:w)) r
             `(,(list x y z) ,(list ax ay az w)))))))


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

#+desig-for-moving-camera-to-pose
(let ((?goal-pose (cl-tf:make-pose-stamped
                   "base_footprint" 0.0
                   (cl-tf:make-3d-vector 1.0 0.0 2.0)
                   (cl-tf:make-quaternion 0.8660254037844386 0 0.5 0))))
  (with-giskard-controlled-robot
    (exe:perform
     (desig:an action
               (type looking)
               (target (desig:a location
                                (pose ?goal-pose)))))))
