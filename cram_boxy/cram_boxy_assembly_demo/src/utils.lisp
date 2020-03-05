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

(defparameter *visualization-map* (make-hash-table))

(let ((vis-id 9000))
  (defun visualize-part (object-name &optional object-color parent-frame)
    (declare (type symbol object-name))
    (let* ((mesh-key (car (btr:item-types (btr:object btr:*current-bullet-world* object-name))))
           (mesh-path (second (find mesh-key btr::*mesh-files* :key #'first)))
           (obj-pose (cl-tf:ensure-pose-stamped (btr-utils:object-pose object-name)
                                                cram-tf:*fixed-frame* 0.0))
           (pose (if parent-frame
                     (cram-tf:ensure-pose-in-frame obj-pose parent-frame)
                     obj-pose))
           (id (or (gethash object-name *visualization-map*)
                   (setf (gethash object-name *visualization-map*) (incf vis-id)))))
      (cram-tf:visualize-marker
       pose
       :mesh-path mesh-path
       :id id
       :marker-type :MESH_RESOURCE
       :scale-list '(1 1 1)
       :r-g-b-list (cond ((eq 3 (length object-color))
                          (append object-color '(1.0)))
                         ((eq 4 (length object-color))
                          object-color)
                         ((alexandria:starts-with-subseq "HOLDER" (string object-name))
                          '(0.9 0.9 0.9 1))
                         ((eq 'BIG-WOODEN-PLATE object-name)
                          '(0.92156863 0.65882355 0.20392157 1))
                         (t
                          '(0.3 0.7 0.3 1)))
       ;; :in-frame parent-frame
       :frame-locked T))))

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
    (visualize-part object-name
                    (third (assoc object-name *object-spawning-data*))
                    link)))

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
