(in-package :demo)

(defparameter *phases*
  `((:chassis :holder-plane-horizontal :horizontal-attachment)
    (:bottom-wing :chassis :wing-attachment)
    (:underbody :bottom-wing :body-attachment)
    ;; underbody -> rear-wing
    (:upper-body :underbody :body-on-body)
    (:bolt-1 :upper-body :rear-thread)
    (:top-wing :upper-body :wing-attachment)
    (:bolt-2 :top-wing :middle-thread)
    (:window :top-wing :window-attachment)
    (:bolt-3 :window :window-thread)
    (:top-wing :holder-plane-vertical :vertical-attachment)
    (:propeller :motor-grill :propeller-attachment)
    (:bolt-4 :propeller :propeller-thread)))

(defun prepare (phase &key (only nil) (reset nil))
  (declare (type number phase))
  (when reset (reset))
  
  )

(defun end-phase (phase &key (only nil) (reset nil))
  (declare (type number phase))
  (when reset (reset))
  (if (<= 0 phase (1- (length *phases*)))
      (flet ((attach (name other-name attachment)
               (btr:detach-all-objects (btr:object btr:*current-bullet-world* name))
               (cram-occasions-events:on-event
                  (make-instance 'cpoe:object-attached-object
                                 :object-name name :other-object-name other-name
                                 :attachment-type attachment))
               (visualize-part name)
               (update-collision-scene name)))
        (if only
            (destructuring-bind (name other-name attachment) (nth phase *phases*)
              (attach name other-name attachment))
            (loop for n to phase
                  do (apply #'attach (nth n *phases*))
                     ;; (when (eq n 2) (attach 'underbody 'rear-wing :loose))
                  )))
      (roslisp:ros-warn (assembly phase-mock) "There is no phase ~a." phase)))

(defun chassis-in-hand ()
  (let* ((pose-in-map
           (cram-tf:ensure-pose-in-frame
            (cl-tf:pose->pose-stamped
             "left_arm_7_link" 0.0
             (btr:ensure-pose `((0.001d0 -0.001d0 0.298d0)
                                (0.71405d0 0.7d0 0.006d0 0.005d0))))
            "map"))
         (btr-pose (pose*->btr-pose pose-in-map)))
    (with-giskard-controlled-robot
      (when (btr:attached-objects (btr:get-robot-object))
        (cram-occasions-events:on-event
         (make-instance 'cpoe:object-detached-robot
                        :arm :left
                        :object-name :chassis)))
      
      (btr:detach-all-objects (btr:object btr:*current-bullet-world* :chassis))
      (setf (btr:pose (btr:object btr:*current-bullet-world* :chassis))
            pose-in-map)
      (exe:perform
       (desig:an action
                (type detecting)
                (object (desig:an object (name :chassis)))))
      (sleep 1)
      (cram-occasions-events:on-event
       (make-instance 'cpoe:object-attached-robot
                      :grasp :top
                      :arm :left
                      :object-name :chassis))
      ;; (exe:perform
      ;;  (desig:an action
      ;;           (type detecting)
      ;;           (object (desig:an object (name :chassis)))))
      )))

(defun bw-in-hand ()
  (let* ((pose-in-map
           (cram-tf:ensure-pose-in-frame
            (cl-tf:pose->pose-stamped
             "left_arm_7_link" 0.0
             (btr:ensure-pose `((0.0d0 -0.03d0 0.38d0)
                                ( 0.0d0 -0.7071067811865475d0 0.0d0 0.7071067811865475d0))))
            "map"))
         ;; (btr-pose (pose*->btr-pose pose-in-map))
         )
    (with-giskard-controlled-robot
      (when (btr:attached-objects (btr:get-robot-object))
        (cram-occasions-events:on-event
         (make-instance 'cpoe:object-detached-robot
                        :arm :left
                        :object-name :chassis)))
      
      (btr:detach-all-objects (btr:object btr:*current-bullet-world* :bottom-wing))
      (setf (btr:pose (btr:object btr:*current-bullet-world* :bottom-wing))
            pose-in-map)
      (exe:perform
       (desig:an action
                (type detecting)
                (object (desig:an object (name :bottom-wing)))))
      (sleep 1)
      (cram-occasions-events:on-event
       (make-instance 'cpoe:object-attached-robot
                      :grasp :right-side
                      :arm :left
                      :object-name :bottom-wing))
      ;; (exe:perform
      ;;  (desig:an action
      ;;           (type detecting)
      ;;           (object (desig:an object (name :chassis)))))
      )))

(defun underb-in-hand ()
  (let* ((pose-in-map
           (cram-tf:ensure-pose-in-frame
            (cl-tf:pose->pose-stamped
             "left_gripper_tool_frame" 0.0
             (btr:ensure-pose `((0.0d0 0.01d0 0.0d0)
                                (0.7071067811865475d0 0.7071067811865475d0 0.0d0 0.0d0))))
            "map")))
    (with-giskard-controlled-robot
      (when (btr:attached-objects (btr:get-robot-object))
        (cram-occasions-events:on-event
         (make-instance 'cpoe:object-detached-robot
                        :arm :left
                        :object-name :underbody)))
      
      (btr:detach-all-objects (btr:object btr:*current-bullet-world* :underbody))
      (setf (btr:pose (btr:object btr:*current-bullet-world* :underbody))
            pose-in-map)
      (exe:perform
       (desig:an action
                (type detecting)
                (object (desig:an object (name :underbody)))))
      (sleep 1)
      (cram-occasions-events:on-event
       (make-instance 'cpoe:object-attached-robot
                      :grasp :top
                      :arm :left
                      :object-name :underbody))
      ;; (exe:perform
      ;;  (desig:an action
      ;;           (type detecting)
      ;;           (object (desig:an object (name :chassis)))))
      )))

(defun upperb-in-hand ()
  (let* ((pose-in-map
           (cram-tf:ensure-pose-in-frame
            (cl-tf:pose->pose-stamped
             "left_gripper_tool_frame" 0.0
             (btr:ensure-pose `((0.0d0 -0.012d0 -0.02d0)
                                (0.7071067811865475d0 -0.7071067811865475d0 0.0d0 0.0d0))))
            "map")))
    (with-giskard-controlled-robot
      (when (btr:attached-objects (btr:get-robot-object))
        (cram-occasions-events:on-event
         (make-instance 'cpoe:object-detached-robot
                        :arm :left
                        :object-name :upper-body)))
      
      (btr:detach-all-objects (btr:object btr:*current-bullet-world* :upper-body))
      (setf (btr:pose (btr:object btr:*current-bullet-world* :upper-body))
            pose-in-map)
      (exe:perform
       (desig:an action
                (type detecting)
                (object (desig:an object (name :upper-body)))))
      (sleep 1)
      (cram-occasions-events:on-event
       (make-instance 'cpoe:object-attached-robot
                      :grasp :top
                      :arm :left
                      :object-name :upper-body)))))

(defun bolt-in-hand ()
  (let* ((pose-in-map
           (cram-tf:ensure-pose-in-frame
            (cl-tf:pose->pose-stamped
             "left_gripper_tool_frame" 0.0
             (btr:ensure-pose `((0.0d0 0.0d0 0.023d0)
                                (0.7071067811865475d0 -0.7071067811865475d0 0.0d0 0.0d0))))
            "map")))
    (with-giskard-controlled-robot
      (when (btr:attached-objects (btr:get-robot-object))
        (cram-occasions-events:on-event
         (make-instance 'cpoe:object-detached-robot
                        :arm :left
                        :object-name :bolt-1)))
      
      (btr:detach-all-objects (btr:object btr:*current-bullet-world* :bolt-1))
      (setf (btr:pose (btr:object btr:*current-bullet-world* :bolt-1))
            pose-in-map)
      (exe:perform
       (desig:an action
                (type detecting)
                (object (desig:an object (name :bolt-1)))))
      (sleep 1)
      (cram-occasions-events:on-event
       (make-instance 'cpoe:object-attached-robot
                      :grasp :top
                      :arm :left
                      :object-name :bolt-1)))))
