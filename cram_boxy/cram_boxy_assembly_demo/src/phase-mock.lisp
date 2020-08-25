(in-package :demo)

(defun prepare (&key (all-preceeding t) (reset nil))
  (when reset (reset))
  (format t "Preparing all-preceeding phases: ~a~%" all-preceeding)
  (format t "Resetting before execution: ~a~%" reset)
  (format t "Choose phase to prepare:~%")
  (loop for phase in *assembly-steps*
        for id to (1- (length *assembly-steps*))
        do (case (car phase)
             (go-connect (format t "~a) ~a on ~a~%" id (second phase) (car (last phase))))
             (btr:attach-object (format t "~a) ~a~%" id phase))
             (:otherwise (format t "not-preparable - ~a~%" phase))))
  (flet ((attach (name other-name attachment)
           (btr:detach-all-objects (btr:object btr:*current-bullet-world* name))
           (cram-occasions-events:on-event
            (make-instance 'cpoe:object-attached-object
                           :object-name name :other-object-name other-name
                           :attachment-type attachment))
           (visualize-part name)
           (update-collision-scene name)))
    (let ((phase-id (read)))
      (if (<= 0 phase-id (1- (length *assembly-steps*)))
          (if all-preceeding
              (loop for pred-id to (1- phase-id)
                    for pred = (nth pred-id *assembly-steps*)
                    do (case (car pred)
                         (go-connect (apply #'attach (mapcar (alexandria:rcurry #'nth pred) '(1 3 5))))
                         (btr:attach-object (apply #'btr:attach-object (cdr pred)))))
              (let ((phase (nth phase-id *assembly-steps*)))
                (case (car phase)
                  (go-connect (apply #'attach (mapcar (alexandria:rcurry #'nth phase) '(1 3 5))))
                  (btr:attach-object (apply #'btr:attach-object (cdr phase)))
                  (:otherwise (roslisp:ros-warn (assembly phase-mock) "Can't prepare ~a." phase-id)))))
          (roslisp:ros-warn (assembly phase-mock) "Phase ~a out of range." phase-id)))))

(defun mock-in-hand (?object-type)
  (with-giskard-controlled-robot
        (let* ((?object-designator 
                 (exe:perform
                  (desig:a motion
                           (type world-state-detecting)
                           (object (desig:an object (type ?object-type))))))
               (manipulation-data
                 (cut:lazy-car
                  (prolog:prolog
                  `(and 
                    (desig:current-designator ,?object-designator ?current-object-desig)
                    ;; (spec:property ?current-object-desig (:type ?object-type))
                    (spec:property ?current-object-desig (:name ?object-name))
                    (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)
                    (and (lisp-fun man-int:get-action-grasps ,?object-type :left ?object-transform ?grasps)
                         (member ?grasp ?grasps))))))
               (?name (cut:var-value '?object-name manipulation-data))
               (grasp (cut:var-value '?grasp manipulation-data))
               (pose-in-map
                 (cram-tf:multiply-transform-stampeds
                  "map" (roslisp-utilities:rosify-underscores-lisp-name ?name) 
                  (cl-tf:lookup-transform cram-tf:*transformer* "map" "left_gripper_tool_frame")
                  (cram-tf:transform-stamped-inv
                   (man-int:get-object-type-to-gripper-transform ?object-type ?name :left grasp))
                  :result-as-pose-or-transform :pose)))
            (when (btr:attached-objects (btr:get-robot-object))
              (btr:detach-all-objects (btr:get-robot-object)))
            (btr:detach-all-objects (btr:object btr:*current-bullet-world* ?name))
            (setf (btr:pose (btr:object btr:*current-bullet-world* ?name))
                  pose-in-map)
            (exe:perform
             (desig:an action
                       (type detecting)
                       (object (desig:an object (name ?name)))))
            (sleep 1)
            (cram-occasions-events:on-event
             (make-instance 'cpoe:object-attached-robot
                            :grasp grasp
                            :arm :left
                            :object-name ?name)))))

#+deprecated
(
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
                      :object-name :chassis)))))

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
)
