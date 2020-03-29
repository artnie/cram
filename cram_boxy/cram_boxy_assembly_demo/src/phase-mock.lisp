(in-package :demo)

(defparameter *phases*
  `((chassis holder-plane-horizontal :horizontal-attachment)
    (bottom-wing chassis :wing-attachment)
    (underbody bottom-wing :body-attachment)
    ;; underbody -> rear-wing
    (upper-body underbody :body-on-body)
    (bolt-1 upper-body :rear-thread)
    (top-wing upper-body :wing-attachment)
    (bolt-2 top-wing :middle-thread)
    (window top-wing :window-attachment)
    (bolt-3 window :window-thread)
    (top-wing holder-plane-vertical :vertical-attachment)
    (propeller motor-grill :propeller-attachment)
    (bolt-4 propeller :propeller-thread)))

(defun end-phase (phase &key (only nil) (reset nil))
  (declare (type number phase))
  (when reset (reset))
  (if (<= 0 phase (1- (length *phases*)))
      (flet ((attach (name other-name attachment)
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
                        :object-name 'chassis)))
      (btr-utils:spawn-object 'chassis :chassis :pose btr-pose)
      (cram-occasions-events:on-event
       (make-instance 'cpoe:object-attached-robot
                      :arm :left
                      :object-name 'chassis))
      (exe:perform
       (desig:a motion
                (type world-state-detecting)
                (object (desig:an object (type :chassis))))))))

