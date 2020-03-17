(in-package :demo)

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

(defun end- (phase)
  (declare (type number phase))
  (let ((phase-fun-name (intern (format nil "END-~a" phase)))
        phase-fun)
    (handler-case (setf phase-fun (symbol-function phase-fun-name))
      (undefined-function () (roslisp:ros-warn (assembly phase-mock) "no mock called ~a" phase-fun-name)))
    (when phase-fun
      (with-giskard-controlled-robot
        (btr:detach-all-objects (btr:get-robot-object))
        (funcall phase-fun)))))
 

(defun end-1 ()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-attached-object
                  :object-name 'chassis
                  :other-object-name 'holder-plane-horizontal
                  :attachment-type :horizontal-attachment))
  (visualize-part 'chassis)
  (update-collision-scene 'chassis))


(defun end-2 ()
  (end-1)
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-attached-object
                  :object-name 'bottom-wing
                  :other-object-name 'chassis
                  :attachment-type :wing-attachment))
  (visualize-part 'bottom-wing)
  (update-collision-scene 'bottom-wing))



(defun end-3 ()
  (end-2)
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-attached-object
                  :object-name 'underbody
                  :other-object-name 'bottom-wing
                  :attachment-type :body-attachment))
  (visualize-part 'underbody)
  (update-collision-scene 'underbody))

(defun end-4 ()
  (end-3)
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-attached-object
                  :object-name 'upper-body
                  :other-object-name 'underbody
                  :attachment-type :body-on-body))
  (visualize-part 'upper-body)
  (update-collision-scene 'upper-body))
