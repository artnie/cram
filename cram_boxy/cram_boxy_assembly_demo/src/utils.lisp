(in-package :demo)

(defun reset-tf-client ()
  ;; (cram-tf::destroy-tf)
  ;; (cram-tf::init-tf)
  (setf cram-tf:*transformer* nil)
  (setf cram-tf:*transformer* (make-instance 'cl-tf2:buffer-client))
  )

(defparameter *visualization-map* (make-hash-table))

(let ((vis-id 0))
  (defun visualize-part (object-name)
    (declare (type symbol object-name))
    (let* ((mesh-key (car (btr:item-types (btr:object btr:*current-bullet-world* object-name))))
           (mesh-path (second (find mesh-key btr::*mesh-files* :key #'first)))
           (obj-pose (btr-utils:object-pose object-name))
           (pose (cl-tf:ensure-pose-stamped obj-pose "map" 0.0))
           (id (or (gethash object-name *visualization-map*)
                   (setf (gethash object-name *visualization-map*) (incf vis-id)))))
      (cram-tf:visualize-marker
       pose :mesh-path mesh-path
            :id id
            :marker-type :MESH_RESOURCE
            :scale-list '(1 1 1)
            :r-g-b-list (cond ((alexandria:starts-with-subseq "HOLDER" (string object-name))
                               '(0.9 0.9 0.9 1))
                              ((eq 'BIG-WOODEN-PLATE object-name)
                               '(0.92156863 0.65882355 0.20392157 1))
                              (t
                               '(0.3 0.7 0.3 1)))))))



;; (defparameter *head-top-goal*
;;   (cl-tf:make-pose-stamped
;;    "base_footprint" 0.0
;;    (cl-tf:make-3d-vector 0.65 0.0 2.3)
;;    (cl-tf:make-quaternion 0.0d0 -0.7071067811865475d0 0.0d0 0.7071067811865476d0)))

;; (defparameter *head-mid-top-goal*
;;   (cl-tf:make-pose-stamped
;;    "neck_base" 0.0
;;    (cl-tf:make-3d-vector 0.08 -0.115 0.457)
;;    (cl-tf:euler->quaternion :ay (- (/ pi 2)) :az (- (/ pi 2)))
;;    ;; "base_footprint" 0.0
;;    ;; (cl-tf:make-3d-vector 0.65 0.0 2.1)
;;    ;; (cl-tf:make-quaternion 0.0d0 -0.7071067811865475d0 0.0d0 0.7071067811865476d0)
;;    ))

;; (defparameter *head-perceive-pre-pre-front-goal*
;;   (cl-tf:make-pose-stamped
;;    "neck_base" 0.0
;;    (cl-tf:make-3d-vector 0.08 -0.34 0.32)
;;    (cl-tf:euler->quaternion :ay (- (/ pi 3)) :az (- (/ pi 2)))
;;    ;; (cl-tf:make-quaternion -0.179 -0.180 -0.684 0.684)
;;    ))

;; (defparameter *head-perceive-pre-front-goal*
;;   (cl-tf:make-pose-stamped
;;    "neck_base" 0.0
;;    (cl-tf:make-3d-vector 0.08 -0.38 0.25)
;;    (cl-tf:euler->quaternion :ay (- (/ pi 3)) :az (- (/ pi 2)))
;;    ;; (cl-tf:make-quaternion -0.179 -0.180 -0.684 0.684)
;;    ))

;; (defparameter *head-perceive-front-goal*
;;   (cl-tf:make-pose-stamped
;;    "neck_base" 0.0
;;    (cl-tf:make-3d-vector 0.08 -0.362 0.163)
;;    (cl-tf:euler->quaternion :ay (- (/ pi 6)) :az (- (/ pi 2)))
;;    ;; (cl-tf:make-quaternion -0.179 -0.180 -0.684 0.684)
;;    ))

;; (defun move-head-cartesian ()
;;   (let ((goal-pose (cl-tf:make-pose-stamped
;;                     "neck_base" 0.0
;;                     (cl-tf:make-3d-vector 0.0 -0.25 0.4)
;;                     (cl-tf:make-quaternion 0.5 0.5 0.5 -0.5)))
;;         (top-goal-pose
;;            (cl-tf:make-pose-stamped
;;                     "base_footprint" 0.0
;;                     (cl-tf:make-3d-vector 0.65 0.0 2.3)
;;                     (cl-tf:make-quaternion 0.0d0 -0.7071067811865475d0 0.0d0 0.7071067811865476d0))))
;;     (giskard:call-giskard-cartesian-action :goal-pose-left goal-pose
;;                                            :action-timeout 60
;;                                            :pose-base-frame "neck_base"
;;                                            :left-tool-frame "neck_ee_link")))

;; (defun move-head-sequence-perceive-pose ()
;;   (let* ((poses (mapcar (lambda (?goal-pose)
;;                           (let ((?goal-location (desig:a location
;;                                                          (pose ?goal-pose))))
;;                             `(exe:perform
;;                               (desig:an action
;;                                         (type looking)
;;                                         (target ,?goal-location)))))
;;                           (list *head-mid-top-goal*
;;                                 *head-perceive-pre-pre-front-goal*
;;                                 *head-perceive-pre-front-goal*
;;                                 *head-perceive-front-goal*)))
;;          (seq-sexp (push 'cpl:seq poses))
;;          ;; (?top-pose *head-top-goal*)
;;          ;; (?mid-top-pose *head-mid-top-goal*)
;;          ;; (?pre-pre-pose *head-perceive-pre-pre-front-goal*)
;;          ;; (?pre-pose *head-perceive-pre-front-goal*)
;;          ;; (?goal-pose *head-perceive-front-goal*)
;;     )
;;     (boxy-pm:with-real-robot
;;       (eval seq-sexp))))



#+desig-for-moving-camera-to-pose
(let ((?goal-pose (cl-tf:make-pose-stamped
                    "base_footprint" 0.0
                    (cl-tf:make-3d-vector 1.0 0.0 2.0)
                    (cl-tf:make-quaternion 0.8660254037844386 0 0.5 0))))
        (boxy-pm:with-real-robot
          (exe:perform
           (desig:an action
                     (type looking)
                     (target (desig:a location
                                      (pose ?goal-pose)))))))
