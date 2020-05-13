(in-package :demo)

(defun pose->transform-stamped-in-base (pose child-frame-lispy)
  (let ((target-pose-in-base
          (cram-tf:ensure-pose-in-frame
           pose cram-tf:*robot-base-frame* :use-zero-time t))
        (child-frame-rosy
          (roslisp-utilities:rosify-underscores-lisp-name child-frame-lispy)))
    (cram-tf:pose-stamped->transform-stamped target-pose-in-base child-frame-rosy)))

(def-fact-group assembly-plans (desig:action-grounding)

  (<- (desig:action-grounding ?action-designator (assemble ?resolved-action-designator))
    (spec:property ?action-designator (:type :assembling))

    ;; find in which hand the object is
    (-> (spec:property ?action-designator (:arm ?arm))
        (-> (spec:property ?action-designator (:object ?object-designator))
            (or (cpoe:object-in-hand ?object-designator ?arm)
                (format "WARNING: Wanted to place an object ~a with arm ~a, ~
                         but it's not in the arm.~%" ?object-designator ?arm))
            (cpoe:object-in-hand ?object-designator ?arm))
        (-> (spec:property ?action-designator (:object ?object-designator))
            (or (cpoe:object-in-hand ?object-designator ?arm)
                (format "WARNING: Wanted to place an object ~a ~
                         but it's not in any of the hands.~%" ?object-designator))
            (cpoe:object-in-hand ?object-designator ?arm)))

    ;;; infer missing information
    (desig:current-designator ?object-designator ?current-object-designator)
    (spec:property ?current-object-designator (:type ?object-type))
    (spec:property ?current-object-designator (:name ?object-name))
    (lisp-fun man-int:get-action-gripper-opening ?object-type ?gripper-opening)

    ;; take object-pose from action-designator :target otherwise from object-designator pose
    (-> (spec:property ?action-designator (:target ?location-designator))
        (and (desig:current-designator ?location-designator ?current-location-designator)
             (desig:designator-groundings ?current-location-designator ?poses)
             (member ?target-object-pose ?poses)
             (lisp-fun pose->transform-stamped-in-base ?target-object-pose ?object-name
                       ?target-object-transform))
        (and (lisp-fun man-int:get-object-old-transform ?current-object-designator
                       ?target-object-transform)
             (lisp-fun man-int:get-object-old-pose ?current-object-designator
                       ?target-object-pose)
             (desig:designator :location ((:pose ?target-object-pose))
                               ?current-location-designator)))

    ;; placing happens on/in an object
    (or (desig:desig-prop ?current-location-designator (:on ?other-object-designator))
        (desig:desig-prop ?current-location-designator (:in ?other-object-designator))
        (equal ?other-object-designator NIL))
    (-> (desig:desig-prop ?current-location-designator (:attachment ?placement-location-name))
        (true)
        (equal ?placement-location-name NIL))

    (-> (spec:property ?action-designator (:grasp ?grasp))
        (true)
        (cpoe:object-in-hand ?object-designator ?arm ?grasp))

    ;; calculate trajectory
    (equal ?objects (?current-object-designator))
    (-> (equal ?arm :left)
        (and (lisp-fun man-int:get-action-trajectory
                       :placing ?arm ?grasp ?objects
                       :target-object-transform-in-base ?target-object-transform
                       ?left-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :reaching
                       ?left-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :putting
                       ?left-put-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :retracting
                       ?left-retract-poses))
        (and (equal ?left-reach-poses NIL)
             (equal ?left-put-poses NIL)
             (equal ?left-retract-poses NIL)))
    (-> (equal ?arm :right)
        (and (lisp-fun man-int:get-action-trajectory
                       :placing ?arm ?grasp ?objects
                       :target-object-transform-in-base ?target-object-transform
                       ?right-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :reaching
                       ?right-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :putting
                       ?right-put-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :retracting
                       ?right-retract-poses))
        (and (equal ?right-reach-poses NIL)
             (equal ?right-put-poses NIL)
             (equal ?right-retract-poses NIL)))
    (or (lisp-pred identity ?left-trajectory)
        (lisp-pred identity ?right-trajectory))

    ;; Use giskard constraints if available
    ;; (once (or (spec:property ?action-designator (:constraints ?constraints))
    ;;           (equal ?constraints nil)))
    (-> (spec:property ?action-designator (:constraints ?constraints))
        (true)
        (equal ?constraints nil)
        ;; (and (rob-int:robot ?robot)
        ;;      (rob-int:gripper-joint ?robot ?arm ?joint)
        ;;      (equal ?constraints ((?joint) (?gripper-opening))))
        )
    
    
    ;; put together resulting designator
    (desig:designator :action ((:type :assembling)
                               (:object ?current-object-designator)
                               (:other-object ?other-object-designator)
                               (:arm ?arm)
                               (:gripper-opening ?gripper-opening)
                               (:target ?current-location-designator)
                               (:attachment-type ?placement-location-name)
                               (:left-reach-poses ?left-reach-poses)
                               (:right-reach-poses ?right-reach-poses)
                               (:left-put-poses ?left-put-poses)
                               (:right-put-poses ?right-put-poses)
                               (:left-retract-poses ?left-retract-poses)
                               (:right-retract-poses ?right-retract-poses)
                               (:constraints ?constraints))
                      ?resolved-action-designator))

  (<- (desig:action-grounding ?action-designator (screw ?resolved-action-designator))
    (spec:property ?action-designator (:type :screwing))

    ;; find in which hand the object is
    (-> (spec:property ?action-designator (:arm ?arm))
        (-> (spec:property ?action-designator (:object ?object-designator))
            (or (cpoe:object-in-hand ?object-designator ?arm)
                (format "WARNING: Wanted to place an object ~a with arm ~a, ~
                         but it's not in the arm.~%" ?object-designator ?arm))
            (cpoe:object-in-hand ?object-designator ?arm))
        (-> (spec:property ?action-designator (:object ?object-designator))
            (or (cpoe:object-in-hand ?object-designator ?arm)
                (format "WARNING: Wanted to place an object ~a ~
                         but it's not in any of the hands.~%" ?object-designator))
            (cpoe:object-in-hand ?object-designator ?arm)))

    ;;; infer missing information
    (desig:current-designator ?object-designator ?current-object-designator)
    (spec:property ?current-object-designator (:type ?object-type))
    (spec:property ?current-object-designator (:name ?object-name))
    (lisp-fun man-int:get-action-gripper-opening ?object-type ?gripper-opening)

    ;; take object-pose from action-designator :target otherwise from object-designator pose
    (-> (spec:property ?action-designator (:target ?location-designator))
        (and (desig:current-designator ?location-designator ?current-location-designator)
             (desig:designator-groundings ?current-location-designator ?poses)
             (member ?target-object-pose ?poses)
             (lisp-fun pose->transform-stamped-in-base ?target-object-pose ?object-name
                       ?target-object-transform))
        (and (lisp-fun man-int:get-object-old-transform ?current-object-designator
                       ?target-object-transform)
             (lisp-fun man-int:get-object-old-pose ?current-object-designator
                       ?target-object-pose)
             (desig:designator :location ((:pose ?target-object-pose))
                               ?current-location-designator)))

    ;; placing happens on/in an object
    (or (desig:desig-prop ?current-location-designator (:on ?other-object-designator))
        (desig:desig-prop ?current-location-designator (:in ?other-object-designator))
        (equal ?other-object-designator NIL))
    (-> (desig:desig-prop ?current-location-designator (:attachment ?placement-location-name))
        (true)
        (equal ?placement-location-name NIL))

    (-> (spec:property ?action-designator (:grasp ?grasp))
        (true)
        (equal ?grasp :top))

    ;; calculate trajectory
    (equal ?objects (?current-object-designator))
    (-> (equal ?arm :left)
        (and (lisp-fun man-int:get-action-trajectory
                       :placing ?arm ?grasp ?objects
                       :target-object-transform-in-base ?target-object-transform
                       ?left-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :reaching
                       ?left-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :putting
                       ?left-put-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :retracting
                       ?left-retract-poses))
        (and (equal ?left-reach-poses NIL)
             (equal ?left-put-poses NIL)
             (equal ?left-retract-poses NIL)))
    (-> (equal ?arm :right)
        (and (lisp-fun man-int:get-action-trajectory
                       :placing ?arm ?grasp ?objects
                       :target-object-transform-in-base ?target-object-transform
                       ?right-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :reaching
                       ?right-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :putting
                       ?right-put-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :retracting
                       ?right-retract-poses))
        (and (equal ?right-reach-poses NIL)
             (equal ?right-put-poses NIL)
             (equal ?right-retract-poses NIL)))
    (or (lisp-pred identity ?left-trajectory)
        (lisp-pred identity ?right-trajectory))

    ;; Use giskard constraints if available
    ;; (once (or (spec:property ?action-designator (:constraints ?constraints))
    ;;           (equal ?constraints nil)))
    (-> (spec:property ?action-designator (:constraints ?constraints))
        (true)
        (equal ?constraints nil)
        ;; (and (rob-int:robot ?robot)
        ;;      (rob-int:gripper-joint ?robot ?arm ?joint)
        ;;      (equal ?constraints ((?joint) (?gripper-opening))))
        )
    
    
    ;; put together resulting designator
    (desig:designator :action ((:type :screwing)
                               (:object ?current-object-designator)
                               (:other-object ?other-object-designator)
                               (:arm ?arm)
                               (:gripper-opening ?gripper-opening)
                               (:target ?current-location-designator)
                               (:attachment-type ?placement-location-name)
                               (:left-reach-poses ?left-reach-poses)
                               (:right-reach-poses ?right-reach-poses)
                               (:left-put-poses ?left-put-poses)
                               (:right-put-poses ?right-put-poses)
                               (:left-retract-poses ?left-retract-poses)
                               (:right-retract-poses ?right-retract-poses)
                               (:constraints ?constraints))
                      ?resolved-action-designator))
  
  (<- (desig:action-grounding ?action-designator (screw
                                                  ?resolved-action-designator))
    (or (spec:property ?action-designator (:type :screwing)))
    (once (or (spec:property ?action-designator (:object ?object))
              (equal ?left-poses nil)))
    (once (or (spec:property ?action-designator (:left-poses ?left-poses))
              (equal ?left-poses nil)))
    (once (or (spec:property ?action-designator (:right-poses ?right-poses))
              (equal ?right-poses nil)))
    (once (or (spec:property ?action-designator (:constraints ?constraints))
              (equal ?constraints nil)))
    (desig:designator :action ((:type :screwing)
                               (:object ?object)
                               (:left-poses ?left-poses)
                               (:right-poses ?right-poses)
                               (:collision-mode :allow-all)
                               (:constraints ?constraints))
                      ?resolved-action-designator)))
