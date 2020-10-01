
(setf (slot-value ?w 'gravity-vector) (cl-tf:make-3d-vector 0 0 0))

(prolog `(and
               (btr:bullet-world ?w)
               (btr:debug-window ?w)
               (btr:assert ?w (btr:object :static-plane 
                                          :floor ((0 0 0) (0 0 0 1))
                                          :normal (0 0 1) 
                                          :constant 0))))

(prolog `(and
          (bullet-world ?w)
          (debug-window ?w)
          (assert (object ?w :box-item :box-1 ((0 0 0.5)(0 0 0 1))
                                       :mass 1.0 
                                       :color (0.8 0.8 0.8)
                                       :size (0.5 0.5 0.5) 
                                       :item-type :box))))



(prolog `(and
          (bullet-world ?w)
          (debug-window ?w)
          (assert (object ?w :box-item :box-2 ((0 0.6 1.0) (0 0 0 1))
                                       :mass 1.0 
                                       :color (0.8 0.8 0.8)
                                       :size (0.1 0.1 0.1) 
                                       :item-type :box))))

(btr-utils:kill-object :box-2)

(setf (cl-bullet::collision-flags (car (rigid-bodies (object ?w :box-1))))
      '(:cf-static-object))

(setf (cl-bullet::collision-flags (car (rigid-bodies (object ?w :box-2))))
      '(:cf-kinematic-object))

;; is dynamic object initially, so dont need to make NIL here
(setf (cl-bullet::collision-flags (car (rigid-bodies (object ?w :box-2))))
      nil)

(loop for times to 10
      do (simulate btr:*current-bullet-world* 0.1)
         (sleep 0.1))

(cl-bullet:apply-central-force (car (rigid-bodies (object ?w :box-2)))
                               (cl-tf:make-3d-vector 0.0 0.0 -10.0))

(cl-bullet:total-force (car (rigid-bodies (object ?w :box-1))))

(cl-bullet:

;; prism
(make-item ?w
           :prism
           :prism-type
           (list (make-instance 'rigid-body
                                :name :prism
                                :mass 1 
                                :pose (btr:ensure-pose '((1 1 1) (0 0 0 1)))
                                :collision-shape (make-octagon-prism-shape 0.3 1))))


(let ((pos '(( 3 -3) ( 3 -2) ( 3 -1) ( 3  0) ( 3  1) ( 3  2) ( 3  3)
             ( 2 -3)                                         ( 2  3)
                  
                  
                  
             (-2 -3)                                         (-2  3)
             (-3 -3) (-3 -2) (-3 -1) (-3  0) (-3  1) (-3  2) (-3  3)))
           (compound-shape (make-instance 'compound-shape))
           (scale 0.21))
  (btr-utils:kill-object :chassis)
  (loop for (x y) in pos
        do (add-child-shape compound-shape
                            (cl-transforms:make-pose
                             (cl-tf:make-3d-vector (* scale x) (* scale y) 0.0)
                             (cl-tf:make-identity-rotation))
                            (make-instance
                             'bt-vis:colored-box-shape
                             :half-extents (cl-tf:v* 
                                            (cl-transforms:make-3d-vector 1 1 1)
                                            (/ scale 2))
                             :color '(0.7 0.7 0.5 1.0))))
  
  (make-item *current-bullet-world*
             :chassis
             :chassis
             (list (make-instance 
                    'rigid-body
                    :name :chassis
                    :mass 1 
                    :pose (btr:ensure-pose `((0 0 0) (0 0 0 1)))
                    :collision-shape  compound-shape))))
