;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
(in-package :demo)

;; (defparameter *plate-x* -1.1906195322672526d0) 
(defparameter *plate-y* 1.6992865244547526d0) ;; 1.6)
;; (defparameter *plate-z* 0.8676) ;;0.884d0);;

(defparameter *plate-x* -1.1106149037679036d0)
;; (defparameter *plate-y* 1.6)
(defparameter *plate-z* 0.8861499150594075d0)

#+setting-for-board-when-other-pos-were-verified
(
(defparameter *plate-x* -1.1571736653645834d0)
(defparameter *plate-y* 1.6363941192626954d0)
(defparameter *plate-z* 0.8626)
)

(defparameter *plate-rad-x* 0.36)
(defparameter *plate-rad-y* 0.60)
(defparameter *plate-rad-z* 0.008)
(defparameter *holder-bolt-rad-x* 0.05)
(defparameter *holder-bolt-rad-y* 0.025)
(defparameter *holder-bolt-rad-z* 0.0125)
(defparameter *bolt-rad-x* 0.01)
(defparameter *bolt-rad-y* 0.01)
(defparameter *bolt-rad-z* 0.0225)
(defparameter *holder-upperbody-rad-x* 0.085)
(defparameter *holder-upperbody-rad-y* 0.025)
(defparameter *holder-upperbody-rad-z* 0.021)
(defparameter *holder-bottom-wing-rad-x* 0.035)
(defparameter *holder-bottom-wing-rad-y* 0.045)
(defparameter *holder-bottom-wing-rad-z* 0.045)
(defparameter *holder-underbody-rad-x* 0.0925)
(defparameter *holder-underbody-rad-y* 0.025)
(defparameter *holder-underbody-rad-z* 0.024)
(defparameter *holder-plane-horizontal-rad-x* 0.208)
(defparameter *holder-plane-horizontal-rad-y* 1.099)
(defparameter *holder-plane-horizontal-rad-z* 0.1202)
(defparameter *holder-window-rad-x* 0.026)
(defparameter *holder-window-rad-y* 0.025)
(defparameter *holder-window-rad-z* 0.017)
(defparameter *front-wheel-rad-z* 0.015)
(defparameter *nut-rad-z* 0.0075)
(defparameter *chassis-rad-z* 0.0515)
(defparameter *holder-plane-vertical-rad-x* 0.065)
(defparameter *holder-plane-vertical-rad-y* 0.06)
(defparameter *holder-plane-vertical-rad-z* 0.0175)
(defparameter *holder-top-wing-rad-x* 0.035)
(defparameter *holder-top-wing-rad-y* 0.0415)
(defparameter *holder-top-wing-rad-z* 0.045)

(defparameter *yellow-plastic* '(0.949 0.807 0.082))
(defparameter *gray-plastic* '(0.568 0.592 0.584))
(defparameter *red-plane* '(0.878 0.376 0.243))
(defparameter *cyan-plane* '(0.078 0.513 0.541))
(defparameter *yellow-plane* '(0.964 0.847 0.584))
(defparameter *transparent-plane* '(1 1 1))
(defparameter *black-plane* '(0.184 0.152 0.121))
(defparameter *gray-plane* '(0.482 0.537 0.549))
(defparameter *wood-plane* '(0.823 0.698 0.513))

(defparameter *base-x* -2.75)
(defparameter *base-most-left-side-left-hand-pose* `((,*base-x* 2.2 0) (0.0d0 0.0d0 -0.382d0 0.923d0)))
(defparameter *base-very-left-side-left-hand-pose* `((,*base-x* 2.0 0) (0.0d0 0.0d0 -0.382d0 0.923d0)))
(defparameter *base-left-side-left-hand-pose* `((,*base-x* 1.5 0) (0.0d0 0.0d0 -0.382d0 0.923d0)))
(defparameter *base-somewhat-left-side-left-hand-pose* `((,*base-x* 1.3 0) (0.0d0 0.0d0 -0.382d0 0.923d0)))
(defparameter *base-middle-side-left-hand-pose* `((,*base-x* 1.1 0) (0.0d0 0.0d0 -0.382d0 0.923d0)))
;; (defparameter *base-right-side-left-hand-pose* `((,*base-x* 0.9 0) (0 0 0 1)))
(defparameter *base-right-side-left-hand-pose* `((,*base-x* 0.7 0) (0.0d0 0.0d0 -0.382d0 0.923d0)))
(defparameter *base-very-right-side-left-hand-pose* `((,*base-x* 0.35 0) (0.0d0 0.0d0 -0.382d0 0.923d0)))
(defparameter *base-touch-pose* `((,(- *base-x* 0.1) 1.5 0) (0.0d0 0.0d0 -0.382d0 0.923d0)))

(defparameter *good-base-pose*
  (cl-tf:make-pose-stamped
   "map" 0.0
   (cl-tf:make-3d-vector -2.6880709033373353d0 2.076750906430788d0 0.0d0)
   (cl-tf:make-quaternion 0.0d0 0.0d0 -0.38311944677109644d0 0.9236988088688916d0))
  "A really good base pose.")

(defparameter *object-spawning-data*
  `((:big-wooden-plate :big-wooden-plate ,*wood-plane*
                      ((,*plate-rad-x* ,*plate-rad-y* ,(- *plate-rad-z*)) (0 0 0 1)))
    (:holder-bolt :holder-bolt ,*yellow-plastic*
                 ((0.05 0.025 ,*holder-bolt-rad-z*) (0 0 0 1)))
    (:holder-upper-body :holder-upper-body ,*yellow-plastic*
                       ((0.153 0.112 ,*holder-upperbody-rad-z*)
                        (0 0 0 1)))
    (:holder-bottom-wing :holder-bottom-wing ,*gray-plastic*
                        ((0.16
                          0.27
                          ,*holder-bottom-wing-rad-z*)
                         ,man-int:*rotation-around-z-90-list*))

    (:holder-underbody :holder-underbody ,*yellow-plastic*
                      ((0.17 0.413 ,*holder-underbody-rad-z*)
                       (0 0 0 1)))



    
    (:holder-plane-horizontal :holder-plane-horizontal ,*yellow-plastic*
                             ((0.146
                               0.675
                               ,*holder-plane-horizontal-rad-z*)
                              (0 0 0.7071067811865475d0 0.7071067811865476d0)))
    ;; (:holder-window :holder-window ,*gray-plastic*
    ;;                ((,*holder-window-rad-x*
    ;;                  ,(+ 0.75 *holder-window-rad-y*)
    ;;                  ,*holder-window-rad-z*)
    ;;                 (0 0 0 1)))
    ;; (:holder-plane-vertical :holder-plane-vertical ,*yellow-plastic*
    ;;                        ((,*holder-plane-vertical-rad-x*
    ;;                          ,(- 1.0 *holder-plane-vertical-rad-y*)
    ;;                          ,*holder-plane-vertical-rad-z*)
    ;;                         (0 0 0 1)))
    (:holder-top-wing :holder-top-wing ,*yellow-plastic*
                     ((0.155 0.943 ,*holder-top-wing-rad-z*)
                      ,man-int:*rotation-around-z-90-list*))

    ;; ;; rear wing is already well positioned
    ;; (:rear-wing :rear-wing ,*yellow-plane*
    ;;            ((0.163 ,(+ 0.569  0.541) 0.137);; (0.079 0.599 0.056)
    ;;             ,man-int:*rotation-around-z+90-list*))

    ;; ;; ;; bolts are used intermediately
    (:bolt-1 :bolt ,*gray-plane*
            ((0.015 0.0125 ,*bolt-rad-z*) (0 0 0 1)))
    ;; (:bolt-2 :bolt ,*gray-plane*
    ;;         ((0.0325 0.0375 ,*bolt-rad-z*) (0 0 0 1)))
    ;; (:bolt-3 :bolt ,*gray-plane*
    ;;         ((0.05 0.0125 ,*bolt-rad-z*) (0 0 0 1)))
    ;; (:bolt-4 :bolt ,*gray-plane*
    ;;         ((0.0675 0.0375 ,*bolt-rad-z*) (0 0 0 1)))
    ;; (:bolt-5 :bolt ,*gray-plane*
    ;;         ((0.085 0.0125 ,*bolt-rad-z*) (0 0 0 1)))

    ;; ;; first part of scenario on horizontal holder
    (:chassis :chassis ,*yellow-plane*
             ((0.123 1.109 ,*chassis-rad-z*) ,man-int:*rotation-around-z-90-list*))
    ;; (:bottom-wing :bottom-wing ,*cyan-plane*
    ;;              ((0.134 0.25 0.093) (0 0 0 1)))
    ;; (:underbody :underbody ,*red-plane*
    ;;            ((0.145 0.399 0.024) (0 0 0 1)))
    ;; (:motor-grill :motor-grill ,*black-plane*
    ;;              ((0.238 0.399 0.039) ,man-int:*rotation-around-y+90-list*))
    ;; (:upper-body :upper-body ,*red-plane*
    ;;             ((0.119 0.1003 0.0482) (0 0 0 1)))
    ;; (:top-wing :top-wing ,*cyan-plane*
    ;;           ((0.18522 ,(- 1.11423 0.55) 0.08852) (0 0 0 1)))
    ;; (:window :window ,*transparent-plane*
    ;;         ((0.024 0.775 0.01962) (0 0 0 1)))

    ;; second part of scenario on vertical holder
    ;; (:propeller :propeller ,*yellow-plane*
    ;;            ((0.075 1.10 0) (0 0 0 1)))
    ;; (:front-wheel-1 :front-wheel ,*black-plane*
    ;;                ((0.15 0.775 ,*front-wheel-rad-z*) (0 0 0 1)))
    ;; (:front-wheel-2 :front-wheel ,*black-plane*
    ;;                ((0.215 0.775 ,*front-wheel-rad-z*) (0 0 0 1)))
    ;; (:nut-1 :nut ,*gray-plane*
    ;;        ((0.15 0.725 ,*nut-rad-z*) (0 0 0 1)))
    ;; (:nut-2 :nut ,*gray-plane*
    ;;        ((0.215 0.725 ,*nut-rad-z*) (0 0 0 1)))
    ))

;;; ASSEMBLY STEPS:
;;; (0)  put chassis on holder (bump inwards)
;;; (1)  put bottom wing on chassis
;;; (2)  put underbody on bottom wing
;;; (3)  attach underbody to rear-wing
;;; (4)  put upperbody on underbody
;;; (5)  screw rear hole
;;; (6)  put top wing on body
;;; (7)  screw top wing
;;; (8)  put window on body
;;; (9)  screw window
;;; (10) put plane on vertical holder
;;; (11) put propeller on grill
;;; (12) screw propeller
;;; * put wheel on
;;; * screw nut onto wheel
;;; * put other wheel on
;;; * screw nut onto wheel
;;; * screw bottom body
(defparameter *assembly-steps*
  `(
  ;; 0
  (palpate-board)  
  ;; 1
  (go-connect :chassis *base-very-left-side-left-hand-pose* ;; *base-very-left-side-left-hand-pose*
              :holder-plane-horizontal *base-most-left-side-left-hand-pose* ;; *base-middle-side-left-hand-pose*
              :horizontal-attachment)
  ;; 2
  (go-connect :bottom-wing *base-very-right-side-left-hand-pose*
              :chassis *base-very-left-side-left-hand-pose* ;; *base-left-side-left-hand-pose*
              :wing-attachment)

  ;; 3
  (go-connect :underbody *base-somewhat-left-side-left-hand-pose*
              :bottom-wing *base-very-left-side-left-hand-pose* ;; *base-middle-side-left-hand-pose*
              :body-attachment)

  ;; we put the underbody on the bottom-wing but by doing that
  ;; we also put it on the rear-wing.
  ;; as there is no explicit placing action, the two will not be
  ;; attached automatically.
  ;; so we have to attach them manually unfortunately.
  ;; this is required for later moving the whole plane onto another holder
  ;; 4
  (btr:attach-object :underbody :rear-wing)

  ;; 5
  (go-connect :upper-body *base-right-side-left-hand-pose*
              :underbody *base-very-left-side-left-hand-pose*
              :body-on-body)
  ;; 6
  (go-connect :bolt *base-right-side-left-hand-pose*
              :upper-body *base-very-left-side-left-hand-pose*
              :rear-thread)
  ;; 7
  (go-connect :top-wing *base-left-side-left-hand-pose*
              :upper-body *base-left-side-left-hand-pose*
              :wing-attachment)
  ;; 8
  (go-connect :bolt *base-right-side-left-hand-pose*
              :top-wing *base-left-side-left-hand-pose*
              :middle-thread)
  ;; 9
  (go-connect :window *base-somewhat-left-side-left-hand-pose*
              :top-wing *base-left-side-left-hand-pose*
              :window-attachment)
  ;; 10
  (go-connect :bolt *base-right-side-left-hand-pose*
              :window *base-left-side-left-hand-pose*
              :window-thread)

  ;; 11
  (go-connect :top-wing  *base-somewhat-left-side-left-hand-pose*
              :holder-plane-vertical *base-left-side-left-hand-pose*
              ;; or `((,(- *base-x* 0.00) 1.45 0) (0 0 0 1))
              :vertical-attachment)

  ;; 12
  (go-connect :propeller `((,(- *base-x* 0.15) 2 0) (0 0 0 1))
              :motor-grill *base-left-side-left-hand-pose*
              ;; or `((,(- *base-x* 0.15) 1.8 0) (0 0 0 1))
              :propeller-attachment)

  ;; 13
  (go-connect :bolt *base-right-side-left-hand-pose*
              :propeller *base-left-side-left-hand-pose*
              ;; or `((,*base-x* 1.85 0) (0 0 0 1))
              :propeller-thread)

  ;; 14
  (exe:perform
   (desig:an action
             (type positioning-arm)
             (left-configuration park)
             (right-configuration park)))))
