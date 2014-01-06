#!/usr/bin/env roseus
(ros::roseus "pour_bottle_water_to_cup")
(load "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus_tutorials/euslisp/template_grasp_samples.l")
(require :pr2-pose "package://jsk_demo_common/euslisp/pr2-pose.l")

(setq *threshold* 12.0)
(setq *quant* 2)
(setq *seq-x* 20)
(setq *seq-y* 20)
(setq *rotate-cap-num* 7)

(setq *bottle-height* 215)
(setq *bottle-radius* 14)
(setq *cup-height* 90)
(setq *cup-radius* 40)

(when (not (boundp '*pr2*))
   (pr2)
   (setq *ri* (instance pr2-interface :init))
   )
 (load "models/arrow-object.l")
(setq *arrow* (arrow))
(send *ri* :objects (list *arrow*))
(setq *action-flg* nil)

(defun look-at-table (torso)
  (pr2-pick-tray-pose :reset-pose t)
  (send *pr2* :head :neck-p :joint-angle 60)
  (send *pr2* :torso-waist-z :joint-angle torso)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
)

(defun setup ()
  (setq *rate* 5)
  (setq *action-flg* nil)
  (setq *base-to-object-pose-list* nil)
  (setq *base-to-camera-pose* nil)
  (ros::set-param "stop_tabletop" 0)
  (ros::rate (/ 1.0 *rate*))
  (ros::subscribe "ObjectDetection" posedetection_msgs::ObjectDetection #'objectdetection-cb)
  (setq *tfl* (instance ros::transform-listener :init))
  (setq *tfb* (instance ros::transform-broadcaster :init));;tmp
)

(defun pour-cup (estimated-center-pose &optional (num 0))
  (send *arrow* :transform estimated-center-pose)
  (send *ri* :objects (list *arrow*))

  ;;right arm
  (send *pr2* :larm :inverse-kinematics
        estimated-center-pose
        :rotation-axis :z)
  (send *pr2* :larm :move-end-pos #f(-100 0 50))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)

  ;; left arm
  ;; approach
  (setq *pour-radius* (- 150 10))
  (send *pr2* :rarm :inverse-kinematics
        estimated-center-pose
        :rotation-axis :z)
  (send *pr2* :rarm :move-end-pos #f(-150 0 50))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  ;; hand-to-cap 150 (-30?)
  (setq roll (make-coords :pos (float-vector *pour-radius* 0 (+ *cup-height* 20))
                          :rpy (float-vector 0 0 0)))
  (send *pr2* :rarm :inverse-kinematics (transform-coords *cup-estimated-pose* roll) :rotation-axis :z)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  ;; start
  (setq roll (make-coords :pos (float-vector *pour-radius* 0 (+ *cup-height* 20))
                          :rpy (float-vector 0 (* pi -0.4) 0)))
  (send *pr2* :rarm :inverse-kinematics (transform-coords *cup-estimated-pose* roll) :rotation-axis :z)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  ;; down
  (setq roll (make-coords :pos (float-vector *pour-radius* 0 (- *cup-height* 50))
                          :rpy (float-vector 0 (* pi -0.4) 0)))
  (send *pr2* :rarm :inverse-kinematics (transform-coords *cup-estimated-pose* roll) :rotation-axis :z)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  ;; finish
  (setq roll (make-coords :pos (float-vector *pour-radius* 0 (+ *cup-height* 10))
                          :rpy (float-vector 0 (* pi -0.60) 0)))
  (send *pr2* :rarm :inverse-kinematics (transform-coords *cup-estimated-pose* roll) :rotation-axis :z)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)

  (unix:sleep 3)
  ;; up
  (setq roll (make-coords :pos (float-vector *pour-radius* 0 (+ *cup-height* 20))
                          :rpy (float-vector 0 (* pi -0.4) 0)))
  (send *pr2* :rarm :inverse-kinematics (transform-coords *cup-estimated-pose* roll) :rotation-axis :z)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  ;; back
  (setq roll (make-coords :pos (float-vector *pour-radius* 0 (+ *cup-height* 20))
                          :rpy (float-vector 0 0 0)))
  (send *pr2* :rarm :inverse-kinematics (transform-coords *cup-estimated-pose* roll) :rotation-axis :z)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
      ;; down
  (send *pr2* :rarm :move-end-pos #f(0 0 -120))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  ;; open hand
  (send *ri* :stop-grasp :rarm)
  (send *ri* :wait-interpolation)

  ;; right arm
  ;; down
  (send *pr2* :larm :move-end-pos #f(0 0 -100))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  ;;open hand
  (send *ri* :stop-grasp :larm)
  (send *ri* :wait-interpolation)
  ;;approach
  (send *pr2* :larm :move-end-pos #f(110 0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  ;;close hand
  (send *ri* :move-gripper :larm 0.080 :effort 30)
  (send *ri* :wait-interpolation)
  )



(defun pick-pet (estimated-center-pose &optional (num 0))
  (send *arrow* :transform estimated-center-pose)
  (send *ri* :objects (list *arrow*))
  ;; (unless (tf-object-update num)
  ;;   (unix:sleep 3))
  (let* (
         ;;(pet (make-coords :pos (send *base-to-object-pose* :worldpos)))
        larm-cds)
    ;; open hand
    (send *ri* :stop-grasp :larm)
    (send *ri* :wait-interpolation)
    ;;left-arm
    (send estimated-center-pose :translate #f(0 0 10) :world)
    (send *pr2* :larm :inverse-kinematics
          estimated-center-pose
          :rotational-axis :z)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    ;; approach
    (send *pr2* :larm :move-end-pos #f(0 0 -15))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    ;;close hand
    (send *ri* :move-gripper :larm 0.010 :effort 50)
    (send *ri* :wait-interpolation)
    ;;right arm
    (send *pr2* :rarm :inverse-kinematics
          estimated-center-pose
          :rotation-axis :z)
    (send *pr2* :rarm :move-end-pos #f(-100 0 -150))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    ;;open hand
    (send *ri* :stop-grasp :rarm)
    (send *ri* :wait-interpolation)
    ;;approach
    (send *pr2* :rarm :move-end-pos #f(115 0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    ;;close hand
    (send *ri* :move-gripper :rarm 0.045 :effort 30)
    (send *ri* :wait-interpolation)
    ;; rotate
    (send estimated-center-pose :translate #f(0 0 -15) :world)
    (dotimes (j *rotate-cap-num*)
      (dotimes (i 7)
        (setq *tmp* (make-coords :rpy (float-vector (+ 0.20 (* i 0.20)) 0 0)))
        (send *pr2* :larm :inverse-kinematics
              (transform-coords
               estimated-center-pose
               *tmp*)
              :rotational-axis :x)
        (send *ri* :angle-vector (send *pr2* :angle-vector))
        (send *ri* :wait-interpolation)
        )
      ;; open
      (send *ri* :stop-grasp :larm)
      (send *ri* :wait-interpolation)
      ;; up
      (send *pr2* :larm :move-end-pos #f(0 0 200))
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (send *ri* :wait-interpolation)
      ;; back
      (send *ri* :wait-interpolation)
      (setq *tmp* (make-coords :pos (float-vector 0 0 200)))
      (send *pr2* :larm :inverse-kinematics
            (transform-coords
             estimated-center-pose
             *tmp*)
            :rotational-axis :x)
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (send *ri* :wait-interpolation)
      ;; down
      (send *pr2* :larm :move-end-pos #f(0 0 -200))
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (send *ri* :wait-interpolation)
      ;; close
      (send *ri* :move-gripper :larm 0.010 :effort 50)
      (send *ri* :wait-interpolation))

    ;;
    (send *ri* :move-gripper :larm 0.015 :effort 10)
    (send *ri* :wait-interpolation)
    ;; up
    (send *pr2* :larm :move-end-pos #f(0 0 50))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    ;; open
    (send *ri* :stop-grasp :larm)
    (send *ri* :wait-interpolation)
    ))

(defun objectdetection-cb (msg)
  (if *action-flg*
      (return-from objectdetection-cb nil))
  (let ((frame-id (send msg :header :frame_id)) type)
    (setq *base-to-table-pose* (send *tfl* :lookup-transform "/base_footprint" "/table" (ros::time 0)))
    (setq *base-to-camera-pose* (send *tfl* :lookup-transform "/base_footprint" "/openni_rgb_optical_frame" (ros::time 0)))
    (if (or (null *base-to-table-pose*)
            (null *base-to-camera-pose*))
        (return-from objectdetection-cb nil))
    (setq *action-flg* t)
    (unless (eq (char frame-id 0) #\/) (setq frame-id (concatenate string "/" frame-id)))
    (setq *base-to-object-pose-list* nil)
    ;; start recognize
    (when (zerop (ros::get-param "stop_tabletop"))
      (let ((td-res (tabletop-detect)) pc-list tablepoly bottle-estimated-pose cup-estimated-pose)
        (when td-res
          (setq pc-list (car td-res))
          (setq tablepoly (cadr td-res))
          (setq bottle-estimated-pose (object-detect pc-list tablepoly :lifetime *rate* :ns "estimate_bottle" :color (float-vector 0 1 0) :counter 100 :scale 10 :obj-poses (send msg :objects) :obj-height *bottle-height* :obj-radius *bottle-radius*))
          (setq cup-estimated-pose (object-detect pc-list tablepoly :lifetime *rate* :ns "estimate_bottle" :color (float-vector 0 1 0) :counter 100 :scale 10 :obj-poses (send msg :objects) :obj-height *cup-height* :obj-radius *cup-radius*))
          (if (or (equal bottle-estimated-pose nil)
                  (equal cup-estimated-pose nil))
              (progn (setq *action-flg* nil)
                     (return-from objectdetection-cb nil))
                  )
          ;; action-start
          (look-at-table 150)
          (setq *bottle-estimated-pose* bottle-estimated-pose)
          (setq *cup-estimated-pose* cup-estimated-pose)
          (pick-pet bottle-estimated-pose)
          (pour-cup cup-estimated-pose)
          )))
    ))


(defun object-detect (pclist tablepoly &key (counter 0) (ns nil) (lifetime 10) (color #f(0 1 0)) (alpha 1) (scale 10) (obj-poses nil) (obj-height 0) (obj-radius 0))

  (let (float-pose estimated-center-pose base-to-point-pose update-estimated-center-pose
                   pc-pose-list pc-pose min max-count count est-x est-y est-count
                   pc-count pc-x pc-y base-to-object-pose table-to-object-pose)

    (dolist (obj-pose obj-poses)
      (setq table-to-object-pose (ros::tf-pose->coords (send obj-pose :pose)))
      (send table-to-object-pose :replace-pos (float-vector (elt (send table-to-object-pose :pos) 0) (elt (send table-to-object-pose :pos) 1) 0))
      (push (send (transform-coords *base-to-table-pose* table-to-object-pose) :translate (float-vector 0 0 obj-height)) *base-to-object-pose-list*))

  (dolist (pc pclist)
    (setq pc-count 0)
    (setq pc-x 0)
    (setq pc-y 0)
    (mapcar #'(lambda(p)
                  (progn (setq pc-pose (transform-coords (send *base-to-camera-pose* :copy-worldcoords) (make-coords :pos (apply #'float-vector p))))
                         (if (and (> (elt (send pc-pose :pos) 2)
                                     (elt (send (elt *base-to-object-pose-list* 0) :pos) 2))
                                  (< (elt (send pc-pose :pos) 2)
                                     (+ (elt (send (elt *base-to-object-pose-list* 0) :pos) 2)
                                        20)))
                             (progn (push pc-pose pc-pose-list)
                                    (setq pc-count(1+ pc-count))
                                    (setq pc-x (+ pc-x (elt (send pc-pose :pos) 0)))
                                    (setq pc-y (+ pc-y (elt (send pc-pose :pos) 1)))))
                         ))
                     (read-from-string (subseq
                       (format nil "~A" (send pc :points)) 3 (length (format nil "~A" (send pc :points))))))
    (print pc)
    (format t "pc-count:~A~%" pc-count)
    (if (> pc-count 0)
        (progn
    ;; decide near object pose
    ;; (let (tmp-distance min-distance)
    ;;   (setq min-distance 10000)
    ;;   (dolist (pose *base-to-object-pose-list*)
    ;;     (setq tmp-distance (+ (* (- (elt (send pose :pos) 0) (/ pc-x pc-count))
    ;;                              (- (elt (send pose :pos) 0) (/ pc-x pc-count)))
    ;;                           (* (- (elt (send pose :pos) 1) (/ pc-y pc-count))
    ;;                              (- (elt (send pose :pos) 1) (/ pc-y pc-count)))))
    ;;     (if (< tmp-distance min-distance)
    ;;         (progn (setq min-distance tmp-distance)
    ;;                (setq base-to-object-pose pose)
    ;;       )
    ;;         ))
    ;; )
    (setq base-to-object-pose (elt *base-to-object-pose-list* 0))
    (send base-to-object-pose :replace-pos (float-vector (/ pc-x pc-count)
                                                         (/ pc-y pc-count)
                                                         (elt (send base-to-object-pose :pos) 2)))
    (setq max-count 10)
    (setq est-count 0.0)
    (setq est-x 0.0)
    (setq est-y 0.0)
    (dotimes (x *seq-x*)
      (dotimes (y *seq-y*)
    (setq count 0)
    (setq update-estimated-center-pose (send (send base-to-object-pose :copy-worldcoords) :translate (float-vector (* (- x (/ *seq-x* 2.0)) *quant*) (* (- y (/ *seq-y* 2.0)) *quant*) 0)))
    (dotimes (i 48)
      (setq float-pose (send
       (send update-estimated-center-pose :copy-worldcoords) :translate
       (float-vector
        (* obj-radius (cos (* i (/ (* 2 pi) 48))))
        (* obj-radius (sin (* i (/ (* 2 pi) 48))))
        0)))
      ;;
      (setq min 1000)
      (dolist (pc-pose pc-pose-list)
          (setq *distance* (distance (send float-pose :pos)
                                   (send pc-pose :pos)))
                    (if (< *distance* min)
                        (setq min *distance*)))
      (if (< min *threshold*)
          (setq count (1+ count)))
    )
    (if (>= count max-count)
        (if (> count max-count)
            (progn (setq max-count count)
                   (setq est-count 1.0)
                   (setq est-x (* (- x (/ *seq-x* 2.0)) *quant*))
                   (setq est-y (* (- y (/ *seq-y* 2.0)) *quant*)))
        (progn (setq est-count (1+ est-count))
               (setq est-x (+ est-x (* (- x (/ *seq-x* 2.0)) *quant*)))
               (setq est-y (+ est-y (* (- y (/ *seq-y* 2.0)) *quant*)))
               ))
    )))
    (if (> max-count 20)
        (progn
          (send *tfb* :send-transform base-to-object-pose "/base_footprint" "before") ;;tmp
          (setq estimated-center-pose (send (send base-to-object-pose :copy-worldcoords) :translate (float-vector (/ est-x est-count) (/ est-y est-count) 0)))
          (send *tfb* :send-transform estimated-center-pose "/base_footprint" "after") ;;tmp
          (format t "(~A,~A)~%" (/ est-x est-count) (/ est-y est-count))
          (return-from object-detect estimated-center-pose))
      (print "mismatch object"))
    )))
))

(setup)
(ros::wait-for-service "/tabletop_segmentation")
(while (ros::ok)
 (ros::spin-once)
 (ros::ros-info "Scanning loop")
 (ros::sleep))
(ros::exit)
