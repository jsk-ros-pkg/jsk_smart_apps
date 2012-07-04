#!/usr/bin/env roseus

(ros::roseus "pickbox")
(require :robot-interface "package://pr2eus/robot-interface.l")
(require :hrp4r "package://hrpsys_ros_bridge/models/hrp4r.l")
(load "./hrp4-utils.l")

(defclass hrp4r-interface
  :super robot-interface
  :slots ())
(defmethod hrp4r-interface
  (:init (&rest args)
	  (send-super :init :robot hrp4r-robot)))

(setq *ri* (instance hrp4r-interface :init))
(setq *hrp4* (instance hrp4r-robot :init))
(objects (list *hrp4*))
(setq *base-pos* (send *hrp4* :head :end-coords :worldpos)
      *msg* nil)

(defun cb (msg)
  (setq *msg* msg))

(defun proccess ()
  (if (not *msg*) (return-from proccess nil))
  (let ((co (ros::tf-pose-stamped->coords *msg*)))
    (send *hrp4* :head :look-at
	  (v+ (send co :transform-vector #f(0 0 -2000)) ;; look at "2m front"
	      *base-pos*))

    (send *ri* :angle-vector (send *hrp4* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ))

(ros::subscribe "/android/orientation" geometry_msgs::PoseStamped #'cb 1)

(ros::rate 1)
(do-until-key
 (if (not (ros::ok)) (exit))
 (ros::spin-once)
 (proccess)
 (x::window-main-one)
 (ros::sleep)
 )
