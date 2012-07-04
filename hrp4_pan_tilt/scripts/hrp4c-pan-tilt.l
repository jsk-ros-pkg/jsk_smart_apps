#!/usr/bin/env roseus

(ros::roseus "pickbox")
(require :robot-interface "package://pr2eus/robot-interface.l")
(require :hrp4 "package://hrpsys_ros_bridge/models/hrp4c.l")
(load "./hrp4-utils.l")

(defclass hrp4c-interface
  :super robot-interface
  :slots ())
(defmethod hrp4c-interface
  (:init (&rest args)
	  (send-super :init :robot hrp4-robot)))

(setq *ri* (instance hrp4c-interface :init))
(setq *hrp4* (instance hrp4-robot :init))
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
    ;(send *ri* :wait-interpolation)
    ))

(ros::subscribe "/android/orientation" geometry_msgs::PoseStamped #'cb 1)

(do-until-key
 (if (not (ros::ok)) (exit))
 (ros::spin-once)
 (proccess)
 (x::window-main-one)
 (unix::usleep (* 100 1000)))
