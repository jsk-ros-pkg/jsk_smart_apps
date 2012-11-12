#!/usr/bin/env roseus
(ros::load-ros-manifest "jsk_smart_gui")
(ros::load-ros-manifest "roseus")

(load "package://pr2eus/pr2-interface.l")
(load "package://jsk_smart_gui/src/utils.l")

(ros::roseus "tablet-emergency-stop")

(defun StopNavigationExecute (&key (wait-for-server-timeout 5))
  (let* ((move-base-action-name "move_base")
	 (move-base-action
	  (instance ros::simple-action-client :init
		    move-base-action-name move_base_msgs::MoveBaseAction)))
    (when (not (send move-base-action :wait-for-server wait-for-server-timeout))
       (return-from StopNavigationExecute nil))
    (send move-base-action :cancel-all-goals)
  ))

(defun StopJointExecute ();;(wait nil)
  (if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init)))
  (send *ri* :cancel-angle-vector)
  )

(defun emergency-command-cb (msg)
  (if (is-old-msg msg) (return-from emergency-command-cb nil))
  (ros::ros-info "emergency-command-cb called")
  (let ((taskname (read-from-string (send msg :data))))
    (case taskname
      ('StopNavigation
       (ros::ros-info "StopNavigation called")
       (StopNavigationExecute))
      ('StopJoint
       (ros::ros-info "StopJoint called")
       (StopJointExecute))
      )))

(defun execute-main ()
  (if (not (boundp '*pr2*)) (pr2))
  (if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init)))

  (ros::subscribe "/Tablet/EmergencyCommand"
		  roseus::StringStamped #'emergency-command-cb)

  (setq *time-count* (ros::time-now))
  (ros::rate 10)
  (ros::ros-info "begin tablet emergency-controller")

  (ros::spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep))
  (ros::exit))