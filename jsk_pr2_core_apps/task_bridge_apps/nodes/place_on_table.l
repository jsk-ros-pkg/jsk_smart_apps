#!/usr/bin/env roseus
(load "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")
(ros::load-ros-manifest "tf")
(ros::roseus "place_on_table")

(when (not (boundp '*pr2*))
  (pr2)
  (setq *ri* (instance pr2-interface :init))
  )

(load "models/arrow-object.l")
(setq *arrow* (arrow))
(send *ri* :objects (list *arrow*))

(defun simulation ()
  (setq *object-to-arrow-pose* nil)
  (setq *base-to-object-pose* nil)

  (send *ri* :go-pos-unsafe 0.3 0 0) ;;tmp ;;0.2

  (send *pr2* :angle-vector (send *ri* :reference-vector))
  (send *pr2* :head :neck-p :joint-angle 70)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (unix::sleep 10) ;;tmp

  ;; reset robot by rosbag data
  (setq *tfl* (instance ros::transform-listener :init))
  (while (or (null *object-to-arrow-pose*)
             (null *base-to-object-pose*))
    (setq *object-to-arrow-pose* (send *tfl* :lookup-transform "/object" "/place_position" (ros::time 0)))
    (setq *base-to-object-pose* (send *tfl* :lookup-transform "/base_link" "/object" (ros::time 0)))
    (unix::sleep 5)
    )

  (setq *place-pose* (transform-coords *base-to-object-pose* *object-to-arrow-pose*))
  (send *arrow* :transform *place-pose*)
  (send *ri* :objects (list *arrow*))

  ;; solve ik
  (send *pr2* :larm :collar-y :joint-angle 45)
  (send *pr2* :rarm :collar-y :joint-angle -60)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  ;; approach
  (setq height 210)
  (while t
    (if (< height 110)
        (return)
      )
    (send *pr2* :larm :inverse-kinematics
          (list (send (send *place-pose* :copy-worldcoords) :translate (float-vector 0 0 height)))
          :use-torso t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (setq height (- height 20)))

  ;; open
  (send *ri* :stop-grasp :larm)
  (send *ri* :wait-interpolation)

  ;; up
  (send *pr2* :larm :inverse-kinematics
        (list (send (send *place-pose* :copy-worldcoords) :translate #f(0 0 200)))
        :use-torso t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (pr2-tuckarm-pose)
  (send *ri* :go-pos-unsafe -0.6 0 0) ;;-0.2
  (ros::exit)
)

(simulation)