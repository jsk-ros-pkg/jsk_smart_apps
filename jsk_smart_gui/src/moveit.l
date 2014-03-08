#!/usr/bin/env roseus
(ros::load-ros-manifest "image_view2")

(load "package://pr2eus_openrave/pr2eus-openrave.l")

(setq *hand-coords* (make-coords))
(setq *move-time* (make-sequence 'list 6 :initial-element 0))

(defun moveit-cb (msg)
  (let ((armid (send msg :arm_id)) (taskname (send msg :task_name)) (direction (send msg :direction)) (resolution (send msg :direction_value)) arm wrt hand-coords-result hand-coords-result2 hand-edges (marker-flag t) move-flag)
    (if (= armid 0)
        (setq arm :rarm)
      (setq arm :larm))
    (if (string= direction "local")
        (setq wrt :local)
      (setq wrt :world))
    (case (intern (string-upcase taskname) *keyword-package*)
      (:x+
       (setq *hand-coords* (send *hand-coords* :translate (float-vector resolution 0 0) wrt))
       (setf (elt *move-time* 0) (+ (elt *move-time* 0) (/ resolution 10.0)))
       )
      (:x-
       (setq *hand-coords* (send *hand-coords* :translate (float-vector (- resolution) 0 0) wrt))
       (setf (elt *move-time* 0) (- (elt *move-time* 0) (/ resolution 10.0)))
       )
      (:y+
       (setq *hand-coords* (send *hand-coords* :translate (float-vector 0 resolution 0) wrt))
       (setf (elt *move-time* 1) (+ (elt *move-time* 1) (/ resolution 10.0)))
       )
      (:y-
       (setq *hand-coords* (send *hand-coords* :translate (float-vector 0 (- resolution) 0) wrt))
       (setf (elt *move-time* 1) (- (elt *move-time* 1) (/ resolution 10.0)))
       )
      (:z+
       (setq *hand-coords* (send *hand-coords* :translate (float-vector 0 0 resolution) wrt))
       (setf (elt *move-time* 2) (+ (elt *move-time* 2) (/ resolution 10.0)))
       )
      (:z-
       (setq *hand-coords* (send *hand-coords* :translate (float-vector 0 0 (- resolution)) wrt))
       (setf (elt *move-time* 2) (- (elt *move-time* 2) (/ resolution 10.0)))
       )
      (:roll+
       (setq *hand-coords* (send *hand-coords* :rotate (deg2rad resolution) :x wrt))
       (setf (elt *move-time* 3) (+ (elt *move-time* 3) (/ resolution 10.0)))
       )
      (:roll-
       (setq *hand-coords* (send *hand-coords* :rotate (deg2rad (- resolution)) :x wrt))
       (setf (elt *move-time* 3) (- (elt *move-time* 3) (/ resolution 10.0)))
       )
      (:pitch+
       (setq *hand-coords* (send *hand-coords* :rotate (deg2rad resolution) :y wrt))
       (setf (elt *move-time* 4) (+ (elt *move-time* 4) (/ resolution 10.0)))
       )
      (:pitch-
       (setq *hand-coords* (send *hand-coords* :rotate (deg2rad (- resolution)) :y wrt))
       (setf (elt *move-time* 4) (- (elt *move-time* 4) (/ resolution 10.0)))
       )
      (:yaw+
       (setq *hand-coords* (send *hand-coords* :rotate (deg2rad resolution) :z wrt))
       (setf (elt *move-time* 5) (+ (elt *move-time* 5) (/ resolution 10.0)))
       )
      (:yaw-
       (setq *hand-coords* (send *hand-coords* :rotate (deg2rad (- resolution)) :z wrt))
       (setf (elt *move-time* 5) (- (elt *move-time* 5) (/ resolution 10.0)))
       )
      (:start
       (send *ri* :update-robot-state)
       (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
       (setq *hand-coords* (send *pr2* arm :end-coords :copy-worldcoords))
       (setq *move-time* (make-sequence 'list 6 :initial-element 0))
       )
      (:stop
       (dotimes (i 7)
         (remove-marker i :ns "hand_traj" :topic-name "openrave_marker_array"))
       (ros::publish "image_marker" (instance image_view2::ImageMarker2 :init :ns "hand" :id 601 :action image_view2::ImageMarker2::*REMOVE*))
       (setq *move-time* (make-sequence 'list 6 :initial-element 0))
       (setq marker-flag nil)
       )
      (:movearm
       (setq move-flag t)
       )
      (t
       (print "do nothing")
       (setq marker-flag nil)
       ))
    (when marker-flag
      (setq hand-coords-result (send *ri* :show-goal-hand-coords *hand-coords* arm))
      (setq hand-coords-result2
            (mapcar #'(lambda(b c)
                        (let ((bb (copy-object b)))
                          (send bb :move-to c :world)))
                    (car hand-coords-result) (cadr hand-coords-result)))
      (send-all hand-coords-result2 :worldcoords)
      (setq hand-edges (flatten (send-all hand-coords-result2 :edges)))
      (setq ik-ret (send *pr2* arm :inverse-kinematics *hand-coords* :use-torso nil))
      (if ik-ret
          (progn
            (visualize-object-image-from-edges
             hand-edges
             :counter 601 :lifetime 10 :ns "hand" :outline-colors (list (instance std_msgs::ColorRGBA :init :r 1.0 :g 0.0 :b 0.0 :a 1.0)))
            (when move-flag
              (send *ri* :angle-vector (send *pr2* :angle-vector) (* (ceiling (apply #'+ (mapcar #'abs *move-time*))) 200))
              (send *ri* :wait-interpolation)
              (setq *move-time* (make-sequence 'list 6 :initial-element 0))))
        (visualize-object-image-from-edges
         hand-edges
         :counter 601 :lifetime 10 :ns "hand" :outline-colors (list (instance std_msgs::ColorRGBA :init :r 0.0 :g 0.0 :b 1.0 :a 1.0))))
      )
    ))
