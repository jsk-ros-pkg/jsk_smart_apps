#!/usr/bin/env roseus
(ros::load-ros-manifest "dynamic_tf_publisher")
;; This script depends on tablet_callback.l
;; Use when creating template for imagesift
(ros::roseus "template-creator")

(defun GetTemplateExecute (x y)
  (let* ((c (screenpoint-srvcall x y))
	 img_res poly_points width height)
    (publish_tf c "/touched_point")
    (change_cropbox_frame "/touched_point")
    (ros::ros-info "waiting for 5 sec")
    (unix::sleep 5)
    ;;save img
    (setq img_res (save_img "test.jpg"))
    ;; calculate width height
    (setq poly_points (send img_res :points :polygon :points))
    (unless (eq (length poly_points) 4)
      (ros::ros-warn "poly_points did not have 4 points")
      (return-from GetTemplateExecute nil))

    (setq width (calc_distance poly_points 0 1))
    (setq height (calc_distance poly_points 0 2))

    ;;TODO:translate from poly_header to base_footprint
    ;;TODO:calculate offset

    ;;call settemplate for pointposeextractor
    (ros::set-param "window_name" "test_window")
    (ros::set-param "template_filename" (format nil "~/~A" "test.jpg"))
    (ros::set-param "object_width" width)
    (ros::set-param "object_height" height)
    (ros::set-param "viewer_window" t)
    ))

(defun publish_tf (pos child_frame &key (parent_frame "base_footprint"))
  (let ((settf (instance dynamic_tf_publisher::SetDynamicTFRequest :init)))
    (send settf :freq 10.0)
    (send settf :cur_tf (ros::coords->tf-transform-stamped (make-coords :pos (send pos :pos))
							   parent_frame child_frame))
    (ros::service-call "set_dynamic_tf" settf)
    ))

(defun delete_tf (frame)
  (let ((deltf (instance dynamic_tf_publisher::DeleteTFRequest :init)))
    (send deltf :header :frame_id frame)
    (ros::service-call "delete_tf" deltf)))

(defun change_cropbox_frame (frame)
  (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
    (send req :config :strs
    	  (list (instance dynamic_reconfigure::StrParameter
    			  :init :name "target_frame_id" :value frame)))
    (ros::service-call "cropbox_manager/set_parameters" req)))

(defun save_img (name)
  (let ((req (instance object_snapshotter::SaveImgRequest :init)) res)
    (send req :file_name name)
    (if (and
	 (ros::service-exists "/save_image")
	 (ros::wait-for-service "/save_image" 30))
	(setq res (ros::service-call "/save_image" req))
       (ros::ros-warn "/save_image did not respond"))
    res))

(defun calc_distance (poly num0 num1)
  (let ((a (ros::tf-point->pos (elt poly num0)))
	(b (ros::tf-point->pos (elt poly num1))))
    (distance a b)))
