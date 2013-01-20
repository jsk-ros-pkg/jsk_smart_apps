#!/usr/bin/env roseus
(ros::load-ros-manifest "dynamic_tf_publisher")
;; This script depends on tablet_callback.l
;; Use when creating template from imagesift
(ros::roseus "template_creator")
;;(load "package://jsk_smart_gui/src/tablet_receiver.l") - loop

(defun ros::coords->tf-pose-stamped0 (coords id)
  (let ((pose (ros::coords->tf-pose coords)))
    (instance geometry_msgs::PoseStamped :init
              :header (instance std_msgs::header :init
                                :frame_id id
                                :stamp (ros::time 0))
				:pose pose)))

(defun get_template_init ()
  (defvar *ray_srv* "/pointcloud_screenpoint_nodelet/screen_to_point")
  (defvar *tfl* (instance ros::transform-listener :init))
  (defvar *tfb* (instance ros::transform-broadcaster :init))
  (defvar *width* 0)
  (defvar *height* 0)
  (defvar *template-origin* (make-coords))
  (ros::advertise "tablet_marker" visualization_msgs::Marker 10)
  )
;;(GetTemplateExecute 320 240)
;;(show-marker (send *template-origin* :pos))
(defun GetTemplateExecute (x y)
  (let* ((c (screenpoint-srvcall x y))
	 (time-now (send (ros::time-now) :to-sec))
	 img_res poly_points) ;; width height)
    (publish_tf c "/touched_point")
    (change_cropbox_frame "/touched_point")
    (ros::ros-info "waiting for 3 sec")
    (unix::sleep 3)
    ;;save img and call settemplate
    (setq img_res (save_img (format nil "~A/.ros/~A.jpg" (unix:getenv "HOME") time-now)))

    ;; calculate width height
    (unless img_res
      (ros::ros-warn "img failed")
      (return-from GetTemplateExecute nil))
    (setq poly_points (send img_res :points :polygon :points))
    (unless (eq (length poly_points) 4)
      (ros::ros-warn "poly_points did not have 4 points")
      (return-from GetTemplateExecute nil))
    ;; 3  0
    ;; 2  1
    (setq *width* (calc_distance poly_points 0 1))
    (setq *height* (calc_distance poly_points 0 2))
    (setq cam-cds (ros::coords->tf-pose-stamped0 (make-coords :pos (ros::tf-point->pos (elt poly_points 3))) "/openni_rgb_optical_frame"))
    (send *tfl* :wait-for-transform "/openni_rgb_optical_frame" "base_footprint" (ros::time 0) 5)
    (setq *template-origin* (send *tfl* :transform-pose "base_footprint" cam-cds))
    ;;TODO:calculate offset

    ;; call settemplate for pointposeextractor
    (ros::set-param "/point_pose_extractor_test/template_filename" (format nil "~A/.ros/~A.jpg" (unix:getenv "HOME") time-now))
    (ros::set-param "/point_pose_extractor_test/object_width" (/ *width* 1000.0))
    (ros::set-param "/point_pose_extractor_test/object_height" (/ *height* 1000.0))
    (unix::system (format nil "rosparam set /point_pose_extractor_test/viewer_window true"))
    (dump-loadable-structure
     (format nil "get-template-~d.l" time-now)
     *width* *height*)
    ))

(defun set_template_sample1 ()
  ;;(ros::set-param "/point_pose_extractor_test/window_name" "test_window")
  (ros::set-param "/point_pose_extractor_test/object_width" (/ *width* 1000.0))
  (ros::set-param "/point_pose_extractor_test/object_height" (/ *height* 1000.0))
  (ros::set-param "/point_pose_extractor_test/template_filename" (format nil "/home/chen/.ros/1.358658e+09.jpg"))
  (unix::system (format nil "rosparam set /point_pose_extractor_test/viewer_window true"))
  )

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
    (ros::service-call "cropbox_nodelet/set_parameters" req)))
;;    (ros::service-call "cropbox_manager/set_parameters" req)))

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
