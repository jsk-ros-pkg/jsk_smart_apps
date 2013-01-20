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
  (ros::advertise "image_marker" image_view2::ImageMarker2 100)
  (ros::advertise "tablet_marker" visualization_msgs::Marker 10)
  (ros::advertise "tablet_marker_array" visualization_msgs::MarkerArray 10)
  )

(defun use_template_init ()
  ;; (ros::subscribe "/ObjectDetection"
  ;; 		  posedetection_msgs::ObjectDetection #'use_template_cb1)

  (ros::subscribe "/ObjectDetection"
		  posedetection_msgs::ObjectDetection #'use_template_cb2)
  )

;;(GetTemplateExecute 320 240)
;;(show-marker (send *template-origin* :pos))
(defun GetTemplateExecute (x y)
  (let* ((c (screenpoint-srvcall x y))
	 (time-now (send (ros::time-now) :to-sec))
	 img_res poly_points cam-cds)
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


;;(show-marker (send *detected-template-origin* :pos))
(defun use_template_cb1(msg)
  (let* ((poses (car (send msg :objects)))
	 (pose (send poses :pose))
	 (frame-id (send msg :header :frame_id))
	 (cam-cds (instance geometry_msgs::PoseStamped :init))
	 )
    (ros::ros-info "using template")
    (send cam-cds :header :stamp (ros::time 0))
    (send cam-cds :header :frame_id frame-id)
    (send cam-cds :pose pose)

    (send *tfl* :wait-for-transform frame-id "base_footprint" (ros::time 0) 5)
    (setq *detected-template-origin* (send *tfl* :transform-pose "base_footprint" cam-cds))

    ;; rviz
    (show-line (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -45 -150) :world) :pos) (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -45 -450) :world) :pos))
    ;; image_view2
    (visualize-text-image-from-pos "Fridge/Open" (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 0 0) :world) :pos))
    (visualize-object-image-from-coords
     (list (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -45 -150) :world)
	   (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -45 -450) :world)) :counter 100)
  ))

(defun use_template_cb2(msg)
  (let* ((poses (car (send msg :objects)))
	 (pose (send poses :pose))
	 (frame-id (send msg :header :frame_id))
	 (cam-cds (instance geometry_msgs::PoseStamped :init))
	 )
    (ros::ros-info "using template")
    (send cam-cds :header :stamp (ros::time 0))
    (send cam-cds :header :frame_id frame-id)
    (send cam-cds :pose pose)

    (send *tfl* :wait-for-transform frame-id "base_footprint" (ros::time 0) 5)
    (setq *detected-template-origin* (send *tfl* :transform-pose "base_footprint" cam-cds))

    ;; rviz
    (show-markers
     (list
      (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -45 -150) :world) :pos)
      (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -45 -450) :world) :pos)
     ) :col #f(1 1 0))
    ;; image_view2
    (visualize-text-image-from-pos "IH button/Push(3)" (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 0 0) :world) :pos))
    (visualize-object-image-from-coords
     (list
      (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -45 -150) :world)
      (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -45 -450) :world)) :counter 100)
  ))

(defun set_template_sample1 () ;;for refridge
  ;; (let ((width *width*) (height *height*)
  (let ((width 203.856) (height 198.81))
  ;;(ros::set-param "/point_pose_extractor_test/window_name" "test_window")
  (ros::set-param "/point_pose_extractor_test/object_width" (/ width 1000.0))
  (ros::set-param "/point_pose_extractor_test/object_height" (/ height 1000.0))
  (ros::set-param "/point_pose_extractor_test/template_filename" (format nil "/home/chen/.ros/1.358658e+09.jpg"))
  (unix::system (format nil "rosparam set /point_pose_extractor_test/viewer_window true"))
  ))

(defun set_template_sample2 () ;;for IM
  (let ((width 207.005) (height 202.051))
  (ros::set-param "/point_pose_extractor_test/object_width" (/ width 1000.0))
  (ros::set-param "/point_pose_extractor_test/object_height" (/ height 1000.0))
  (ros::set-param "/point_pose_extractor_test/template_filename" (format nil "/home/chen/.ros/1.358671e+09.jpg"))
  (unix::system (format nil "rosparam set /point_pose_extractor_test/viewer_window true"))
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


(defun visualize-object-image-from-coords
  (cds-list &key (counter 0) (ns nil) (lifetime 10)
	    (outline-colors (list (instance std_msgs::ColorRGBA :init :r 1.0 :g 0.0 :b 0.0 :a 1.0))))
  (let* ((mrk (instance image_view2::ImageMarker2 :init))
	 point-list)
    (send mrk :header :stamp (ros::time-now))
    (send mrk :type image_view2::ImageMarker2::*LINE_STRIP3D*)
    (send mrk :points3D :header :frame_id "base_footprint")
;;    (send mrk :points3D :header :frame_id "base_footprint")
    (send mrk :lifetime (ros::Time lifetime))
    (send mrk :id counter)
    (dolist (cds cds-list)
      ;;(ros::ros-warn "cds: ~A" (send cds :pos))
      (push (ros::pos->tf-point (send cds :worldpos)) point-list))
    (send mrk :points3D :points point-list)
    (send mrk :outline_colors outline-colors)
    (if ns (send mrk :ns ns))
    (ros::publish "image_marker" mrk)
  ))


(defun visualize-text-image-from-pos
  (text pos &key (counter 0) (ns nil) (lifetime 10))
  (let ((text-mrk (instance image_view2::ImageMarker2 :init))
	)
    (send text-mrk :header :stamp (ros::time-now))
    (send text-mrk :type image_view2::ImageMarker2::*TEXT3D*)
    (send text-mrk :position3D :header :frame_id "base_footprint")
    (send text-mrk :position3D :point (ros::pos->tf-point pos))
    (send text-mrk :scale 1.0)
    (send text-mrk :lifetime (ros::time lifetime))
    (send text-mrk :id counter)
    (send text-mrk :text (if text text (format nil "~A" obj-cntr)))
    (if ns (send text-mrk :ns ns))
    (ros::publish "image_marker" text-mrk)
    ))
