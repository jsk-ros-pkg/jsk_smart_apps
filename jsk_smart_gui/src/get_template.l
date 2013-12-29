#!/usr/bin/env roseus
(ros::load-ros-manifest "jsk_smart_gui")
(ros::load-ros-manifest "jsk_pcl_ros")
(ros::load-ros-manifest "dynamic_tf_publisher")
(ros::load-ros-manifest "object_snapshotter")
;; This script depends on tablet_callback.l
;; Use when creating template from imagesift
(ros::roseus "template_creator")
(load "package://jsk_smart_gui/src/utils.l")
(load "package://jsk_smart_gui/src/extra_actions.l")
(load "package://pr2eus_openrave/pr2eus-openrave.l")
(load "package://jsk_2011_07_pr2_semantic/euslisp/actions.l")

(defun ros::coords->tf-pose-stamped0 (coords id)
  (let ((pose (ros::coords->tf-pose coords)))
    (instance geometry_msgs::PoseStamped :init
              :header (instance std_msgs::header :init
                                :frame_id id
                                :stamp (ros::time 0))
              :pose pose)))

(defun template_init ()
  (get_template_init)
  (use_template_init)
  (pr2) (setq *ri* (instance pr2-interface :init))
  )
(defun main_loop ()
  (do-until-key
   (ros::spin-once)
   (ros::sleep)))

(defun setup ()
  (change-inflation-range 0.15)
  (template_init)
  (subscribe-tablet)
  (make-spots-yaml (get-current-spot))
  (main_loop)
  )

(defun get_template_init ()
  (defvar *ray_srv* "/pointcloud_screenpoint_nodelet/screen_to_point")
  (defvar *tfl* (instance ros::transform-listener :init))
  (defvar *tfb* (instance ros::transform-broadcaster :init))
  (defvar *width* 0)
  (defvar *height* 0)
  (defvar *template-origin* (make-coords))
  (defvar *username* "chen")
  (setq *current-camera* "/openni/rgb/image_color")
  (setq *time-count* (ros::time-now))
  (ros::rate 0.5)
  (setq *flag1* nil *flag2* nil *flag3* nil *flag4* nil)
  (setq *flag_push* nil)
  (ros::advertise "image_marker" image_view2::ImageMarker2 100)
  (ros::advertise "tablet_marker" visualization_msgs::Marker 10)
  (ros::advertise "tablet_marker_array" visualization_msgs::MarkerArray 10)
  )

(defun use_template_init ()
  ;; (ros::subscribe "/ObjectDetection1"
  ;;   posedetection_msgs::ObjectDetection #'use_template_cb1)

  ;; (ros::subscribe "/ObjectDetection2"
  ;;        posedetection_msgs::ObjectDetection #'use_template_cb2)

  ;; (ros::subscribe "/ObjectDetection3"
  ;;                 posedetection_msgs::ObjectDetection #'use_template_cb3)

  ;; (ros::subscribe "/ObjectDetection4"
  ;;                 posedetection_msgs::ObjectDetection #'use_template_cb4)

  ;; (ros::subscribe "/ObjectDetection"
  ;;                 posedetection_msgs::ObjectDetection #'use_template_cb_push)

  ;; (ros::subscribe "/ObjectDetection"
  ;;                 posedetection_msgs::ObjectDetection #'use_template_cb_push_no)
;;(ros::unsubscribe "/ObjectDetection")
  )

(defun use_yes ()
  (ros::subscribe "/ObjectDetection"
                  posedetection_msgs::ObjectDetection #'use_template_cb_push))
(defun use_no ()
  (ros::subscribe "/ObjectDetection"
                  posedetection_msgs::ObjectDetection #'use_template_cb_push_no))

(defun use_template_cb_push_no (msg) )

;;(move-to-spot "/eng2/7f/room73B2-cooking-header")
(defun use_template_cb_push (msg)
  (let* ((poses (car (send msg :objects)))
         (pose (send poses :pose))
         (frame-id (send msg :header :frame_id))
         (cam-cds (instance geometry_msgs::PoseStamped :init))
         (taskname "kettle") (arm :rarm)
         target-cds)
    (ros::ros-info "using template_push")
    (setq *username* taskname)
    (load "push-variables.l")

    (unless *flag_push*
      ;;(setusrtask *username* taskname)
      (unix::sleep 1)
      (setappealtask taskname)
      (setq *flag_push* t))
    (send cam-cds :header :stamp (ros::time 0))
    (send cam-cds :header :frame_id frame-id)
    (send cam-cds :pose pose)

    (setq *cam-cds* cam-cds)
    (send *tfl* :wait-for-transform frame-id "base_footprint" (ros::time 0) 5)
    (setq *detected-template-origin* (send *tfl* :transform-pose "base_footprint" cam-cds))

    (setq target-cds (make-coords :pos (send (send (send *detected-template-origin* :copy-worldcoords) :translate *transform-diff* :world) :pos) :rot *hand-rot*))

    (setq point2d (point2pixel :px (aref (send target-cds :worldpos) 0) :py (aref (send target-cds :worldpos) 1) :pz (aref (send target-cds :worldpos) 2)))

    (setq new-target-cds (screenpoint-srvcall (elt point2d 0) (elt point2d 1)))

    (setq new-target-cds-transformed (make-coords :pos (float-vector (aref (send new-target-cds :worldpos) 0) (aref (send target-cds :worldpos) 1) (aref (send target-cds :worldpos) 2)) :rot *hand-rot*))

    ;; (show-marker
    ;;   (send (send (send *detected-template-origin* :copy-worldcoords) :translate *transform-diff* :world) :pos)
    ;;   :col #f(1 1 0))

    (show-markers
      (list (send (send *detected-template-origin* :copy-worldcoords) :pos)
            (send (send target-cds :copy-worldcoords) :pos)
            (send (send new-target-cds :copy-worldcoords) :pos)
            (send (send new-target-cds-transformed :copy-worldcoords) :pos)
            ) :col #f(1 1 0))

;;    (visualize-text-image-from-pos "StopIH/Push(1)" (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 0 0) :world) :pos))
    (visualize-text-image-from-pos "OffSwitch/Push(1)" (send (send (send new-target-cds-transformed :copy-worldcoords) :translate #f(0 0 0) :world) :pos))
    (visualize-point-image-from-coords
     (list
      (send new-target-cds-transformed :copy-worldcoords))
     :counter 100)

;;    (send *detected-template-origin* :rotate pi/2 :y)

    (ros::ros-warn "use this?")
    (unless (y-or-n-p)
      (ros::ros-error "return")
      (setq *flag_push* nil)
      (pr2-tuckarm-pose :rarm)
      (return-from use_template_cb_push nil))

    (setq *current-arm* arm)

;;    (move-arm arm (send (send (send *detected-template-origin* :copy-worldcoords) :translate *transform-diff* :world) :translate #f(-100 0 0)) :rotation-axis nil :rthre (deg2rad 55) :sec 3000)
;;    (move-arm arm (send (send (send *detected-template-origin* :copy-worldcoords) :translate *transform-diff* :world) :translate #f(10 0 0)) :rotation-axis nil :rthre (deg2rad 35) :sec 1000)

    (print 'movearm1)
    
    (if *changeflag*
        (progn
          (move-arm arm (send (send new-target-cds :copy-worldcoords) :translate #f(-150 0 0) :world) :rotation-axis t :rthre (deg2rad 35) :sec 3000)
          (print 'movearm2)
          (move-arm arm (send (send new-target-cds :copy-worldcoords) :translate #f(50 0 0) :world) :rotation-axis t :rthre (deg2rad 15) :sec 1000)
          (setq *changeflag* nil)
          )
      (progn
        (move-arm arm (send (send new-target-cds-transformed :copy-worldcoords) :translate #f(-150 0 0) :world) :rotation-axis t :rthre (deg2rad 35) :sec 3000)
        (print 'movearm2)
        (move-arm arm (send (send new-target-cds-transformed :copy-worldcoords) :translate #f(50 0 0) :world) :rotation-axis t :rthre (deg2rad 15) :sec 1000)
        (setq *changeflag* t)
        ))
    ))
(setq *changeflag* t)
(defun PushOnceExecute (x y arm)
  (let* ((c (screenpoint-srvcall x y))
         (time-now (send (ros::time-now) :to-sec))
         )
    (unless (boundp '*template-origin*)
      (ros::ros-error "template-origin not set!!")
      (return-from PuchOnceExecute nil))

    (setq *transform-diff* (v- (send (send c :copy-worldcoords) :pos) (send (send *template-origin* :copy-worldcoords) :pos)))

    (setq *hand-rot* (send (send c :copy-worldcoords) :rot))

    (dump-loadable-structure
     "push-variables.l"
     *image-name* *width* *height* *hand-rot* *transform-diff*)

    (let* ((launch_dir (ros::resolve-ros-path (format nil "package://jsk_smart_gui/src")))
           (launch_file (format nil "~A/test_push.launch" launch_dir)))
      (ros::ros-info "tablet_receiver:: creating launch for tablet")
      (with-open-file
       (f launch_file :direction :output)
       (format f "<launch>~%")
       (format f "<node pkg=\"jsk_perception\" type=\"point_pose_extractor\"~%")
       (format f "        output=\"screen\" name=\"point_pose_extractor_test\" >~%")
       (format f "   <remap from=\"ImageFeature0D\" to=\"/left/openni_feature\" />~%")
       (format f "   <remap from=\"ObjectDetection\" to=\"ObjectDetection\" />~%")
       (format f "   <param name=\"template_filename\" value=\"~A\"/>~%" *image-name*)
       (format f "   <param name=\"object_width\" value=\"~A\"/>~%" (/ *width* 1000.0))
       (format f "   <param name=\"object_height\" value=\"~A\"/>~%" (/ *height* 1000.0))
       (format f "   <param name=\"viewer_window\" value=\"true\"/>~%")
       (format f "</node>~%")
       (format f "</launch>~%")
       ))
    (print c)
    (pr2-tuckarm-pose arm)
    (move-arm arm (send (send c :copy-worldcoords) :translate #f(-100 0 0) :world) :rotation-axis t :rthre (deg2rad 55) :sec 3000)
    (move-arm arm (send (send c :copy-worldcoords) :translate #f(10 0 0) :world) :rotation-axis t :rthre (deg2rad 35) :sec 1000)
    ))

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
    ;; (setq img_res (save_img (format nil "~A/.ros/~A.jpg" (unix:getenv "HOME") time-now)))
    ;; (setq *image-name* (format nil "~A/.ros/~A.jpg" (unix:getenv "HOME") time-now))

    (setq img_res (save_img (format nil "/tmp/~A.jpg" time-now)))
    (setq *image-name* (format nil "/tmp/~A.jpg" time-now))

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
    ;; calculate offset in push once

    ;; call settemplate for pointposeextractor
    (ros::set-param "/point_pose_extractor_test/template_filename" (format nil "~A/.ros/~A.jpg" (unix:getenv "HOME") time-now))
    (ros::set-param "/point_pose_extractor_test/object_width" (/ *width* 1000.0))
    (ros::set-param "/point_pose_extractor_test/object_height" (/ *height* 1000.0))
    (unix::system (format nil "rosparam set /point_pose_extractor_test/viewer_window true"))
    (dump-loadable-structure
     (format nil "get-template-~d.l" time-now)
     *width* *height*)
    ))

;; (show-marker (send *detected-template-origin* :pos))

(defun use_template_cb1(msg)
  (let* ((poses (car (send msg :objects)))
         (pose (send poses :pose))
         (frame-id (send msg :header :frame_id))
         (cam-cds (instance geometry_msgs::PoseStamped :init))
         (taskname "fridge")
         (rad (/ pi 3))
         )
    (ros::ros-info "using template1")

    (unless *flag1*
      (setusrtask *username* taskname)
      (unix::sleep 1)
      (setappealtask taskname)
      (setq *flag1* t))

    (send cam-cds :header :stamp (ros::time 0))
    (send cam-cds :header :frame_id frame-id)
    (send cam-cds :pose pose)

    (send *tfl* :wait-for-transform frame-id "base_footprint" (ros::time 0) 5)
    (setq *detected-template-origin* (send *tfl* :transform-pose "base_footprint" cam-cds))

    ;; rviz
    (show-line (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -45 -150) :world) :pos) (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -45 -450) :world) :pos))
    ;; image_view2
    (visualize-text-image-from-pos "Fridge/Open" (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 0 0) :world) :pos))
    (visualize-line-image-from-coords
     (list (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -45 -150) :world)
	   (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -45 -450) :world)) :counter 100)

    (setq *door* (list 
                  (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -545 -150) :world) :pos)
                  (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -545 -450) :world) :pos) 0))

    (setq rad (/ pi 3))
    (ros::ros-error "excute?")
    (read-line)
    (setq *current-arm* :rarm)
    (OpenDoorInputEusExecute :rarm (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -45 -300) :world) :pos) rad :grab t)
    (OpenDoorEusExecute :rarm rad) ;;todo
  ))

(defun use_template_cb2(msg)
  (let* ((poses (car (send msg :objects)))
	 (pose (send poses :pose))
	 (frame-id (send msg :header :frame_id))
	 (cam-cds (instance geometry_msgs::PoseStamped :init))
         (taskname "ih")
	 )
    (ros::ros-info "using template2")
    (unless *flag2*
      (setusrtask *username* taskname)
      (unix::sleep 1)
      (setappealtask taskname)
      (setq *flag2* t))

    (send cam-cds :header :stamp (ros::time 0))
    (send cam-cds :header :frame_id frame-id)
    (send cam-cds :pose pose)

    (send *tfl* :wait-for-transform frame-id "base_footprint" (ros::time 0) 5)
    (setq *detected-template-origin* (send *tfl* :transform-pose "base_footprint" cam-cds))

    ;; rviz
    (show-markers
     (list
      (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -65 -80) :world) :pos)
      (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -135 -70) :world) :pos)
      (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -205 -80) :world) :pos)
      ) :col #f(1 1 0))

    ;; image_view2
    (visualize-text-image-from-pos "IH button/Push(3)" (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 0 0) :world) :pos))
    (visualize-point-image-from-coords
     (list (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -65 -80) :world)
           (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -135 -70) :world)
           (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -205 -80) :world))  :counter 100)
    ))

(defun use_template_cb3(msg)
  (let* ((poses (car (send msg :objects)))
	 (pose (send poses :pose))
	 (frame-id (send msg :header :frame_id))
	 (cam-cds (instance geometry_msgs::PoseStamped :init))
         (taskname "cupboard")
	 )
    (ros::ros-info "using template3")
    (unless *flag3*
      (setusrtask *username* taskname)
      (unix::sleep 1)
      (setappealtask taskname)
      (setq *flag3* t))


    (send cam-cds :header :stamp (ros::time 0))
    (send cam-cds :header :frame_id frame-id)
    (send cam-cds :pose pose)

    (send *tfl* :wait-for-transform frame-id "base_footprint" (ros::time 0) 5)
    (setq *detected-template-origin* (send *tfl* :transform-pose "base_footprint" cam-cds))

    ;; rviz
    (show-line (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -120 -200) :world) :pos) (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -120 -250) :world) :pos))
    ;; (show-line (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(-130 -120 -200) :world) :pos) (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(-130 -120 -250) :world) :pos))
    ;; image_view2
    (visualize-text-image-from-pos "CupBoard/Open" (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 0 0) :world) :pos))

    (visualize-line-image-from-coords
     (list (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -120 -200) :world)
           (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -120 -250) :world)) :counter 100)
    ;; (visualize-line-image-from-coords
    ;;  (list (send (send *detected-template-origin* :copy-worldcoords) :translate #f(-130 -120 -200) :world)
    ;;        (send (send *detected-template-origin* :copy-worldcoords) :translate #f(-130 -120 -250) :world)) :counter 100)
    ))

(defun use_template_cb4(msg)
  (let* ((poses (car (send msg :objects)))
	 (pose (send poses :pose))
	 (frame-id (send msg :header :frame_id))
	 (cam-cds (instance geometry_msgs::PoseStamped :init))
         (taskname "microwave")
	 )
    (ros::ros-info "using template4")
    (unless *flag4*
      (setusrtask *username* taskname)
      (unix::sleep 1)
      (setappealtask taskname)
      (setq *flag4* t))

    (send cam-cds :header :stamp (ros::time 0))
    (send cam-cds :header :frame_id frame-id)
    (send cam-cds :pose pose)

    (send *tfl* :wait-for-transform frame-id "base_footprint" (ros::time 0) 5)
    (setq *detected-template-origin* (send *tfl* :transform-pose "base_footprint" cam-cds))

    ;; rviz
    (show-line (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 180 30) :world) :pos) (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -120 30) :world) :pos) :ns "3d_lines")
    (show-markers (list (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(-140 -120 -40) :world) :pos)) :ns "3d_points")

    ;; image_view2
    (visualize-text-image-from-pos "MicroWave/Open" (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 0 60) :world) :pos))
    (visualize-text-image-from-pos "MicroWave/Push" (send (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 0 20) :world) :pos) :counter 1)
    (visualize-line-image-from-coords
     (list (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 180 30) :world)
           (send (send *detected-template-origin* :copy-worldcoords) :translate #f(0 -120 30) :world)) :counter 100)
    (visualize-point-image-from-coords
     (list (send (send *detected-template-origin* :copy-worldcoords) :translate #f(-140 -120 -40) :world)) :counter 200)
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

(defun set_template_sample2 () ;;for IH
  (let ((width 207.005) (height 202.051))
  (ros::set-param "/point_pose_extractor_test/object_width" (/ width 1000.0))
  (ros::set-param "/point_pose_extractor_test/object_height" (/ height 1000.0))
  (ros::set-param "/point_pose_extractor_test/template_filename" (format nil "/home/chen/.ros/1.358671e+09.jpg"))
  (unix::system (format nil "rosparam set /point_pose_extractor_test/viewer_window true"))
  ))

(defun set_template_sample3 () ;;for cupboard
  (let ((width 184.302) (height 250.753))
  (ros::set-param "/point_pose_extractor_test/object_width" (/ width 1000.0))
  (ros::set-param "/point_pose_extractor_test/object_height" (/ height 1000.0))
  (ros::set-param "/point_pose_extractor_test/template_filename" (format nil "/home/chen/.ros/1.358682e+09.jpg"))
  (unix::system (format nil "rosparam set /point_pose_extractor_test/viewer_window true"))
  ))

(defun set_template_sample4 () ;;for microwave
  (let ((width 207.167) (height 272.828))
  (ros::set-param "/point_pose_extractor_test/object_width" (/ width 1000.0))
  (ros::set-param "/point_pose_extractor_test/object_height" (/ height 1000.0))
  (ros::set-param "/point_pose_extractor_test/template_filename" (format nil "/home/chen/.ros/1.358684e+09.jpg"))
  (unix::system (format nil "rosparam set /point_pose_extractor_test/viewer_window true"))
  ))

;;-----------------------------------------------------------------------

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


(defun visualize-line-image-from-coords
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

(defun visualize-point-image-from-coords
  (cds-list &key (counter 0) (ns nil) (lifetime 10)
	    (outline-colors (list (instance std_msgs::ColorRGBA :init :r 1.0 :g 0.0 :b 0.0 :a 1.0))))
  (let* ((mrk (instance image_view2::ImageMarker2 :init))
	 point-list)
    (send mrk :header :stamp (ros::time-now))
    (send mrk :type image_view2::ImageMarker2::*POINTS3D*)
    (send mrk :points3D :header :frame_id "base_footprint")
    (send mrk :lifetime (ros::Time lifetime))
    (send mrk :id counter)
    (dolist (cds cds-list)
      ;;(ros::ros-warn "cds: ~A" (send cds :pos))
      (push (ros::pos->tf-point (send cds :worldpos)) point-list))
    (send mrk :points3D :points point-list)
    (send mrk :scale 15)
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


;; (setusrtask "chen" "fridge")
;; (setusrtask "chen" "im")
;; (setusrtask "chen" "cupboard")
;; (setusrtask "chen" "microwave")
;; (setappealtask "fridge")

(defun setappealtask (name)
  (unix::system (format nil "rosparam set /Tablet/Found '[~A]'" name)))

(defun setusrtask (usrname taskname)
  (let ((usrlst (ros::get-param "/Tablet/UserList"))
        (ulstr "'[")
        (fridgeaction (list "CloseDoor"))
        (ihaction (list "PushIHButton1" "PushIHButton2" "PushIHButton3"))
        (cupboardaction (list "OpenCupboardDoor1"))
        (microwaveaction (list "PushMicroWaveButton1" "OpenMicrowaveDoor1"))
        (tasklst "'[DoNothing") (actlst nil))
    (setq ulstr (format nil "~A~A" ulstr usrname))
    (dolist (usr usrlst)
      (if (not (string= usr usrname))
          (setq ulstr (format nil "~A, ~A" ulstr usr))))
    (setq ulstr (format nil "~A]'" ulstr))
    ;;    (unix::system (format nil "rosparam set /Tablet/UserList ~A" ulstr))
    (case (read-from-string taskname)
      ('FRIDGE (setq actlst fridgeaction))
      ('IH (setq actlst ihaction))
      ('CUPBOARD (setq actlst cupboardaction))
      ('MICROWAVE (setq actlst microwaveaction))
      (t (ros::ros-error "~A is not found" taskname)
         (return-from setusrtask nil)))
    (dolist (act actlst)
      (setq tasklst (format nil "~A, ~A" tasklst act)))
    (setq tasklst (format nil "~A]'" tasklst))
    (unix::system (format nil "rosparam set /Tablet/User/~A ~A" usrname tasklst))
    (unix::system (format nil "rosparam set /Tablet/UserList ~A" ulstr))
    (ros::ros-info (format nil "set ~A ~A param" usrname taskname))
    ))

(defun pr2-tuckarm-pose (&rest args)
  (no-head-reset-pose)
  )

(defun no-head-reset-pose (&rest args)
  (let* ((current-arm (check-tuckarm-pose :thre 40)) ;; nil rarm larm
         (free-arm (or (car args) current-arm :larm))
         (msec 2000))
    (send *pr2* :larm :angle-vector #f(60.0 74.0 70.0 -120.0 20.0 -30.0 180.0))
    (send *pr2* :rarm :angle-vector #f(-60.0 74.0 -70.0 -120.0 -20.0 -30.0 180.0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)))


