#!/usr/bin/env roseus
(ros::load-ros-manifest "jsk_smart_gui")
(ros::load-ros-manifest "jsk_pcl_ros")
(ros::load-ros-manifest "roseus")

(ros::roseus "tablet_robot_controller")

(load "package://pr2eus/speak.l")
(load "package://jsk_smart_gui/src/tablet_callback.l")

(load "package://pr2eus_openrave/pr2eus-openrave.l")
(load "package://jsk_2011_07_pr2_semantic/euslisp/actions.l")

(defconstant *default-deg* pi)

(defun move-to-spot (spot &key (frame-id "/world"))
  (let* ((s1 (send *scene* :find-object spot)) (s2 nil) temp-scene)
    (if (boundp '*temp-spots*)
        (progn (setq temp-scene (make-temp-scene))
               (setq s2 (send temp-scene :find-object spot))))
    (cond ((null (or s1 s2))
           (ros::ros-warn "no such place like ~A" spot)
           (return-from move-to-spot nil))
          (s1
           (ros::ros-info "spot found: ~A" spot)
           (send *ri* :move-to (car s1) :frame-id frame-id))
          (s2
           (ros::ros-info "spot found: ~A" spot)
           (send *ri* :move-to (car s2) :frame-id frame-id)))
    (speak-jp "とうちゃく")))

(defun start-prosilica();;
  (setq px_offset 0 py_offset 0
	pwidth 2432 pheight 1824;;pidth 2448 pheight 2050
	pbinning_x 2 pbinning_y 2)
  (setq pmaxlength (/ 1 (sqrt (+ (* 640 640) (* 480 480)))))
  (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
    (ros::ros-info "start prosilica")
    (send req :config :strs
    	  (list (instance dynamic_reconfigure::StrParameter
    			  :init :name "trigger_mode" :value "streaming")))
    (send req :config :ints
	  (list (instance dynamic_reconfigure::IntParameter
			  :init :name "binning_x" :value pbinning_x)
		(instance dynamic_reconfigure::IntParameter
			  :init :name "binning_y" :value pbinning_y)
		(instance dynamic_reconfigure::IntParameter
			  :init :name "x_offset" :value px_offset)
		(instance dynamic_reconfigure::IntParameter
			  :init :name "y_offset" :value py_offset)
		(instance dynamic_reconfigure::IntParameter
			  :init :name "width" :value pwidth)
		(instance dynamic_reconfigure::IntParameter
			  :init :name "height" :value pheight)))
    (ros::service-call "prosilica_driver/set_parameters" req)
    ))

;; (send (car (send *scene* :find-object "/eng2/7f/room73B2-sink-front")) :worldcoords)
;; (send *ri* :move-to  (car (send *scene* :find-object "/eng2/7f/room73B2-sink-front")))
(defun execute-main ()
  (if (not (boundp '*pr2*)) (pr2))
  (if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init)))

  (setq *node-debug* nil *visualize* nil *chosen-ret* nil)
  (setq *line* (make-list 2) *door* (make-list 3) *drawer* (make-list 3))
  (setq *circlecoords* nil *circlediameter* nil *circleangle* nil)

  ;;Default *ray_srv* is "/pointcloud_screenpoint_nodelet/screen_to_point"
  (setq *ray_srv* (ros::get-param "~ray_srv"))
  ;;(setq *ray_srv* "/pointcloud_screenpoint_nodelet/screen_to_point")
  (setq *current-image* "/openni/rgb/image_color")
  (setq *current-camera* "/openni/rgb")
  (change-inflation-range 0.15)

  ;;load all current spot and convert to javascript/yaml
  ;;(make-spots-js (get-current-spot))
  (make-spots-yaml (get-current-spot))

  (defvar *tfl* (instance ros::transform-listener :init))
  (defvar *tfb* (instance ros::transform-broadcaster :init))
  (defvar *pcm* (instance pinhole-camera-model :init))

  (ros::advertise "robotsound_jp" sound_play::SoundRequest 5)
  (ros::advertise "ray_coords" geometry_msgs::PoseStamped 1)
  (ros::advertise "tablet_marker_array" visualization_msgs::MarkerArray 10)
  (ros::advertise "input_vel" geometry_msgs::Twist)
  (ros::advertise "image_marker" image_view2::ImageMarker2 10)
  (subscribe-tablet)
  ;;(start-prosilica)
  (setq *time-count* (ros::time-now))
  (setq *use-joy* nil)
  (ros::rate 10)
  (ros::ros-info "begin tablet controller")

  (ros::spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

  (while (ros::ok)
    (ros::spin-once)
    (send *ri* :spin-once)
    (ros::sleep))
  (ros::exit))

;;(execute-main)
