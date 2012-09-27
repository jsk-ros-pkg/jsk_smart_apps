#!/usr/bin/env roseus
(ros::load-ros-manifest "jsk_smart_gui")
(ros::load-ros-manifest "jsk_pcl_ros")

;;(ros::load-ros-manifest "jsk_android_gui")
;;(ros::load-ros-manifest "jsk_ipad_gui")

(ros::roseus "tablet-robot-controller")

(load "package://pr2eus_openrave/pr2eus-openrave.l")
(load "package://jsk_smart_gui/src/utils.l")
(load "package://jsk_smart_gui/src/tablet_callback.l")
(load "package://jsk_smart_gui/src/get_spot.l")
(load "package://jsk_2011_07_pr2_semantic/euslisp/actions.l")

(defconstant *default-deg* pi)

;; load eng*-scene.l , make *scene* and return spots
(defun get-current-spot ()
  (let ((place "/eng2") spots) ;;(place (ros::get-param "map_server/frame_id")
    (cond ((null place)
	   (ros::ros-warn "tablet_receiver:: Could not get current place")
	   (setq spots (list "/unknown/unknown/unknown")))
	  (t
	   (ros::ros-info "tablet_receiver:: the robot is in ~A now" place)
	   (load (ros::resolve-ros-path (format nil "package://jsk_maps/src/~A-scene.l" place)))
	   (setq *scene* (eval (read-from-string
				(format nil "(make-~A-scene)" ;;remove "/"
					(subseq (string-downcase place) 1)))))
	   (setq spots (mapcar #'(lambda(x) (send x :name)) (flatten (mapcar #'(lambda(x) (if (find-method x :spot) (send x :spots))) (send *scene* :objects)))))
	   ))
    spots))

;; create spots.js in jsk_ipad_gui/jslib
(defun make-spots-js (spots)
  (let* ((js_dir (ros::resolve-ros-path (format nil "package://jsk_ipad_gui/src/jsk_ipad_gui/jslib")))
	 (js_file (format nil "~A/spots.js" js_dir)))
    (ros::ros-info "tablet_receiver:: creating spots for tablet")
    (with-open-file
     (f js_file :direction :output)
     (format f "current_spots = new Array(~%")
     (format f "\"~A\"~%" (car spots))
     (dolist (spot (cdr spots))
       (format f ",\"~A\"~%" spot))
     (format f ")~%"))
    ))

(defun make-temp-spots-cb ()
  (let* ((mappose (send *tfl* :lookup-transform "/map" "/base_link"  (ros::time 0)))
	 (js_dir (ros::resolve-ros-path (format nil "package://jsk_maps/src")))
	 (js_file (format nil "~A/temp-scene.l" js_dir)))
    (if (null mappose)
	(progn (ros::ros-warn "robot not localized")
	       (return-from make-temp-spots-cb nil)))

    (unless (boundp '*temp-spots*)
      (setq *temp-spots* nil))
    (ros::ros-info "*temp-spots* : ~A" *temp-spots*)
    (push (list (send mappose :rot) (send mappose :pos)) *temp-spots*)
    ;; *temp-spots* : '('(rot pos) '(rot pos))
    (ros::ros-info "tablet_receiver:: creating temp spots for tablet")
    (with-open-file
     (f js_file :direction :output)

     (format f "(defun make-temp-f-scene ()~%")
     (format f " (let (scene map)~%")
     (format f "  (setq map (make-floor-plane (ros::resolve-ros-path \"package://jsk_maps/raw_maps/eng2-2f-0.05.pgm\") 50 -13000 -48500 :name \"/temp/f/floor-plane\")) ;;need this? ~%")
     (format f "  (make-cascoords-tree map ~%")
     (dotimes (i (length *temp-spots*))
       (let* ((spot (elt *temp-spots* i)) (spot-pos (cadr spot)))
	 (format f "`(:rot ~A :pos ,(float-vector ~A ~A ~A) :name \"/temp/f/place~A\")~%" (car spot) (aref spot-pos 0) (aref spot-pos 1) (aref spot-pos 2) i)))
     (format f "  ) ~%")
     (format f "  (setq scene (instance nav-scene-model :init :name \"/temp/f\" :objects (append (list map) (send map :descendants)))) ~%")
     (format f "  (send scene :map map) ;;need this? ~%")
     (format f "scene)) ~%")
     (format f "(defun make-temp-scene () ~%")
     (format f " (let (scene maps)  ~%")
     (format f "  (setq maps (list (make-temp-f-scene))) ~%")
     (format f "  (setq scene (instance building-model :init :name \"/temp\" :objects maps)) ~%")
     (format f ")) ~%")
     )
    (unix:sleep 1)
    (load js_file)
    (let* ((spots (mapcar #'(lambda(x) (send x :name)) (flatten (mapcar #'(lambda(x) (if (find-method x :spot) (send x :spots))) (send *scene* :objects)))))
	   (temp-scene (make-temp-scene))
	   (temp-spots (mapcar #'(lambda(x) (send x :name)) (flatten (mapcar #'(lambda(x) (if (find-method x :spot) (send x :spots))) (send temp-scene :objects))))))
      ;;(make-spots-js (append spots temp-spots))
      (make-spots-js temp-spots)
      (speak-jp "とうろくしました")
      )))

(defun move-to-spot (spot &key (frame-id "/world")) ;;&optional (spot "/eng2/7f/room73A3-front"))
  (let* ((s1 (send *scene* :find-object spot))
	 ;;(temp-scene (make-temp-scene))
	 (s2 nil));;(send temp-scene :find-object spot)))
    (cond ((null (or s1 s2))
	   (ros::ros-warn "no such place like ~A" spot)
	   (return-from move-to-spot nil))
	  (s1
	   (print spot)
	   (send *ri* :move-to (car s1) :frame-id frame-id))
	   ;;(send *scene* :transformation (send (car s1) :copy-worldcoords)))))
	  (s2
	   (print spot)
	   (send *ri* :move-to (car s2) :frame-id frame-id)))

    (speak-jp "とうちゃく")
    (if (boundp '*irtviewer*)
	(send *irtviewer* :draw-objects))))

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

(defun speak-jp (jp-str)
  (ros::publish "robotsound_jp"
                (instance sound_play::SoundRequest :init
                          :sound sound_play::SoundRequest::*say*
                          :command sound_play::SoundRequest::*play_once*
                          :arg jp-str)))

;; send (car (send *scene* :find-object "/eng2/7f/room73B2-sink-front")) :worldcoords
;; send *ri* :move-to  (car (send *scene* :find-object "/eng2/7f/room73B2-sink-front"))
(defun execute-main ()
  (if (not (boundp '*pr2*)) (pr2))
  (if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init)))

  (setq *node-debug* nil *visualize* nil *chosen-ret* nil)
  (setq *line* (make-list 2) *door* (make-list 3) *drawer* (make-list 3))

  ;;Default *ray_srv* is "/pointcloud_screenpoint_nodelet/screen_to_point"
  (setq *ray_srv* (ros::get-param "~ray_srv"))
  (setq *current-camera* "/camera/rgb")
  (change-inflation-range 0.15)

  ;;load all current spot and convert to javascript
 ;; make-spots-js (get-current-spot))
 (get-current-spot)

  (defvar *tfl* (instance ros::transform-listener :init))
  (defvar *tfb* (instance ros::transform-broadcaster :init))

  (ros::advertise "robotsound_jp" sound_play::SoundRequest 5)
  (ros::advertise "ray_coords" geometry_msgs::PoseStamped 1)
  (ros::advertise "tablet_marker_array" visualization_msgs::MarkerArray 10)
  (ros::advertise "tablet_marker" visualization_msgs::Marker 10)
  (ros::advertise "image_marker" image_view2::ImageMarker2 10)
  (subscribe-tablet)
  ;;(start-prosilica)
  (setq *time-count* (ros::time-now))
  (ros::rate 10)
  (ros::ros-info "begin tablet controller")

  (ros::spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep))
  (ros::exit))


;;(execute-main)



