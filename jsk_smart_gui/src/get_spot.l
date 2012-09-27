#!/usr/bin/env roseus
(ros::load-ros-manifest "jsk_smart_gui")
;; Use this file for creating jsk_maps' spots
(ros::roseus "spot-creator")
#|
rostopic pub "/Tablet/GetSpot" std_msgs/Empty  -1
|#

(defun spot-init-setting ()
  (ros::subscribe "/Tablet/GetSpot"
		  std_msgs::Empty #'get-spot-cb)
  (setq *tfl* (instance ros::transform-listener :init))
  (setq *time-count* (ros::time-now))
  (make-spots-js (get-current-spot)))

(defun spot-execute-main ()
  (ros::rate 10)
  (ros::ros-info "get_spot::start running")
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    ))

(warn "(execute-main)~%")

(defun get-spot-cb (msg)
  (ros::ros-info "make-temp-spots-cb called")
  (make-temp-spots-cb))


(defun get-current-spot ()
  (let ((place "/eng2") spots) ;;(place (ros::get-param "map_server/frame_id")
    (cond ((null place)
	   (ros::ros-warn "get_spot:: Could not get current place")
	   (setq spots (list "/unknown/unknown/unknown")))
	  (t
	   (ros::ros-info "get_spot:: the robot is in ~A now" place)
	   (load (ros::resolve-ros-path (format nil "package://jsk_maps/src/~A-scene.l" place)))
	   (setq *scene* (eval (read-from-string
				(format nil "(make-~A-scene)" ;;remove "/"
					(subseq (string-downcase place) 1)))))
	   (setq spots (mapcar #'(lambda(x) (send x :name)) (flatten (mapcar #'(lambda(x) (if (find-method x :spot) (send x :spots))) (send *scene* :objects)))))
	   ))
    spots))

;; create spots.js in jsk_ipad_gui/jslib
(defun make-spots-js (spots)
  (let* ((js_dir (ros::resolve-ros-path (format nil "package://jsk_ipad_gui/src/jsk_ipad_gui/jslib"))) (js_file (format nil "~A/spots.js" js_dir)))
    (ros::ros-info "get_spot:: creating spots for Tablet")
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

    (ros::ros-info "get_spot:: creating temp spots for Tablet")
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
      ;; (make-spots-js (append spots temp-spots))
      (make-spots-js temp-spots)
      ;; (speak-jp "とうろくしました")
      )))
