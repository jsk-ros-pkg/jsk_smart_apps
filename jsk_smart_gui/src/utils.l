;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;utils
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun ros::tf-translation->pos (trans)
  (float-vector (send trans :x) (send trans :y) (send trans :z)))

(defun is-old-msg (msg)
  (let ((htime (send msg :header :stamp)))
    (cond ((> 0 (send (ros::time- htime *time-count*) :to-sec))
	   (ros::ros-debug "message is old, return with no action")
	   t))))

(defun check-not-ray-srv (&optional (srv *ray_srv*))
  (not (ros::wait-for-service srv 2)))

(defun point2pixel (&key (px 1000.0) (py 0.0) (pz 1000.0))
  (let ((req (instance jsk_smart_gui::point2screenpointRequest :init))
  	(po (instance geometry_msgs::PointStamped :init)) res)
    (send po :header :seq 0)
    (send po :header :stamp (ros::time 0))
    (send po :header :frame_id "base_footprint")
    (send po :point :x (/ px 1000.0))
    (send po :point :y (/ py 1000.0))
    (send po :point :z (/ pz 1000.0))
    (send req :point po)
    (setq res (ros::service-call "TransformPixelRequest" req))
    (list (send res :x) (send res :y))
    ))

(defun generate-slide-path
  (start-point point1 point2 path-length &key (div 10))
  (let* ((di (normalize-vector (v- point2 point1)))
         (ln (make-line point1 point2))
         (fp (send ln :point (send ln :foot start-point))))
    (let* ((ret nil)
           (fc (make-coords :pos (copy-object start-point))))
      (push (send fc :copy-worldcoords) ret)
      (scale (/ path-length (float div)) di di)
      (dotimes (i div)
        (send fc :translate di)
        (push (send fc :copy-worldcoords) ret))
      (nreverse ret))))

(defun generate-circle-path
  (start-point point1 point2 angle &key (div 20))
  (let* ((axis (normalize-vector (v- point1 point2)))
	 (ln (make-line point1 point2))
	 (fp (send ln :point (send ln :foot start-point))))
    (unless (and (float-vector-p start-point)
		 (= (length start-point) 3)
		 (float-vector-p axis)
		 (= (length axis) 3))
      (return-from generate-circle-path nil))
    (setq *tmp-fp* fp);;
    (let* ((ret nil)
	   (fc (make-coords :pos fp))
	   (trans (send fc :transformation (make-coords :pos start-point))))
      (dotimes (i div)
	(send fc :rotate (/ angle div) axis)
	(push
	 (send (send (send fc :copy-worldcoords) :transform trans) :rotate pi/2 :x)
	 ret))
      (nreverse ret))))

(defun set-z-coordinate-axis (target-coords v &optional (axis :z))
  (let ((d (case axis
	     (:x (matrix-column (send target-coords :worldrot) 0))
	     (:y (matrix-column (send target-coords :worldrot) 1))
	     (:z (matrix-column (send target-coords :worldrot) 2)))))
    (setq v (normalize-vector v))
    (if (< -1.0 (v. v d) 1.0)
	(send target-coords :rotate (acos (v. v d)) (v* d v) :world))
    target-coords))

;; (defun point-srvcall (msg) ;; deprecated
;;   (let* ((x (send msg :point :x)) (y (send msg :point :y))
;; 	 (req (instance jsk_pcl_ros::TransformScreenpointRequest :init
;; 			:x x :y y)) res)
;;     ;;(ros::wait-for-service *ray_srv* 1)
;;     (setq res (ros::service-call *ray_srv* req))
;;     (if (equal (send res :header :frame_id) "")
;; 	(progn
;; 	  (ros::ros-warn "detected service fail")
;; 	  (return-from point-srvcall nil)))
;;     (let* ((frame (send res :header :frame_id))
;; 	   (header (instance std_msgs::header :init
;; 			     :stamp (ros::time-now)
;; 			     :frame_id frame))
;; 	   (p1 (ros::tf-point->pos (send res :point)))
;; 	   (av (ros::tf-translation->pos (send res :vector)))
;; 	   ;; (xv #f(1 0 0)) (bv (v* av xv)) (cv (v* av bv))
;; 	   ;; (dm (matrix (normalize-vector av)
;; 	   ;; 	       (normalize-vector bv)
;; 	   ;; 	       (normalize-vector cv)))
;; 	   (cam-cds (send *tfl* :lookup-transform
;; 			  "base_footprint" frame (ros::time 0)))
;; 	   ;; (base-cds (send cam-cds :transform (make-coords :pos p1 :rot dm)))
;; 	   (ros::ros-error "error:~A" cam-cds)
;; 	   (if (null cam-cds)
;; 	       (progn
;; 		 (ros::ros-warn "detected service fail2")
;; 		 (return-from point-srvcall nil)))
;; 	   (base-cds (send cam-cds :transform
;; 			   (set-z-coordinate-axis
;; 			    (make-coords :pos p1) (scale 1.0 av) :x)))
;; 	   (rmsg (instance geometry_msgs::PoseStamped :init)))
;;       (ros::ros-warn "estimated cds is  ~A" base-cds)
;;       (send rmsg :header header)
;;       (send rmsg :pose (ros::coords->tf-pose base-cds))
;;       (ros::publish "ray_coords" rmsg)
;;       base-cds
;;       )))

(defun screenpoint-srvcall (x y)
  (let* ((req (instance jsk_pcl_ros::TransformScreenpointRequest :init
			:x x :y y)) res)
    ;;(ros::wait-for-service *ray_srv* 1)
    (setq res (ros::service-call *ray_srv* req))
    (if (equal (send res :header :frame_id) "")
	(progn
	  (ros::ros-warn "detected service fail")
	  (return-from screenpoint-srvcall nil)))
    (let* ((frame (send res :header :frame_id))
	   ;;(header (instance std_msgs::header :init
		;;	     :stamp (ros::time-now)
		;;	     :frame_id frame))
	   (p1 (ros::tf-point->pos (send res :point)))
	   (av (ros::tf-translation->pos (send res :vector)))
	   (cam-cds (send *tfl* :lookup-transform
			  "base_footprint" frame (ros::time 0)))
	   (ros::ros-error "error:~A" cam-cds)
	   (if (null cam-cds)
	       (progn
		 (ros::ros-warn "detected service fail2")
		 (return-from point-srvcall nil)))
	   (base-cds (send cam-cds :transform
			   (set-z-coordinate-axis
			    (make-coords :pos p1) (scale 1.0 av) :x)))
	   ;;(rmsg (instance geometry_msgs::PoseStamped :init))
	   )
      (ros::ros-warn "estimated cds is  ~A" base-cds)
      ;;(send rmsg :header header)
      ;;(send rmsg :pose (ros::coords->tf-pose base-cds))
      ;;(ros::publish "ray_coords" rmsg)
      base-cds
      )))

(defun visualize-object-image-from-edges
  (edges &key (counter 0) (ns nil) (lifetime 10)
         (outline-colors (list (instance std_msgs::ColorRGBA :init :r 1.0 :g 0.0 :b 0.0 :a 1.0))))
  (let* ((mrk (instance image_view2::ImageMarker2 :init))
         point-list)
    (send mrk :header :stamp (ros::time-now))
    (send mrk :type image_view2::ImageMarker2::*LINE_LIST3D*)
    (send mrk :points3D :header :frame_id "base_footprint")
    (send mrk :lifetime (ros::Time lifetime))
    (send mrk :width 1.0)
    (send mrk :id counter)
    (dolist (edge edges)
      (let ((v (send edge :vertices)))
        (dolist (p v)
          (push (ros::pos->tf-point p) point-list))))
    (send mrk :points3D :points point-list)
    (send mrk :outline_colors outline-colors)
    (if ns (send mrk :ns ns))
    (ros::publish "image_marker" mrk)
    ))


(defun visualize-line (p1 p2)
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :header :stamp (ros::time-now))
    (send mrk :ns "tablet_line")
    (send mrk :id 1)
    (send mrk :type image_view2::ImageMarker2::*LINE_STRIP*)
    (send mrk :action image_view2::ImageMarker2::*ADD*)
    (send mrk :points (list (instance geometry_msgs::Point :init
				      :x (elt p1 0) :y (elt p1 1))
			    (instance geometry_msgs::Point :init
				      :x (elt p2 0) :y (elt p2 1))))
    (send mrk :outline_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 1.0))
    (ros::publish "image_marker" mrk)))

(defun stop-visualize-line ()
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :header :stamp (ros::time-now))
    (send mrk :ns "tablet_line")
    (send mrk :id 1)
    (send mrk :type image_view2::ImageMarker2::*LINE_STRIP*)
    (send mrk :action image_view2::ImageMarker2::*REMOVE*)
    (ros::publish "image_marker" mrk)))

(defun visualize-point
  (x y &key (i 1) ((:lifetime lf) 5) (r 0.0) (g 0.0) (b 0.0) (a 1.0) (sca 1))
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :header :stamp (ros::time-now))
    (send mrk :id i)
    (send mrk :type image_view2::ImageMarker2::*CIRCLE*)
    (send mrk :action image_view2::ImageMarker2::*ADD*)
    (send mrk :position (instance geometry_msgs::Point :init :x x :y y))
    (send mrk :outline_color (instance std_msgs::ColorRGBA :init :r r :g g :b b :a a))
    (send mrk :scale sca)
    (send mrk :lifetime (ros::Time lf))
    (ros::publish "image_marker" mrk)))

(defun visualize-frame (frame &key ((:lifetime lf) 20))
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :header :stamp (ros::time-now))
    (send mrk :type image_view2::ImageMarker2::*FRAMES*)
    (send mrk :frames frame)
    (send mrk :lifetime (ros::Time lf))
    (ros::publish "image_marker" mrk)))

(defun visualize-circle (cds scale &key (angle 360.0) (width 3) (outline-color (float-vector 1 0 0)) fill-color (lifetime 10))
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :header :stamp (ros::time-now))
    (send mrk :ns "tablet_circle")
    (send mrk :id 1)
    (send mrk :type image_view2::ImageMarker2::*CIRCLE3D*)
    (send mrk :action image_view2::ImageMarker2::*ADD*)
    (send mrk :pose :header :frame_id "base_footprint")
    (send mrk :pose :pose (ros::coords->tf-pose cds))
    (send mrk :scale (/ scale 1000.0))
    (send mrk :width width)
    (send mrk :outline_color (vector->rgba outline-color))
    (send mrk :lifetime (ros::Time lifetime))
    (when fill-color
      (send mrk :filled 1)
      (send mrk :fill_color (vector->rgba fill-color)))
    (unless (>= angle 360.0)
      (send mrk :arc 1)
      (send mrk :angle angle))
    (ros::publish "image_marker" mrk)
    ))

(defun stop-visualize-circle ()
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :header :stamp (ros::time-now))
    (send mrk :ns "tablet_circle")
    (send mrk :id 1)
    (send mrk :action image_view2::ImageMarker2::*REMOVE*)
    (ros::publish "image_marker" mrk)
    ))

(defun visualize-text (text pos &key (scale 1.0) (color (float-vector 1 0 0)) (lifetime 10))
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :header :stamp (ros::time-now))
    (send mrk :ns "tablet_text")
    (send mrk :id 1)
    (send mrk :type image_view2::ImageMarker2::*TEXT3D*)
    (send mrk :action image_view2::ImageMarker2::*ADD*)
    (send mrk :text text)
    (send mrk :position3D :header :frame_id "base_footprint")
    (send mrk :position3D :point (ros::pos->tf-point pos))
    (send mrk :scale scale)
    (send mrk :outline_color (vector->rgba color))
    (send mrk :lifetime (ros::Time lifetime))
    (ros::publish "image_marker" mrk)
    ))

(defun stop-visualize-text ()
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :header :stamp (ros::time-now))
    (send mrk :ns "tablet_text")
    (send mrk :id 1)
    (send mrk :action image_view2::ImageMarker2::*REMOVE*)
    (ros::publish "image_marker" mrk)
    ))

(defun stop-visualize ()
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :header :stamp (ros::time-now))
    (send mrk :id -1)
    (send mrk :action image_view2::ImageMarker2::*REMOVE*)
    (ros::publish "image_marker" mrk)))

;;frame_id is based on "base_footprint"
(defun show-line (p1 p2 &key (scale 15) ((:lifetime lf) 60) (ns "3d_line") (col #f(1 0 0)) (alpha 0.5))
  (let* ((header (instance std_msgs::header :init
			   :stamp (ros::time-now) :frame_id "base_footprint"))
         (li-msg (line->marker-msg (list p1 p2) header :scale scale
				   :color col :alpha alpha))
	 (msg (instance visualization_msgs::MarkerArray :init)))
    (send li-msg :ns ns)
    (send li-msg :lifetime (ros::time lf))
    (send msg :markers (list li-msg))
    (ros::publish "tablet_marker_array" msg)
    ))

;;frame_id is based on "base_footprint"
;; (defun show-marker (p &key ((:lifetime lf) 60) (col #f(1 0 1)) (ns "3d_point"))
;;   (let* ((header (instance std_msgs::header :init
;; 			   :stamp (ros::time-now) :frame_id "base_footprint"))
;; 	 (msg (sphere->marker-msg (make-sphere 20 :pos p) header :color col :alpha 1.0)))
;;     (send msg :lifetime (ros::time lf))
;;     (send msg :ns ns)
;;     (ros::publish "tablet_marker" msg)
;;     ))

(defun show-markers (pl &key ((:lifetime lf) 60) (col #f(0 0 1)) (ns "3d_points"))
  (unless (listp pl)
    (ros::ros-warn "show-markers wants a list of points")
    (return-from show-markers nil))
  (let* ((header (instance std_msgs::header :init
			   :stamp (ros::time-now) :frame_id "base_footprint"))
	 (msg (instance visualization_msgs::MarkerArray :init))
	 mks)
    (mapcar #'(lambda(x) (push (sphere->marker-msg (make-sphere 15 :pos x) header :color col :alpha 1.0) mks)) pl)
    (mapcar #'(lambda(x) (send x :lifetime (ros::time lf))
		(send x :ns ns)) mks)
    (dotimes (x (length mks)) (send (elt mks x) :id x))
    (send msg :markers mks)
    (ros::publish "tablet_marker_array" msg)
    ))

(defun grasp-gripper (arm);;:gain 0.01
  ;; (send *ri* :move-gripper arm range :effort (* 2000 g) :wait t))
  (send *ri* :start-grasp arm))
(defun close-gripper (arm)
  (send *ri* :start-grasp arm))
(defun open-gripper (arm)
  (send *ri* :stop-grasp arm :wait t))

(defun reset-all ()
  (ros::ros-error "not reseting all")
  (stop-visualize)
  )

(defun reset-all-bak ()
  (setq *node-debug* nil *visualize* nil *chosen-ret* nil)
  (setq *line* (make-list 2) *door* (make-list 3) *drawer* (make-list 3))
  (setq *opendoor* nil *slidedoor* nil)
  (setq *time-count* (ros::time-now))
  (stop-visualize)
  )

(defun move-arm ;;TODO::choose openrave or eus
  (arm coords &rest args &key (move-robot t) (wait-interpolation t) (rotation-axis :z) (debug nil)
       (sec 2000) (use-torso 0.005) (record-ik nil) &allow-other-keys)
  (unless
      (send* *pr2* :inverse-kinematics coords :move-arm arm :rotation-axis rotation-axis
             :use-torso use-torso
             :revert-if-fail nil :debug-view debug args)
    (when record-ik
      (setq *ik-fail-av*  (send *pr2* :angle-vector))
      (setq *ik-fail-coords*  (send coords :copy-worldcoords))
      (dump-loadable-structure (format nil "ik-fail-~d.l" (send (ros::time-now) :to-sec))
                               *ik-fail-coords* *ik-fail-av*))
    (return-from move-arm nil))
  (when move-robot
    (send *ri* :angle-vector (send *pr2* :angle-vector) sec)
    (when wait-interpolation (send *ri* :wait-interpolation)))
  t)

(defun RotateGripper (arm num)
  (let (current-angle)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq current-angle (send *pr2* arm :angle-vector))
    (ros::ros-info "RotateGripper: current-angle = ~A" (elt current-angle 6))
    (setf (aref current-angle 6) (+ (aref current-angle 6) num))
    (send *pr2* arm :angle-vector current-angle)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (setq *current-angle6* (aref current-angle 6)) ;;
    (setq *current-num6* num) ;;
    ))

(defun hand-over2 (arm &key (wait-shock nil))
  (ros::spin-once)
  (let* ((av (send *ri* :state :potentio-vector))
	(tuckarm (check-tuckarm-pose))
	(isfreearm (eq arm tuckarm))
	;; this is for :larm
	(avs (list #f(12 0 64 70 -122 50 -115 160 -4 74 -105 -90 70 -5 20 2 15)
		   #f(12 6 9 106 -77 35 -124 -128 -4 75 -104 -89 70 0 20 3 30)
		   #f(12 13 21 62 -105 -117 -66 -71 -4 74 -104 -89 70 -5 20 4 40)
		   #f(12 9 24 50 -94 -158 -70 39 -4 74 -104 -89 70 -5 20 5 30)))
	(tms (make-list (length avs) :initial-element 1000))
	(l-r-reverse #f(1  -1 1 -1 1 -1 1 -1  -1 1 -1 1 -1 1 -1  -1 1)))
    ;;
    (if (eq arm :rarm)
	(setq avs
	      (mapcar #'(lambda(av)
			  (map float-vector #'*
			       (concatenate float-vector
					    (subseq av 0 1) (subseq av 8 15)
					    (subseq av 1 8) (subseq av 15 17))
			       l-r-reverse))
			  avs)))
    ;;
    (unless isfreearm
      (pr2-reset-pose)
      (setq avs (subseq avs (- (length avs) 2))
	    tms (subseq tms (- (length avs) 2))
	    av (send *ri* :state :potentio-vector))
      (setq tuckarm arm))
    (send *ri* :angle-vector-sequence avs tms)
    (send *ri* :wait-interpolation)
    ))

(provide :jsk-smart-gui-utils)