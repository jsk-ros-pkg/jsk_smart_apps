(load "package://jsk_smart_gui/src/utils.l")
(load "package://jsk_smart_gui/src/get_spot.l")
(load "package://jsk_smart_gui/src/get_template.l")
(load "package://jsk_smart_gui/src/extra_actions.l")
(load "package://jsk_smart_gui/src/greetings.l")
(load "package://jsk_smart_gui/src/moveit.l")
(load "package://jsk_smart_gui/src/eusimage_geometry.l")

(require "package://euslib/irteus_proposals/string.l");;

(defun subscribe-tablet ()
  (ros::subscribe "/Tablet/Command"
                  jsk_gui_msgs::Tablet #'tablet-command-cb)

  (ros::subscribe "/Tablet/Task"
                  jsk_gui_msgs::Tablet #'tablet-task-cb)

  (ros::subscribe "/Tablet/GetSpot"
                  std_msgs::Empty #'tablet-getspot-cb)

  (ros::subscribe "/Tablet/MoveToSpot"
                  roseus::StringStamped #'tablet-movetospot-cb)

  (ros::subscribe "/Tablet/StartDemo"
                  roseus::StringStamped #'tablet-startdemo-cb)

  (ros::subscribe "/android/cmd_vel"
                  geometry_msgs::Twist #'cmd-vel-cb)

  ;; warning tablet-select-cb is also defined in template_grasp_samples.l
  (ros::subscribe "/Tablet/Select" roseus::StringStamped #'tablet-select-cb)

  ;; (ros::subscribe "/Tablet/CommandDebug"
  ;;   jsk_gui_msgs::Tablet #'tablet-command-debug-cb)

  (ros::subscribe "/Tablet/MoveIt" jsk_gui_msgs::Action #'moveit-cb)

  (ros::subscribe "/tablet/marked/camera_info" sensor_msgs::CameraInfo #'caminfo-cb)
  )


(defun tablet-select-cb (msg)
  (if (is-old-msg msg) (return-from tablet-select-cb nil))
  (ros::ros-info "tablet-select-cb called")
  (let ((taskname (read-from-string (send msg :data))))
    (case taskname
      ('SwitchJoy (if *use-joy*
                      (progn (ros::ros-warn "set joy nil")
                             (setq *use-joy* nil))
                    (progn (ros::ros-warn "set joy t")
                           (setq *use-joy* t))))
      )))

(defun cmd-vel-cb (msg)
  (if *use-joy*
      (progn
        (ros::ros-warn "input_vel changing")
        (let ((nmsg (instance geometry_msgs::Twist :init))
              (+linear-factor+ 2.0) (+angular-factor+ 2.0))
          (send nmsg :linear :x (/ (send msg :linear :x) +linear-factor+))
          (send nmsg :linear :y (/ (send msg :linear :y) +linear-factor+))
          (send nmsg :linear :z (/ (send msg :linear :z) +linear-factor+))
          (send nmsg :angular :x (/ (send msg :angular :x) +angular-factor+))
          (send nmsg :angular :y (/ (send msg :angular :y) +angular-factor+))
          (send nmsg :angular :z (/ (send msg :angular :z) +angular-factor+))
          (ros::publish "input_vel" nmsg)))
    (progn
      (ros::ros-info "input_vel not changing")))
  )

(defun tablet-movetospot-cb(msg)
  (if (is-old-msg msg) (return-from tablet-movetospot-cb nil))
  (ros::ros-info "tablet-movetospot-cb called")
  (let ((spot (send msg :data)))
    (move-to-spot spot)))

(defun tablet-startdemo-cb(msg)
  (if (is-old-msg msg) (return-from tablet-startdemo-cb nil))
  (ros::ros-info "tablet-startdemo-cb called")
  (let ((data (send msg :data)))
    (cond ((equal data "/Tablet/chen/GoToElevator")
	   (ros::ros-info "GoToElevator called")
	   (Move-to-spot "/eng2/7f/before-elevator"))
	  ((equal data "/Tablet/chen/Greeting1")
	   (ros::ros-info "Greeting1 called")
	   (greeting1))
	  ((equal data "/Tablet/chen/Greeting2")
	   (ros::ros-info "Greeting2 called")
	   (greeting2))
	  ((equal data "/Tablet/chen/Greeting3")
	   (ros::ros-info "Greeting3 called")
	   (greeting3))
	  ((equal data "/Tablet/chen/Greeting4")
	   (ros::ros-info "Greeting4 called")
	   (greeting4))
	  ((equal data "/Tablet/chen/Greeting5")
	   (ros::ros-info "Greeting5 called")
	   (greeting5))
	  ((equal data "/Tablet/chen/HandOver")
	   (ros::ros-info "HandOver called")
	   (hand-over2 :rarm))
	  )))

(defun tablet-getspot-cb(msg)
  (ros::ros-info "tablet-getspot-cb called")
  (make-temp-spots-cb))

(defun tablet-command-cb (msg)
  (if (is-old-msg msg) (return-from tablet-command-cb nil))
  (let* ((touches (send msg :touches))
	 ;; (x (send (car touches) :x)) (y (send (car touches) :y))
	 (taskname (read-from-string (send msg :action :task_name)))
	 (arm-id-raw (send msg :action :arm_id))
	 (arm-id (if (or (zerop arm-id-raw) (equal arm-id-raw jsk_gui_msgs::Action::*RARMID*)) ':rarm ':larm)))
;;    (ros::ros-debug "/Tablet/command: Received screenpoint ~A" (list x y))
    (case taskname
      ('MoveNeck
       (ros::ros-info "MoveNeck called")
       (MoveNeckExecute (send msg :action :direction) (send msg :action :direction_value)))
      ('MoveCameraCenter
       (ros::ros-info "MoveCameraCenter called")
       (MoveCameraCenterExecute (send msg :action :touch_x) (send msg :action :touch_y)))
      ('Show3DScreenPoint
       (ros::ros-info "Show3DScreenPoint called")
       (Show3DScreenPointExecute (send msg :action :touch_x) (send msg :action :touch_y) *current-camera*))
      ('GetTemplate
       (ros::ros-info "GetTemplate called")
       (GetTemplateExecute (send msg :action :touch_x) (send msg :action :touch_y)))
      ('PushOnce
       (ros::ros-info "PushOnce called")
       (PushOnceExecute (send msg :action :touch_x) (send msg :action :touch_y) arm-id))
      ('SwitchSensor
       (ros::ros-info "SwitchSensor called")
       (SwitchSensorExecute (send msg :action :state)))
      ('ZoomCamera
       (ros::ros-info "ZoomCamera called")
       (ZoomCameraExecute (send msg :action :touch_x) (send msg :action :touch_y) (send msg :action :state_value)))
      ('TorsoUp
       (ros::ros-info "TorsoUp called")
       (send *pr2* :torso :waist-z :joint-angle 325)
       (send *ri* :angle-vector (send *pr2* :angle-vector))
       ;; (send *ri* :wait-interpolation)
       )
      ('TorsoDown
       (ros::ros-info "TorsoDown called")
       (send *pr2* :torso :waist-z :joint-angle 50)
       (send *ri* :angle-vector (send *pr2* :angle-vector))
       ;; (send *ri* :wait-interpolation)
       )
      ('StopMotion
       (ros::ros-info "StopMotion called")
       (ros::spin-once)
       (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
       (unix::usleep 100)
       (send *ri* :angle-vector (send *pr2* :angle-vector))
       (send *ri* :wait-interpolation))
      ('TuckArmPose
       (ros::ros-info "TuckArmPose called ~A" arm-id)
       (pr2-tuckarm-pose arm-id))
      ('OpenDoor
       (ros::ros-info "OpenDoor called")
       (if (null *chosen-ret*) (return-from tablet-command-cb nil))
       (print (list *chosen-ret* *range* ))
       (OpenDoorExecute arm-id *chosen-ret*))
      ('RotateGripper
       (ros::ros-info "RotateGripper called")
       (RotateGripper arm-id (send msg :action :state_value)))
      ('OpenGripper
       (ros::ros-info "OpenGripper called")
       (open-gripper arm-id))
      ('CloseGripper
       (ros::ros-info "CloseGripper called")
       (grasp-gripper arm-id))
      ('ResetAll
       (ros::ros-info "ResetAll called")
       (reset-all))
      ;;  ('give
      ;;  (ros::ros-info "give-cb")
      ;;  (hand-over hand))
      )))

;; task concering open or pull
(defun tablet-task-cb (msg)
  (if (or (is-old-msg msg) (check-not-ray-srv)) (return-from tablet-task-cb nil))
  (let* ((touches (send msg :touches))
	 (taskname (read-from-string (send msg :action :task_name)))
	 (arm-id-raw (send msg :action :arm_id))
	 (arm-id (if (or (zerop arm-id-raw) (equal arm-id-raw jsk_gui_msgs::Action::*RARMID*)) ':rarm ':larm))
	 tx ty c)
    (cond ((eq (length touches) 1)
	   (setq tx (send (car touches) :x) ty (send (car touches) :y))
	   (ros::ros-info "/Tablet/task: Sreenpoint = ~A" (list tx ty))
	   (if (null (setq c (screenpoint-srvcall tx ty)))
	       (return-from tablet-task-cb nil))
	   (ros::ros-info "/Tablet/task: Actual point = ~A" c))
	  (t
	   (ros::ros-info "/Tablet/task: touches >= 2")))
    (case taskname
      ('OpenDoorInput
       (ros::ros-error "OpenDoorInput called")
       ;; assumed that touches = 3
       ;; (OpenDoorInputExecute arm-id touches)
       (OpenDoorInputExecute arm-id touches :use-openrave nil :grab t);;!!
       )
      ('MovingPointInfo
       (ros::ros-info "MovingPointInfoExecute called")
       ;; assumed that touches = 3
       (MovingPointInfoExecute touches))
      ('DrawLine
       (ros::ros-info "DrawLine called")
       ;; assumed that touches = 2
       (DrawLineExecute touches))
      ('DrawCircle
       (ros::ros-info "DrawCircle called")
       ;; assumed that touches = 3
       (DrawCircleExecute arm-id touches))
      ('CircularManipulation
       (ros::ros-info "CircularManipulation called")
       ;; assumed that touches = 3
       (CircularManipulationExecute arm-id touches))
      ('CancelManipulation
       (ros::ros-info "CancelManipulation called")
       (stop-visualize-line)
       (stop-visualize-circle)
       (stop-visualize-text))

      ;; ('pick
      ;;  (ros::ros-info "pick-cb")
      ;;  (pick-cb hand c))
      ;; ('slide2
      ;;  (ros::ros-info "slide-cb2")
      ;;  (slide-cb2 hand (send c :pos)))
      )))

;;       ('execute-slide2
;;        (ros::ros-info "execute-slide2")
;;        (if (null *chosen-ret*) (return-from execute-cb nil))
;;        (print (list *chosen-ret* *range* ))
;;        (execute-slide2 hand (send (elt *ret* *chosen-ret*) :worldpos))))

(defun DrawLineExecute (touches)
  (unless (eq (length touches) 2)
    (ros::ros-warn "DrawLineExecute: ~A touches, returning" (length touches))
    (return-from DrawLineExecute nil))
  (dotimes (i 2) (setf (elt *line* i) (list (send (elt touches i) :x) (send (elt touches i) :y))))
  (ros::ros-info "DrawLineExecute: *line* = ~A" *line*)
  (let ((msg1 (instance geometry_msgs::PointStamped :init))
	(msg2 (instance geometry_msgs::PointStamped :init)) p0 p1)
    (visualize-line (elt *line* 0) (elt *line* 1)) ;; image_view2::ImageMarker2
    (send msg1 :point :x (elt (elt *line* 0) 0))
    (send msg1 :point :y (elt (elt *line* 0) 1))
    (send msg2 :point :x (elt (elt *line* 1) 0))
    (send msg2 :point :y (elt (elt *line* 1) 1))
    (setq p0 (screenpoint-srvcall (send msg1 :point :x) (send msg1 :point :y)))
    (setq p1 (screenpoint-srvcall (send msg2 :point :x) (send msg2 :point :y)))
    ;;(setq p0 (point-srvcall msg1) p1 (point-srvcall msg2))
    (if (or (null p0) (null p1)) (return-from DrawLineExecute nil))
    (ros::ros-info "DrawLineExecute: screenpoint-srvcall succeed")
    ;; visualization_msgs::MarkerArray
    (show-line (setf (elt *door* 0) (send p0 :worldpos))
               (setf (elt *door* 1) (send p1 :worldpos)))
    (setf (elt *drawer* 0) (send p0 :worldpos))
    (setf (elt *drawer* 1) (send p1 :worldpos))
    ))

(defun MovingPointInfoExecute (touches)
  ;; (if (and (and (null (elt *door* 0)) (null (elt *door* 1)))
  ;; 	   (< (length *opendoor*) 4)) (return-from MovingPointInfoExecute nil))
  (if (and (null (elt *door* 0)) (null (elt *door* 1)))
      (return-from MovingPointInfoExecute nil))
  (unless (boundp '*fp*) (return-from MovingPointInfoExecute nil))
  (unless (eq (length touches) 1)
    (ros::ros-warn "MovingPointInfoExecute: ~A touches, returning" (length touches))
    (return-from MovingPointInfoExecute nil))
  (let* ((tx (send (car touches) :x)) (ty (send (car touches) :y))
	 ;;(c (screenpoint-srvcall tx ty))
	 (fi (list tx ty)))
    ;; (if (null c) (return-from MovingPointInfoExecute nil))
    ;;    (print c)
    (print fi)
    (visualize-line fi (point2pixel :px (aref *fp* 0) :py (aref *fp* 1) :pz (aref *fp* 2)))
    (setq *finger1* fi)
    (setq *chosen-ret* (position
			(apply #'min (mapcar #'(lambda (x) (distance (coerce *finger1* float-vector) (coerce x float-vector))) *ret2d*))
			(mapcar #'(lambda (x) (distance (coerce *finger1* float-vector) (coerce x float-vector))) *ret2d*)))
    (print (list *chosen-ret* *finger1* *ret2d*))
    ))

(defun MoveCameraCenterExecute (x y);;todo
  (let (c)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq c (make-camera-from-ros-camera-info-aux 640 480 #f(892.302 0.0 422.326 0.0 0.0 892.302 268.051 0.0 0.0 0.0 1.0 0.0) (make-cascoords :pos (send *pr2* :wide_stereo-left :worldpos))))
    (send *pr2* :head :look-at (send (send (send *pr2* :wide_stereo-left :copy-worldcoords) :translate (scale 30000 (send c :ray x y))) :worldpos))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)))

(defun Show3DScreenPointExecute (x y cam)
  (let* ((c (screenpoint-srvcall x y))
         (cam-name-orig (string-left-trim "/" cam))
         (cam-name (substitute #\- #\/ (string-downcase cam-name-orig)))
         ;; ex) from /wide_stereo/left to :wide_stereo-left
         (cam-method-name (intern (string-upcase cam-name) *keyword-package*)))
    (ros::ros-info "cam-method-name : ~A" cam-method-name)
    ;; (setq cam-method-name :openni-rgb)
    ;; (ros::ros-info "re cam-method-name : ~A" cam-method-name)
    (if (null c) (return-from Show3DScreenPointExecute nil))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (visualize-point x y :sca 10 :r 1.0 :lifetime 10)
    (show-markers (list (send (send c :copy-worldcoords) :pos)) :lifetime 10)
    (if (find-method *pr2* cam-method-name)
        (show-line (send (send c :copy-worldcoords) :pos)
                   (send (funcall 'send *pr2* cam-method-name :copy-worldcoords) :pos) :lifetime 10)
      (ros::ros-warn "Assume that you are using external camera"))
    (ros::ros-info "Show3DScreenPointExecute: showing for 10 seconds")
    ))

;; moved to get_template.l
;; (defun GetTemplateExecute (x y)
;;   (let* ((c (screenpoint-srvcall x y))
;;          )
;;     ))

;; Example: (setq str "((:image /r_forearm_cam/image_mono) (:points /camera/rgb/points) (:camera_info /r_forearm_cam/camera_info))")
(defun SwitchSensorExecute (switch-str)
  (let* ((switch-lst (read-from-string switch-str))
         (image (assoc :image switch-lst)) (points (assoc :points switch-lst))
         (camera_info (assoc :camera_info switch-lst)))
    (if (or (null image) (null points) (null camera_info))
        (progn (ros::ros-warn "SwitchSensorExecute: nor image/camera_info or points") (return-from SwitchSensorExecute nil)))
    (print image) (print camera_info) (print points)
    (if (eq (elt image 1) 'cameras) (setq image nil))
    (if (eq (elt points 1) 'points) (setq points nil))
    (if (eq (elt camera_info 1) 'camera_info) (setq camera_info nil));;nonsense
    (let ((mux-req1 (instance topic_tools::MuxSelectRequest :init))
          (mux-req2 (instance topic_tools::MuxSelectRequest :init))
          (mux-req3 (instance topic_tools::MuxSelectRequest :init)))
      (when (and camera_info (ros::service-exists "/camera_info_mux/select"))
        ;;for image_view2 and depth_image_creator
        (send mux-req1 :topic (format nil "~A" (elt camera_info 1)))
        (ros::service-call "/camera_info_mux/select" mux-req1))
      (when (and image (ros::service-exists "/image_mux/select"))
        (setq *current-image* (string-downcase (elt image 1)))
        ;; ex) from "/wide_stereo/left/image_color" to /wide_stereo/left
        ;; ex) from "/r_forearm_cam/image_mono" to /r_forearm_cam
        (cond ((= 2 (count #\/ *current-image*))
               (setq *current-camera* (read-from-string (string-right-trim "/" (string-right-trim (car (last (split-string *current-image* #\/))) *current-image*)))))
              ((= 3 (count #\/ *current-image*))
               (setq *current-camera* (read-from-string (string-right-trim (concatenate string "/" (car (last (split-string *current-image* #\/)))) *current-image*))))
              (t
               (ros::ros-error "*current-image*:~A did not have two or three slash(/), *current-camera* not created" *current-image*)
               ))
        ;;for image_view2
        (send mux-req2 :topic (format nil "~A" (elt image 1)))
        (ros::service-call "/image_mux/select" mux-req2))
      (when (and points (ros::service-exists "/points_mux/select"))
        ;;for depth_image_creator
        (send mux-req3 :topic (format nil "~A" (elt points 1)))
        (ros::service-call "/points_mux/select" mux-req3))
      (ros::subscribe "/tablet/marked/camera_info" sensor_msgs::CameraInfo #'caminfo-cb)
      )))

(defun MoveNeckExecute (direction value)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (case (read-from-string direction)
    ('up
     (send *pr2* :head :neck-p :joint-angle (*  0.1 value) :relative t))
    ('down
     (send *pr2* :head :neck-p :joint-angle (* -0.1 value) :relative t))
    ('left
     (send *pr2* :head :neck-y :joint-angle (* -0.1 value) :relative t))
    ('right
     (send *pr2* :head :neck-y :joint-angle (*  0.1 value) :relative t))
    )
  (send *ri* :angle-vector (send *pr2* :angle-vector) 500))

(defun ZoomCameraExecute (mx my value);;
  (let* ((req (instance dynamic_reconfigure::ReconfigureRequest :init))
	 (width (floor (* pwidth (1+ (* value pmaxlength)))))
	 (height (floor (* pheight (1+ (* value pmaxlength)))))
	 (old_pwidth pwidth)(old_pheight pheight))
    (cond ((or (< width 640) (< height 480))
	   (setq pwidth 640 pheight 480)
	   (setq pbinning_x 1 pbinning_y 1))
	  ((or (> width 2432) (> height 1824))
	   (setq pwidth 2431 pheight 1823)
	   (setq pbinning_x 3 pbinning_y 3)
	   )
	  (t (setq pwidth width pheight height)
	     (setq pbinning_x 2 pbinning_y 2)
	     ))
    (ros::ros-info "ZoomCamera: width:~A, height:~A" pwidth pheight)
    (let ((x_offset (floor (+ px_offset (* mx (/ old_pwidth 640)) (/ pwidth -2))))
	  (y_offset (floor (+ py_offset (* my (/ old_pheight 640)) (/ pheight -2)))))
      (cond ((or (< x_offset 0) (< y_offset 0))
	     (setq px_offset 0 py_offset 0))
	    ((or (> x_offset 1792) (> y_offset 1344))
	     (setq px_offset 1792 py_offset 1344))
	    (t (setq px_offset x_offset py_offset y_offset)))
      (ros::ros-info "ZoomCamera: x_offset:~A, y_offset:~A" px_offset py_offset)
      (ros::ros-info "ZoomCamera: binning_x:~A, binning_y:~A" pbinning_x pbinning_y)
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
      (unix::usleep (* 500 1000))
      )))

(defun caminfo-cb(msg)
  (ros::ros-info "Get CameraInfo: ~A" (send msg :header :frame_id))
  (send *pcm* :from-camera-info msg)
  (ros::unsubscribe "/tablet/marked/camera_info")
  )

(defun DrawCircleExecute (arm-id touches)
  (unless (eq (length touches) 3)
    (ros::ros-warn "DrawCircleExecute: ~A touches, returning" (length touches))
    (return-from DrawCircleExecute nil))
  (let ((axis-line nil) (touch3rd (find-if #'(lambda(x) (= (send x :finger_id) 2)) touches)) touchv handpos footp footv x-axis y-axis z-axis circleply camcds intersectpoint cangle asign
        (ocolor (float-vector 0.0 1.0 0.0))
        fcolor pose)
    (setq touchv (send *pcm* :project-pixel-to-3dray (float-vector (send touch3rd :x) (send touch3rd :y))))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (if (<= (elt (elt *drawer* 0) 2) (elt (elt *drawer* 1) 2))
        (setq axis-line (make-line (elt *drawer* 0) (elt *drawer* 1)))
      (setq axis-line (make-line (elt *drawer* 1) (elt *drawer* 0)))
      )
    (setq handpos (send (send *pr2* arm-id :end-coords :copy-worldcoords) :worldpos))
    (setq footp (send axis-line :foot handpos))
    (setq footv (send axis-line :point footp))
    (setq z-axis (send axis-line :direction))
    (setq x-axis (normalize-vector (v- handpos footv)))
    (setq y-axis (v* z-axis x-axis))
    (setq *circlecoords* (make-coords :pos footv :rot (transpose (matrix x-axis y-axis z-axis))))
    (setq *circlediameter* (send axis-line :distance handpos))
    (setq circleply (instance polygon :init :vertices (list footv handpos (v+ footv (scale *circlediameter* y-axis)))))
    (setq camcds (send *pr2* (intern (string-upcase (string-left-trim "/" (send *pcm* :tf-frame))) *keyword-package*) :copy-worldcoords))
    (setq intersectpoint (cadr (send circleply :intersect-point-vector (send camcds :worldpos) (normalize-vector (transform (send camcds :worldrot) touchv)))))
    (if (< (elt (v* x-axis (v- intersectpoint footv)) 2) 0)
        (progn
          (setq asign -1)
          (setq fcolor (float-vector 1.0 0.0 0.0)))
      (progn
        (setq asign 1)
        (setq fcolor (float-vector 0.0 0.0 1.0))))
    (setq cangle (rad2deg (acos (v. x-axis (normalize-vector (v- intersectpoint footv))))))
    (setq *circleangle* (* asign cangle))
    (visualize-circle *circlecoords* *circlediameter* :angle *circleangle* :width 5 :outline-color ocolor :fill-color fcolor)
    (visualize-text (format nil "~A degree" *circleangle*) (send *circlecoords* :worldpos) :scale 0.5 :color fcolor)
    )
  )
