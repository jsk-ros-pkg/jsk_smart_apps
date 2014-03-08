;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; extra actions: push pick open pick
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun push-cb (hand c);;maybe should check free
  (show-markers (list (send c :worldpos)))
  ;;  (move-arm hand (send (send c :copy-worldcoords) :translate #f(-200 0 0)) :sec 3000)
  (move-arm hand (send (send c :copy-worldcoords) :translate #f(-100 0 0)) :rotation-axis t :rthre (deg2rad 55) :sec 3000)
  ;;  (move-arm hand (send (send c :copy-worldcoords) :translate #f(10 0 0)) :sec 1000)
  (move-arm hand (send (send c :copy-worldcoords) :translate #f(10 0 0)) :rotation-axis t :rthre (deg2rad 35) :sec 1000)
  )

(defun pick-cb (hand c)
  (ros::spin-once)
  (let* ((av (send *ri* :state :potentio-vector))
	 (tuckarm (check-tuckarm-pose))
	 (isfreearm (eq hand tuckarm)) (di pi/2) ;;(di pi/2) ;; 0 for tate rimokon
	 tmp issuccess ravs rtms)
    (show-markers (list (send c :worldpos)))
    (setq tmp (copy-object (send *pr2* :angle-vector)))
    (unless isfreearm
      (pr2-reset-pose)
      (setq tmp (copy-object (send *pr2* :angle-vector))))
    ;;    (setq issuccess (send *ri* :move-end-coords-plan (send (send (send c :copy-worldcoords) :translate #f(-100 0 0) :local) :rotate 0 :x) :move-target hand))
    (setq issuccess (send *ri* :move-end-coords-plan 
			  (send (make-coords :pos (send (send c :copy-worldcoords) :worldpos) :rpy (float-vector di pi/2 0)) :translate #f(0 0 200) :world)
			  :move-target hand))
    (send *ri* :wait-interpolation)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (print 'finished-openrave)
    (cond (issuccess
	   (print 'solved)
	   (send *ri* :move-gripper hand 0.09 :effort 30 :wait t)
	   (send *ri* :wait-interpolation)
	   (move-arm hand (send (make-coords :pos (send (send c :copy-worldcoords) :worldpos) :rpy (float-vector di pi/2 0)) :translate #f(0 0 -30) :world) :rotation-axis t :rthre (deg2rad 15) :sec 1000)
	   (send *ri* :move-gripper hand 0.005 :effort 30 :wait t)
	   (send *ri* :wait-interpolation)
	   (move-arm hand (send (make-coords :pos (send (send c :copy-worldcoords) :worldpos) :rpy (float-vector di pi/2 0)) :translate #f(0 0 100) :world) :rotation-axis t :rthre (deg2rad 15) :sec 1000)
	   (setq ravs (copy-object (elt issuccess 0)) rtms (copy-object (elt issuccess 1)))
	   (send *ri* :angle-vector-sequence (append (cdr (reverse ravs))(list tmp)) (reverse rtms))
	   (send *ri* :wait-interpolation)
	   )
	  (t
	   (print 'unsolved)
	   ;;(send *pr2* :torso :waist-z :joint-angle 300)
	   (send *pr2* :reset-manip-pose)
	   (send *ri* :angle-vector (send *pr2* :angle-vector))
	   (send *ri* :wait-interpolation)

	   (move-arm hand (send (make-coords :pos #f(560 0 1220) :rpy (float-vector di pi/2 0)) :translate #f(0 0 0) :world) :rotation-axis t :rthre (deg2rad 35) :sec 5000 :revert-if-fail nil)

	   (send *ri* :move-gripper hand 0.09 :effort 30 :wait t)
	   (send *ri* :wait-interpolation)
	   (move-arm hand (send (make-coords :pos (send (send c :copy-worldcoords) :worldpos) :rpy (float-vector di pi/2 0)) :translate #f(0 0 20) :world) :rotation-axis t :rthre (deg2rad 35) :sec 2000)
	   (move-arm hand (send (makep-coords :pos (send (send c :copy-worldcoords) :worldpos) :rpy (float-vector di pi/2 0)) :translate #f(0 0 -50) :world) :rotation-axis t :rthre (deg2rad 15) :sec 1000)
	   (send *ri* :move-gripper hand 0.005 :effort 30 :wait t)
	   (send *ri* :wait-interpolation)
	   (move-arm hand (send (make-coords :pos (send (send c :copy-worldcoords) :worldpos) :rpy (float-vector di pi/2 0)) :translate #f(0 0 100) :world) :rotation-axis t :rthre (deg2rad 15) :sec 1000)
	   (pr2-tuckarm-pose hand)

	   (send *pr2* :torso :waist-z :joint-angle 100)
	   (send *ri* :angle-vector (send *pr2* :angle-vector))
	   (send *ri* :wait-interpolation)
	   ))
    (unix::sleep 2)
    (pr2-tuckarm-pose hand)
    ))

;; (defun give-cb (hand)
;;   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
;;   (send *pr2* :head :neck-p :joint-angle 20)
;;   (send *pr2* :head :neck-y :joint-angle 0)
;;   (send *ri* :angle-vector (send *pr2* :angle-vector))

;;   (move-arm hand (make-coords :pos #f(800 0 800) :rpy (float-vector 0 0 pi/2)))
;;   (send *ri* :move-gripper hand 0.09 :effort 30 :wait t)
;;   (send *ri* :wait-interpolation)
;;   (pr2-tuckarm-pose hand)
;;   )

(setq *circle-div* 40)
(defun OpenDoorInputExecute (arm touches &key (grab t) (use-openrave t))
  (if (or (null (elt *door* 0)) (null (elt *door* 1))
          (return-from OpenDoorInputExecute nil)))
  (let* ((a (elt *door* 0))(b (elt *door* 1))
         (c (screenpoint-srvcall (send (elt touches 2) :x) (send (elt touches 2) :y)))
         (di pi/2))
    (setf (elt *door* 2) (send c :pos))
    (if (null c) (return-from OpenDoorInputExecute nil))
    (ros::ros-info "OpenDoorInputExecute: Actual point = ~A" c)
    (setq *ret* (generate-circle-path (send c :pos) a b (* *default-deg* 2) :div *circle-div*))
    (ros::ros-info "OpenDoorInputExecute: Actual point end")
    (setq *range* (make-list (length *ret*)));;
    (dotimes (i (length *ret*)) (setf (elt *range* i) i));;
    (setq *visualize* t)
    (show-markers (mapcar #'(lambda(x)(copy-object (send x :worldpos))) *ret*))
    ;;From now, We assume that PR2 head will not move until this function ends
    (mapcar #'(lambda(x y) (send *tfb* :send-transform x "base_footprint" (format nil "/ret~A" y)) (format nil "/ret~A" y)) *ret* *range*)
    (mapcar #'(lambda(x y)
                (setq tpos (send x :worldpos))
                (setq tbcolor (+ (* (/ y (* (length *range*) 1.0)) 0.5) 0.5))
                (setq tacolor (* (/ y (* (length *range*) 1.0)) 1.0))
                (apply #'visualize-point
                       (flatten (point2pixel :px (aref tpos 0) :py (aref tpos 1) :pz (aref tpos 2)) (list :b tbcolor :a tacolor :i y :lifetime 0))))
            *ret* *range*)
    (setq *ret2d* (mapcar #'(lambda(x) (point2pixel :px (aref (send x :worldpos) 0) :py (aref (send x :worldpos) 1) :pz (aref (send x :worldpos) 2))) *ret*))

    ;;TODO grab shuold be separated
    (send *ri* :move-gripper arm 0.09 :wait nil) ;; open
    (setq *current-arm* arm)
    (when grab
      (if use-openrave
          (progn
            (send *ri* :move-end-coords-plan (send (send (send (elt *ret* 0) :copy-worldcoords) :translate #f(-250 0 0) :local) :rotate di :x) :move-arm arm)
            (send *ri* :wait-interpolation)
            (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
            (print 'finished-openrave)
            ))
      (cond
       ((move-arm arm (send (send (send (car *ret*) :copy-worldcoords)
                                  :translate #f(-150 0 0) :local) :rotate di :x)
                  :rotation-axis t :sec 1000) t)
       (t
        (ros::ros-warn "Could not solve ik, return from execute-open")
        (return-from OpenDoorInputExecute nil)))
      (move-arm arm (send (send (send (car *ret*) :copy-worldcoords)
                                :translate #f(-35 0 0) :local) :rotate di :x)
                :rotation-axis t :sec 1000)
      (grasp-gripper *current-arm*)
      )
    ))

;; handle is position
(defun OpenDoorInputEusExecute (arm handle rad &key (grab nil))
  (if (or (null (elt *door* 0)) (null (elt *door* 1))
	  (return-from OpenDoorInputEusExecute nil)))
  (print 'ok)
  (let* ((a (elt *door* 0))(b (elt *door* 1))
	 (di pi/2))
    (setf (elt *door* 2) handle)
    (setq *ret* (generate-circle-path handle a b rad :div (/ *circle-div* 3)))
    (setq *range* (make-list (length *ret*)));;
    (dotimes (i (length *ret*)) (setf (elt *range* i) i));;
    (setq *visualize* t)
    (show-markers (mapcar #'(lambda(x)(copy-object (send x :worldpos))) *ret*))
    ;;From now, We assume that PR2 head will not move until this function ends
    (mapcar #'(lambda(x y) (send *tfb* :send-transform x "base_footprint" (format nil "/ret~A" y)) (format nil "/ret~A" y)) *ret* *range*)
    (mapcar #'(lambda(x y)
                (setq tpos (send x :worldpos))
                (setq tbcolor (+ (* (/ y (* (length *range*) 1.0)) 0.5) 0.5))
                (setq tacolor (* (/ y (* (length *range*) 1.0)) 1.0))
                (apply #'visualize-point
                       (flatten (point2pixel :px (aref tpos 0) :py (aref tpos 1) :pz (aref tpos 2)) (list :b tbcolor :a tacolor :i y :lifetime 0)))
                )
            *ret* *range*)
    (setq *ret2d* (mapcar #'(lambda(x) (point2pixel :px (aref (send x :worldpos) 0) :py (aref (send x :worldpos) 1) :pz (aref (send x :worldpos) 2))) *ret*))
    (unix::sleep 5)
    (stop-visualize)
    (open-gripper *current-arm*)
    (when grab
      (cond
       ((move-arm arm (send (send (send (car *ret*) :copy-worldcoords)
                                  :translate #f(-150 0 0) :local) :rotate di :x)
                  :rotation-axis t :sec 1000) t)
       (t
        (ros::ros-warn "Could not solve ik, return from execute-open")
        (return-from OpenDoorInputEusExecute nil)))
      (move-arm arm (send (send (send (car *ret*) :copy-worldcoords)
                                :translate #f(0 0 0) :local) :rotate di :x)
                :rotation-axis t :sec 1000)
      (grasp-gripper *current-arm*)
    )))

(defun OpenDoorExecute (hand chosen-ret)
  (let ((a (elt *door* 0)) (b (elt *door* 1)) (c (elt *door* 2))
         (di pi/2) ret avs tms uavs utms rad current-angle
         (ra :z) (use-torso 0.0025))
    (print (list hand chosen-ret))
    (if (< chosen-ret (/ *circle-div* 2))
        (progn
          (setq rad (* (/ chosen-ret (* (length *range*) 1.0)) (* *default-deg*) 2))
          (setq ret (generate-circle-path c a b rad)))
      (progn
        (setq rad (* (/ (- *circle-div* chosen-ret) (* (length *range*) 1.0)) (* *default-deg*) 2))
        (setq ret (generate-circle-path c b a rad))))
    (ros::ros-warn "Evaluated degree: ~A [deg]" (rad2deg rad))
    (show-markers (mapcar #'(lambda(x) (copy-object (send x :worldpos))) ret))
    (stop-visualize)
    
    (dolist (r (cdr ret))
      (setq rr (send r :copy-worldcoords))

      (ros::ros-warn "rot 2 2: ~A" (elt (matrix-column (send (send r :copy-worldcoords) :rot) 2) 2) 0.0)
      (if (< (elt (matrix-column (send (send r :copy-worldcoords) :rot) 2) 2) 0.0)
          (sedtq ra :zm))
      (let ((ik-solved
             (send *pr2* :inverse-kinematics
                   (send (send rr :copy-worldcoords) :rotate di :x)
                   :use-torso use-torso
                   :move-arm hand :rotation-axis ra :debug-view nil :warnp nil)))
        (unless (or ik-solved (eq ra nil))
          (warn ";; retry ~A~%" rr);
          (setq ik-solved
                (send *pr2* :inverse-kinematics
                      (send (send rr :copy-worldcoords) :rotate di :x)
                      :use-torso use-torso
                      :move-arm hand :rotation-axis nil :debug-view nil :warnp nil)))
        (unless ik-solved (warn ";; failed ~A~%" rr))
        (when ik-solved
          (push (send *pr2* :angle-vector) avs)
          (push 300 tms))
        ))
    (send *ri* :angle-vector-sequence (reverse avs) tms)
    (setq *close* (copy-object avs));;
    (send *ri* :wait-interpolation)
    ;; (open-gripper *current-arm*)
    ;; (pr2-tuckarm-pose hand)
    (setq *door* (make-list 3))
    (setq *visualize* nil *chosen-ret* nil)
    ))

(defun OpenDoorEusExecute (hand rad)
  (let ((a (elt *door* 0)) (b (elt *door* 1)) (c (elt *door* 2))
        ret avs tms uavs utms current-angle
        (di pi/2)  (ra :z) (use-torso 0.0025))
    (setq ret (generate-circle-path c a b rad))
    (print a)
    (print b)
    (print c)
    (print ret)
    (ros::ros-warn "Evaluated degree: ~A [deg]" (rad2deg rad))
    (show-markers (mapcar #'(lambda(x) (copy-object (send x :worldpos))) ret))
    (stop-visualize)

    (dolist (r (cdr ret))
      (setq rr (send r :copy-worldcoords))

      (ros::ros-warn "rot 2 2: ~A" (elt (matrix-column (send (send r :copy-worldcoords) :rot) 2) 2) 0.0)
      (if (< (elt (matrix-column (send (send r :copy-worldcoords) :rot) 2) 2) 0.0)
          (sedtq ra :zm))
      (let ((ik-solved
             (send *pr2* :inverse-kinematics
                   (send (send rr :copy-worldcoords) :rotate di :x)
                   :use-torso use-torso
                   :move-arm hand :rotation-axis ra :debug-view nil :warnp nil)))
        (unless (or ik-solved (eq ra nil))
          (warn ";; retry ~A~%" rr);
          (setq ik-solved
                (send *pr2* :inverse-kinematics
                      (send (send rr :copy-worldcoords) :rotate di :x)
                      :use-torso use-torso
                      :move-arm hand :rotation-axis nil :debug-view nil :warnp nil)))
        (unless ik-solved (warn ";; failed ~A~%" rr))
        (when ik-solved
          (push (send *pr2* :angle-vector) avs)
          (push 300 tms))
        ))
    (send *ri* :angle-vector-sequence (reverse avs) tms)
    (setq *close* (copy-object avs));;
    (send *ri* :wait-interpolation)
    ;; (open-gripper *current-arm*)
    ;; (pr2-tuckarm-pose hand)
    (setq *door* (make-list 3))
    (setq *visualize* nil *chosen-ret* nil)
    ))

(defun CircularManipulationExecute (arm-id touches)
  (let ((dnum (ceiling (/ (abs *circleangle*) 10.0)))
        inithandcds
        (avs nil)
        )
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq inithandcds (send (send *circlecoords* :copy-worldcoords) :transformation (send *pr2* arm-id :end-coords :copy-worldcoords)))
    (dotimes (i dnum)
      (let ((rotmat (make-coords :rot (matrix (float-vector (cos (deg2rad (* (1+ i) (/ *circleangle* dnum)))) (- (sin (deg2rad (* (1+ i) (/ *circleangle* dnum))))) 0) (float-vector (sin (deg2rad (* (1+ i) (/ *circleangle* dnum)))) (cos (deg2rad (* (1+ i) (/ *circleangle* dnum)))) 0) (float-vector 0 0 1)))) av)
        (send rotmat :transform inithandcds)
        (send rotmat :transform *circlecoords* :world)
        (print rotmat)
        (setq av (send *pr2* :inverse-kinematics rotmat :move-arm arm-id :use-torso nil))
        (if av
            (push av avs)
          (progn
            (setq av (send *pr2* :inverse-kinematics rotmat :move-arm arm-id :use-torso t))
            (when av
              (push av avs))))
        )
      )
    (send *ri* :angle-vector-sequence (reverse avs) (make-sequence 'list (length avs) :initial-element 500))
    (send *ri* :wait-interpolation)
    (stop-visualize-line)
    (stop-visualize-circle)
    (stop-visualize-text)
    )
  )
