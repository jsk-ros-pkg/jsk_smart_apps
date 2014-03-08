(load "package://jsk_smart_gui/lib/eusimage_geometry.so")

(defclass pinhole-camera-model
  :super propertied-object
  :slots (cobject camera-info))
(defmethod pinhole-camera-model
  (:init
   (&key (use-tf2))
   (sys::dispose-hook self t)
   (cond
    (use-tf2
     (unless (boundp '*tfl*)
       (defvar *tfl* (instance ros::buffer-client :init))))
    (t
     (unless (boundp '*tfl*)
       (defvar *tfl* (instance ros::transform-listener :init)))))
   (setq cobject (euspinhole-camera-model)))
  (:dispose
   ()
   (euspinhole-camera-model-dispose cobject))
  (:from-camera-info
   (cam-info)
   (setq camera-info cam-info)
   (euspinhole-camera-model-from-camera-info cobject (send cam-info :serialize) (send cam-info :serialization-length)))
  (:project-pixel-to-3dray
   (uv)
   (euspinhole-camera-model-project-pixel-to-3dray cobject uv))
  (:project-3d-to-pixel
   (pos &optional (frame-id (send camera-info :header :frame_id)) (tm (ros::time-now)))
   (let (cam-pos cds)
     (if (string= frame-id (send camera-info :header :frame_id))
         (setq cam-pos pos)
       (progn
         (setq cds (send *tfl* :lookup-transform (send camera-info :header :frame_id) frame-id tm))
         (unless cds
           (ros::ros-error "failed to lookup-transform from ~A to ~A" (send camera-info :header :frame_id) frame-id)
           (return-from :project-3d-to-pixel nil))
         (setq cam-pos (send (send (send cds :transformation (make-coords :pos pos)) :copy-worldcoords) :worldpos))))
     (euspinhole-camera-model-project-3d-to-pixel cobject cam-pos)))
  (:tf-frame
   ()
   (send camera-info :header :frame_id))
  (:camera-info
   ()
   camera-info)
  )