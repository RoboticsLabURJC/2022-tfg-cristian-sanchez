;; Auto-generated. Do not edit!


(when (boundp 'heatmap_util::GetPowerFrissFeedback)
  (if (not (find-package "HEATMAP_UTIL"))
    (make-package "HEATMAP_UTIL"))
  (shadow 'GetPowerFrissFeedback (find-package "HEATMAP_UTIL")))
(unless (find-package "HEATMAP_UTIL::GETPOWERFRISSFEEDBACK")
  (make-package "HEATMAP_UTIL::GETPOWERFRISSFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude GetPowerFrissFeedback.msg.html


(defclass heatmap_util::GetPowerFrissFeedback
  :super ros::object
  :slots (_feedback ))

(defmethod heatmap_util::GetPowerFrissFeedback
  (:init
   (&key
    ((:feedback __feedback) nil)
    )
   (send-super :init)
   (setq _feedback __feedback)
   self)
  (:feedback
   (&optional (__feedback :null))
   (if (not (eq __feedback :null)) (setq _feedback __feedback)) _feedback)
  (:serialization-length
   ()
   (+
    ;; bool _feedback
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _feedback
       (if _feedback (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _feedback
     (setq _feedback (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get heatmap_util::GetPowerFrissFeedback :md5sum-) "f1f168a39479bedb24dba7a087426182")
(setf (get heatmap_util::GetPowerFrissFeedback :datatype-) "heatmap_util/GetPowerFrissFeedback")
(setf (get heatmap_util::GetPowerFrissFeedback :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
bool feedback

")



(provide :heatmap_util/GetPowerFrissFeedback "f1f168a39479bedb24dba7a087426182")

