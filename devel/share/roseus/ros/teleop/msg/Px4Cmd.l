;; Auto-generated. Do not edit!


(when (boundp 'teleop::Px4Cmd)
  (if (not (find-package "TELEOP"))
    (make-package "TELEOP"))
  (shadow 'Px4Cmd (find-package "TELEOP")))
(unless (find-package "TELEOP::PX4CMD")
  (make-package "TELEOP::PX4CMD"))

(in-package "ROS")
;;//! \htmlinclude Px4Cmd.msg.html


(defclass teleop::Px4Cmd
  :super ros::object
  :slots (_cmd ))

(defmethod teleop::Px4Cmd
  (:init
   (&key
    ((:cmd __cmd) 0)
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:serialization-length
   ()
   (+
    ;; uint8 _cmd
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _cmd
       (write-byte _cmd s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _cmd
     (setq _cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get teleop::Px4Cmd :md5sum-) "26f05c6e9fb9de81f12f2b92304c2961")
(setf (get teleop::Px4Cmd :datatype-) "teleop/Px4Cmd")
(setf (get teleop::Px4Cmd :definition-)
      "uint8 cmd
")



(provide :teleop/Px4Cmd "26f05c6e9fb9de81f12f2b92304c2961")


