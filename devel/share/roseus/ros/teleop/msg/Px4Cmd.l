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
  :slots (_land ))

(defmethod teleop::Px4Cmd
  (:init
   (&key
    ((:land __land) nil)
    )
   (send-super :init)
   (setq _land __land)
   self)
  (:land
   (&optional (__land :null))
   (if (not (eq __land :null)) (setq _land __land)) _land)
  (:serialization-length
   ()
   (+
    ;; bool _land
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _land
       (if _land (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _land
     (setq _land (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get teleop::Px4Cmd :md5sum-) "2082f282fb525dbaba0ae95502684c61")
(setf (get teleop::Px4Cmd :datatype-) "teleop/Px4Cmd")
(setf (get teleop::Px4Cmd :definition-)
      "bool land
")



(provide :teleop/Px4Cmd "2082f282fb525dbaba0ae95502684c61")


