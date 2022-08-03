; Auto-generated. Do not edit!


(cl:in-package teleop-msg)


;//! \htmlinclude Px4Cmd.msg.html

(cl:defclass <Px4Cmd> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Px4Cmd (<Px4Cmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Px4Cmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Px4Cmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name teleop-msg:<Px4Cmd> is deprecated: use teleop-msg:Px4Cmd instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <Px4Cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teleop-msg:cmd-val is deprecated.  Use teleop-msg:cmd instead.")
  (cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Px4Cmd>) ostream)
  "Serializes a message object of type '<Px4Cmd>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Px4Cmd>) istream)
  "Deserializes a message object of type '<Px4Cmd>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Px4Cmd>)))
  "Returns string type for a message object of type '<Px4Cmd>"
  "teleop/Px4Cmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Px4Cmd)))
  "Returns string type for a message object of type 'Px4Cmd"
  "teleop/Px4Cmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Px4Cmd>)))
  "Returns md5sum for a message object of type '<Px4Cmd>"
  "26f05c6e9fb9de81f12f2b92304c2961")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Px4Cmd)))
  "Returns md5sum for a message object of type 'Px4Cmd"
  "26f05c6e9fb9de81f12f2b92304c2961")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Px4Cmd>)))
  "Returns full string definition for message of type '<Px4Cmd>"
  (cl:format cl:nil "uint8 cmd~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Px4Cmd)))
  "Returns full string definition for message of type 'Px4Cmd"
  (cl:format cl:nil "uint8 cmd~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Px4Cmd>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Px4Cmd>))
  "Converts a ROS message object to a list"
  (cl:list 'Px4Cmd
    (cl:cons ':cmd (cmd msg))
))
