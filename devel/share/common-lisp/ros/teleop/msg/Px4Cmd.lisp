; Auto-generated. Do not edit!


(cl:in-package teleop-msg)


;//! \htmlinclude Px4Cmd.msg.html

(cl:defclass <Px4Cmd> (roslisp-msg-protocol:ros-message)
  ((land
    :reader land
    :initarg :land
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Px4Cmd (<Px4Cmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Px4Cmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Px4Cmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name teleop-msg:<Px4Cmd> is deprecated: use teleop-msg:Px4Cmd instead.")))

(cl:ensure-generic-function 'land-val :lambda-list '(m))
(cl:defmethod land-val ((m <Px4Cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teleop-msg:land-val is deprecated.  Use teleop-msg:land instead.")
  (land m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Px4Cmd>) ostream)
  "Serializes a message object of type '<Px4Cmd>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'land) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Px4Cmd>) istream)
  "Deserializes a message object of type '<Px4Cmd>"
    (cl:setf (cl:slot-value msg 'land) (cl:not (cl:zerop (cl:read-byte istream))))
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
  "2082f282fb525dbaba0ae95502684c61")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Px4Cmd)))
  "Returns md5sum for a message object of type 'Px4Cmd"
  "2082f282fb525dbaba0ae95502684c61")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Px4Cmd>)))
  "Returns full string definition for message of type '<Px4Cmd>"
  (cl:format cl:nil "bool land~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Px4Cmd)))
  "Returns full string definition for message of type 'Px4Cmd"
  (cl:format cl:nil "bool land~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Px4Cmd>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Px4Cmd>))
  "Converts a ROS message object to a list"
  (cl:list 'Px4Cmd
    (cl:cons ':land (land msg))
))
