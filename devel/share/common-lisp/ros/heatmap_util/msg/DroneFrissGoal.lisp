; Auto-generated. Do not edit!


(cl:in-package heatmap_util-msg)


;//! \htmlinclude DroneFrissGoal.msg.html

(cl:defclass <DroneFrissGoal> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass DroneFrissGoal (<DroneFrissGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DroneFrissGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DroneFrissGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name heatmap_util-msg:<DroneFrissGoal> is deprecated: use heatmap_util-msg:DroneFrissGoal instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <DroneFrissGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader heatmap_util-msg:index-val is deprecated.  Use heatmap_util-msg:index instead.")
  (index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DroneFrissGoal>) ostream)
  "Serializes a message object of type '<DroneFrissGoal>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'index))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'index))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DroneFrissGoal>) istream)
  "Deserializes a message object of type '<DroneFrissGoal>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'index) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'index)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DroneFrissGoal>)))
  "Returns string type for a message object of type '<DroneFrissGoal>"
  "heatmap_util/DroneFrissGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DroneFrissGoal)))
  "Returns string type for a message object of type 'DroneFrissGoal"
  "heatmap_util/DroneFrissGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DroneFrissGoal>)))
  "Returns md5sum for a message object of type '<DroneFrissGoal>"
  "2916b3cbd91737121b9062b1c41f982f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DroneFrissGoal)))
  "Returns md5sum for a message object of type 'DroneFrissGoal"
  "2916b3cbd91737121b9062b1c41f982f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DroneFrissGoal>)))
  "Returns full string definition for message of type '<DroneFrissGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%int8[] index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DroneFrissGoal)))
  "Returns full string definition for message of type 'DroneFrissGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%int8[] index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DroneFrissGoal>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'index) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DroneFrissGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'DroneFrissGoal
    (cl:cons ':index (index msg))
))
