; Auto-generated. Do not edit!


(cl:in-package heatmap_util-msg)


;//! \htmlinclude RvizFrissResult.msg.html

(cl:defclass <RvizFrissResult> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (size
    :reader size
    :initarg :size
    :type cl:fixnum
    :initform 0))
)

(cl:defclass RvizFrissResult (<RvizFrissResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RvizFrissResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RvizFrissResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name heatmap_util-msg:<RvizFrissResult> is deprecated: use heatmap_util-msg:RvizFrissResult instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <RvizFrissResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader heatmap_util-msg:data-val is deprecated.  Use heatmap_util-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <RvizFrissResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader heatmap_util-msg:size-val is deprecated.  Use heatmap_util-msg:size instead.")
  (size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RvizFrissResult>) ostream)
  "Serializes a message object of type '<RvizFrissResult>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'data))
  (cl:let* ((signed (cl:slot-value msg 'size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RvizFrissResult>) istream)
  "Deserializes a message object of type '<RvizFrissResult>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'size) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RvizFrissResult>)))
  "Returns string type for a message object of type '<RvizFrissResult>"
  "heatmap_util/RvizFrissResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RvizFrissResult)))
  "Returns string type for a message object of type 'RvizFrissResult"
  "heatmap_util/RvizFrissResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RvizFrissResult>)))
  "Returns md5sum for a message object of type '<RvizFrissResult>"
  "3cf2e4aade105b07f617c7c61e3ce58c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RvizFrissResult)))
  "Returns md5sum for a message object of type 'RvizFrissResult"
  "3cf2e4aade105b07f617c7c61e3ce58c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RvizFrissResult>)))
  "Returns full string definition for message of type '<RvizFrissResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%float64[] data~%int16 size~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RvizFrissResult)))
  "Returns full string definition for message of type 'RvizFrissResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%float64[] data~%int16 size~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RvizFrissResult>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RvizFrissResult>))
  "Converts a ROS message object to a list"
  (cl:list 'RvizFrissResult
    (cl:cons ':data (data msg))
    (cl:cons ':size (size msg))
))
