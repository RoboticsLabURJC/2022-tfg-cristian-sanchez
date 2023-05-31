; Auto-generated. Do not edit!


(cl:in-package heatmap_util-msg)


;//! \htmlinclude GetPowerFrissResult.msg.html

(cl:defclass <GetPowerFrissResult> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0)
   (size
    :reader size
    :initarg :size
    :type cl:fixnum
    :initform 0)
   (source_coords
    :reader source_coords
    :initarg :source_coords
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass GetPowerFrissResult (<GetPowerFrissResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPowerFrissResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPowerFrissResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name heatmap_util-msg:<GetPowerFrissResult> is deprecated: use heatmap_util-msg:GetPowerFrissResult instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <GetPowerFrissResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader heatmap_util-msg:data-val is deprecated.  Use heatmap_util-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <GetPowerFrissResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader heatmap_util-msg:size-val is deprecated.  Use heatmap_util-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'source_coords-val :lambda-list '(m))
(cl:defmethod source_coords-val ((m <GetPowerFrissResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader heatmap_util-msg:source_coords-val is deprecated.  Use heatmap_util-msg:source_coords instead.")
  (source_coords m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPowerFrissResult>) ostream)
  "Serializes a message object of type '<GetPowerFrissResult>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'source_coords))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'source_coords))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPowerFrissResult>) istream)
  "Deserializes a message object of type '<GetPowerFrissResult>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'size) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'source_coords) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'source_coords)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPowerFrissResult>)))
  "Returns string type for a message object of type '<GetPowerFrissResult>"
  "heatmap_util/GetPowerFrissResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPowerFrissResult)))
  "Returns string type for a message object of type 'GetPowerFrissResult"
  "heatmap_util/GetPowerFrissResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPowerFrissResult>)))
  "Returns md5sum for a message object of type '<GetPowerFrissResult>"
  "8a3f44e19130d36ab5f37e91f8827dfc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPowerFrissResult)))
  "Returns md5sum for a message object of type 'GetPowerFrissResult"
  "8a3f44e19130d36ab5f37e91f8827dfc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPowerFrissResult>)))
  "Returns full string definition for message of type '<GetPowerFrissResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%float64 data~%int16 size~%int16[] source_coords~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPowerFrissResult)))
  "Returns full string definition for message of type 'GetPowerFrissResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%float64 data~%int16 size~%int16[] source_coords~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPowerFrissResult>))
  (cl:+ 0
     8
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'source_coords) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPowerFrissResult>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPowerFrissResult
    (cl:cons ':data (data msg))
    (cl:cons ':size (size msg))
    (cl:cons ':source_coords (source_coords msg))
))
