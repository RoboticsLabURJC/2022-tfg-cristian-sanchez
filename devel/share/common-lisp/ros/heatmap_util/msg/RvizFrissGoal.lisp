; Auto-generated. Do not edit!


(cl:in-package heatmap_util-msg)


;//! \htmlinclude RvizFrissGoal.msg.html

(cl:defclass <RvizFrissGoal> (roslisp-msg-protocol:ros-message)
  ((origin
    :reader origin
    :initarg :origin
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (heatmap_config
    :reader heatmap_config
    :initarg :heatmap_config
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass RvizFrissGoal (<RvizFrissGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RvizFrissGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RvizFrissGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name heatmap_util-msg:<RvizFrissGoal> is deprecated: use heatmap_util-msg:RvizFrissGoal instead.")))

(cl:ensure-generic-function 'origin-val :lambda-list '(m))
(cl:defmethod origin-val ((m <RvizFrissGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader heatmap_util-msg:origin-val is deprecated.  Use heatmap_util-msg:origin instead.")
  (origin m))

(cl:ensure-generic-function 'heatmap_config-val :lambda-list '(m))
(cl:defmethod heatmap_config-val ((m <RvizFrissGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader heatmap_util-msg:heatmap_config-val is deprecated.  Use heatmap_util-msg:heatmap_config instead.")
  (heatmap_config m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RvizFrissGoal>) ostream)
  "Serializes a message object of type '<RvizFrissGoal>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'origin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'origin))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'heatmap_config))))
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
   (cl:slot-value msg 'heatmap_config))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RvizFrissGoal>) istream)
  "Deserializes a message object of type '<RvizFrissGoal>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'origin) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'origin)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'heatmap_config) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'heatmap_config)))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RvizFrissGoal>)))
  "Returns string type for a message object of type '<RvizFrissGoal>"
  "heatmap_util/RvizFrissGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RvizFrissGoal)))
  "Returns string type for a message object of type 'RvizFrissGoal"
  "heatmap_util/RvizFrissGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RvizFrissGoal>)))
  "Returns md5sum for a message object of type '<RvizFrissGoal>"
  "01a81ba5d3520aaf0596aa1fa2695ac0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RvizFrissGoal)))
  "Returns md5sum for a message object of type 'RvizFrissGoal"
  "01a81ba5d3520aaf0596aa1fa2695ac0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RvizFrissGoal>)))
  "Returns full string definition for message of type '<RvizFrissGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%int16[] origin~%float64[] heatmap_config~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RvizFrissGoal)))
  "Returns full string definition for message of type 'RvizFrissGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%int16[] origin~%float64[] heatmap_config~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RvizFrissGoal>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'origin) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'heatmap_config) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RvizFrissGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'RvizFrissGoal
    (cl:cons ':origin (origin msg))
    (cl:cons ':heatmap_config (heatmap_config msg))
))
