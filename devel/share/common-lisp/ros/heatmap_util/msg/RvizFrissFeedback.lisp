; Auto-generated. Do not edit!


(cl:in-package heatmap_util-msg)


;//! \htmlinclude RvizFrissFeedback.msg.html

(cl:defclass <RvizFrissFeedback> (roslisp-msg-protocol:ros-message)
  ((feedback
    :reader feedback
    :initarg :feedback
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RvizFrissFeedback (<RvizFrissFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RvizFrissFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RvizFrissFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name heatmap_util-msg:<RvizFrissFeedback> is deprecated: use heatmap_util-msg:RvizFrissFeedback instead.")))

(cl:ensure-generic-function 'feedback-val :lambda-list '(m))
(cl:defmethod feedback-val ((m <RvizFrissFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader heatmap_util-msg:feedback-val is deprecated.  Use heatmap_util-msg:feedback instead.")
  (feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RvizFrissFeedback>) ostream)
  "Serializes a message object of type '<RvizFrissFeedback>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'feedback) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RvizFrissFeedback>) istream)
  "Deserializes a message object of type '<RvizFrissFeedback>"
    (cl:setf (cl:slot-value msg 'feedback) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RvizFrissFeedback>)))
  "Returns string type for a message object of type '<RvizFrissFeedback>"
  "heatmap_util/RvizFrissFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RvizFrissFeedback)))
  "Returns string type for a message object of type 'RvizFrissFeedback"
  "heatmap_util/RvizFrissFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RvizFrissFeedback>)))
  "Returns md5sum for a message object of type '<RvizFrissFeedback>"
  "f1f168a39479bedb24dba7a087426182")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RvizFrissFeedback)))
  "Returns md5sum for a message object of type 'RvizFrissFeedback"
  "f1f168a39479bedb24dba7a087426182")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RvizFrissFeedback>)))
  "Returns full string definition for message of type '<RvizFrissFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%bool feedback~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RvizFrissFeedback)))
  "Returns full string definition for message of type 'RvizFrissFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%bool feedback~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RvizFrissFeedback>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RvizFrissFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'RvizFrissFeedback
    (cl:cons ':feedback (feedback msg))
))
