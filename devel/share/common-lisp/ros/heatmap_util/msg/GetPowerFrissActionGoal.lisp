; Auto-generated. Do not edit!


(cl:in-package heatmap_util-msg)


;//! \htmlinclude GetPowerFrissActionGoal.msg.html

(cl:defclass <GetPowerFrissActionGoal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal_id
    :reader goal_id
    :initarg :goal_id
    :type actionlib_msgs-msg:GoalID
    :initform (cl:make-instance 'actionlib_msgs-msg:GoalID))
   (goal
    :reader goal
    :initarg :goal
    :type heatmap_util-msg:GetPowerFrissGoal
    :initform (cl:make-instance 'heatmap_util-msg:GetPowerFrissGoal)))
)

(cl:defclass GetPowerFrissActionGoal (<GetPowerFrissActionGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPowerFrissActionGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPowerFrissActionGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name heatmap_util-msg:<GetPowerFrissActionGoal> is deprecated: use heatmap_util-msg:GetPowerFrissActionGoal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GetPowerFrissActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader heatmap_util-msg:header-val is deprecated.  Use heatmap_util-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goal_id-val :lambda-list '(m))
(cl:defmethod goal_id-val ((m <GetPowerFrissActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader heatmap_util-msg:goal_id-val is deprecated.  Use heatmap_util-msg:goal_id instead.")
  (goal_id m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <GetPowerFrissActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader heatmap_util-msg:goal-val is deprecated.  Use heatmap_util-msg:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPowerFrissActionGoal>) ostream)
  "Serializes a message object of type '<GetPowerFrissActionGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPowerFrissActionGoal>) istream)
  "Deserializes a message object of type '<GetPowerFrissActionGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPowerFrissActionGoal>)))
  "Returns string type for a message object of type '<GetPowerFrissActionGoal>"
  "heatmap_util/GetPowerFrissActionGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPowerFrissActionGoal)))
  "Returns string type for a message object of type 'GetPowerFrissActionGoal"
  "heatmap_util/GetPowerFrissActionGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPowerFrissActionGoal>)))
  "Returns md5sum for a message object of type '<GetPowerFrissActionGoal>"
  "be3b09da80630cad7766f4eceae85ef1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPowerFrissActionGoal)))
  "Returns md5sum for a message object of type 'GetPowerFrissActionGoal"
  "be3b09da80630cad7766f4eceae85ef1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPowerFrissActionGoal>)))
  "Returns full string definition for message of type '<GetPowerFrissActionGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%GetPowerFrissGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: heatmap_util/GetPowerFrissGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%int16[] index~%bool obstacle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPowerFrissActionGoal)))
  "Returns full string definition for message of type 'GetPowerFrissActionGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%GetPowerFrissGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: heatmap_util/GetPowerFrissGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%int16[] index~%bool obstacle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPowerFrissActionGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPowerFrissActionGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPowerFrissActionGoal
    (cl:cons ':header (header msg))
    (cl:cons ':goal_id (goal_id msg))
    (cl:cons ':goal (goal msg))
))
