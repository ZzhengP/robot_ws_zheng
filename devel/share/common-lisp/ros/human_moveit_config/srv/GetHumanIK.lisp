; Auto-generated. Do not edit!


(cl:in-package human_moveit_config-srv)


;//! \htmlinclude GetHumanIK-request.msg.html

(cl:defclass <GetHumanIK-request> (roslisp-msg-protocol:ros-message)
  ((desired_poses
    :reader desired_poses
    :initarg :desired_poses
    :type (cl:vector geometry_msgs-msg:PoseStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PoseStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
   (fixed_joints
    :reader fixed_joints
    :initarg :fixed_joints
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState))
   (tolerance
    :reader tolerance
    :initarg :tolerance
    :type cl:float
    :initform 0.0)
   (group_names
    :reader group_names
    :initarg :group_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (seed
    :reader seed
    :initarg :seed
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState)))
)

(cl:defclass GetHumanIK-request (<GetHumanIK-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetHumanIK-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetHumanIK-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name human_moveit_config-srv:<GetHumanIK-request> is deprecated: use human_moveit_config-srv:GetHumanIK-request instead.")))

(cl:ensure-generic-function 'desired_poses-val :lambda-list '(m))
(cl:defmethod desired_poses-val ((m <GetHumanIK-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_moveit_config-srv:desired_poses-val is deprecated.  Use human_moveit_config-srv:desired_poses instead.")
  (desired_poses m))

(cl:ensure-generic-function 'fixed_joints-val :lambda-list '(m))
(cl:defmethod fixed_joints-val ((m <GetHumanIK-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_moveit_config-srv:fixed_joints-val is deprecated.  Use human_moveit_config-srv:fixed_joints instead.")
  (fixed_joints m))

(cl:ensure-generic-function 'tolerance-val :lambda-list '(m))
(cl:defmethod tolerance-val ((m <GetHumanIK-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_moveit_config-srv:tolerance-val is deprecated.  Use human_moveit_config-srv:tolerance instead.")
  (tolerance m))

(cl:ensure-generic-function 'group_names-val :lambda-list '(m))
(cl:defmethod group_names-val ((m <GetHumanIK-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_moveit_config-srv:group_names-val is deprecated.  Use human_moveit_config-srv:group_names instead.")
  (group_names m))

(cl:ensure-generic-function 'seed-val :lambda-list '(m))
(cl:defmethod seed-val ((m <GetHumanIK-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_moveit_config-srv:seed-val is deprecated.  Use human_moveit_config-srv:seed instead.")
  (seed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetHumanIK-request>) ostream)
  "Serializes a message object of type '<GetHumanIK-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'desired_poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'desired_poses))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'fixed_joints) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tolerance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'group_names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'group_names))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'seed) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetHumanIK-request>) istream)
  "Deserializes a message object of type '<GetHumanIK-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'desired_poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'desired_poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PoseStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'fixed_joints) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tolerance) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'group_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'group_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'seed) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetHumanIK-request>)))
  "Returns string type for a service object of type '<GetHumanIK-request>"
  "human_moveit_config/GetHumanIKRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHumanIK-request)))
  "Returns string type for a service object of type 'GetHumanIK-request"
  "human_moveit_config/GetHumanIKRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetHumanIK-request>)))
  "Returns md5sum for a message object of type '<GetHumanIK-request>"
  "2602509d5e7f9ded6d3747732576ab30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetHumanIK-request)))
  "Returns md5sum for a message object of type 'GetHumanIK-request"
  "2602509d5e7f9ded6d3747732576ab30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetHumanIK-request>)))
  "Returns full string definition for message of type '<GetHumanIK-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%geometry_msgs/PoseStamped[] desired_poses~%sensor_msgs/JointState fixed_joints~%float32 tolerance~%string[] group_names~%sensor_msgs/JointState seed~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetHumanIK-request)))
  "Returns full string definition for message of type 'GetHumanIK-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%geometry_msgs/PoseStamped[] desired_poses~%sensor_msgs/JointState fixed_joints~%float32 tolerance~%string[] group_names~%sensor_msgs/JointState seed~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetHumanIK-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'desired_poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'fixed_joints))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'group_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'seed))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetHumanIK-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetHumanIK-request
    (cl:cons ':desired_poses (desired_poses msg))
    (cl:cons ':fixed_joints (fixed_joints msg))
    (cl:cons ':tolerance (tolerance msg))
    (cl:cons ':group_names (group_names msg))
    (cl:cons ':seed (seed msg))
))
;//! \htmlinclude GetHumanIK-response.msg.html

(cl:defclass <GetHumanIK-response> (roslisp-msg-protocol:ros-message)
  ((joint_state
    :reader joint_state
    :initarg :joint_state
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState)))
)

(cl:defclass GetHumanIK-response (<GetHumanIK-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetHumanIK-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetHumanIK-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name human_moveit_config-srv:<GetHumanIK-response> is deprecated: use human_moveit_config-srv:GetHumanIK-response instead.")))

(cl:ensure-generic-function 'joint_state-val :lambda-list '(m))
(cl:defmethod joint_state-val ((m <GetHumanIK-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_moveit_config-srv:joint_state-val is deprecated.  Use human_moveit_config-srv:joint_state instead.")
  (joint_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetHumanIK-response>) ostream)
  "Serializes a message object of type '<GetHumanIK-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetHumanIK-response>) istream)
  "Deserializes a message object of type '<GetHumanIK-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetHumanIK-response>)))
  "Returns string type for a service object of type '<GetHumanIK-response>"
  "human_moveit_config/GetHumanIKResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHumanIK-response)))
  "Returns string type for a service object of type 'GetHumanIK-response"
  "human_moveit_config/GetHumanIKResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetHumanIK-response>)))
  "Returns md5sum for a message object of type '<GetHumanIK-response>"
  "2602509d5e7f9ded6d3747732576ab30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetHumanIK-response)))
  "Returns md5sum for a message object of type 'GetHumanIK-response"
  "2602509d5e7f9ded6d3747732576ab30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetHumanIK-response>)))
  "Returns full string definition for message of type '<GetHumanIK-response>"
  (cl:format cl:nil "sensor_msgs/JointState joint_state~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetHumanIK-response)))
  "Returns full string definition for message of type 'GetHumanIK-response"
  (cl:format cl:nil "sensor_msgs/JointState joint_state~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetHumanIK-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetHumanIK-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetHumanIK-response
    (cl:cons ':joint_state (joint_state msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetHumanIK)))
  'GetHumanIK-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetHumanIK)))
  'GetHumanIK-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHumanIK)))
  "Returns string type for a service object of type '<GetHumanIK>"
  "human_moveit_config/GetHumanIK")