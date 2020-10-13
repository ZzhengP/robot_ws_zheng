; Auto-generated. Do not edit!


(cl:in-package openpose_ros_msgs-msg)


;//! \htmlinclude OpenPoseHumanList.msg.html

(cl:defclass <OpenPoseHumanList> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (image_header
    :reader image_header
    :initarg :image_header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (num_humans
    :reader num_humans
    :initarg :num_humans
    :type cl:integer
    :initform 0)
   (human_list
    :reader human_list
    :initarg :human_list
    :type (cl:vector openpose_ros_msgs-msg:OpenPoseHuman)
   :initform (cl:make-array 0 :element-type 'openpose_ros_msgs-msg:OpenPoseHuman :initial-element (cl:make-instance 'openpose_ros_msgs-msg:OpenPoseHuman))))
)

(cl:defclass OpenPoseHumanList (<OpenPoseHumanList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OpenPoseHumanList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OpenPoseHumanList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name openpose_ros_msgs-msg:<OpenPoseHumanList> is deprecated: use openpose_ros_msgs-msg:OpenPoseHumanList instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <OpenPoseHumanList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:header-val is deprecated.  Use openpose_ros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'image_header-val :lambda-list '(m))
(cl:defmethod image_header-val ((m <OpenPoseHumanList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:image_header-val is deprecated.  Use openpose_ros_msgs-msg:image_header instead.")
  (image_header m))

(cl:ensure-generic-function 'num_humans-val :lambda-list '(m))
(cl:defmethod num_humans-val ((m <OpenPoseHumanList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:num_humans-val is deprecated.  Use openpose_ros_msgs-msg:num_humans instead.")
  (num_humans m))

(cl:ensure-generic-function 'human_list-val :lambda-list '(m))
(cl:defmethod human_list-val ((m <OpenPoseHumanList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:human_list-val is deprecated.  Use openpose_ros_msgs-msg:human_list instead.")
  (human_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OpenPoseHumanList>) ostream)
  "Serializes a message object of type '<OpenPoseHumanList>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image_header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'num_humans)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'human_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'human_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OpenPoseHumanList>) istream)
  "Deserializes a message object of type '<OpenPoseHumanList>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image_header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_humans) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'human_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'human_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'openpose_ros_msgs-msg:OpenPoseHuman))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OpenPoseHumanList>)))
  "Returns string type for a message object of type '<OpenPoseHumanList>"
  "openpose_ros_msgs/OpenPoseHumanList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OpenPoseHumanList)))
  "Returns string type for a message object of type 'OpenPoseHumanList"
  "openpose_ros_msgs/OpenPoseHumanList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OpenPoseHumanList>)))
  "Returns md5sum for a message object of type '<OpenPoseHumanList>"
  "520d81a8dc226a535b021aef637b0340")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OpenPoseHumanList)))
  "Returns md5sum for a message object of type 'OpenPoseHumanList"
  "520d81a8dc226a535b021aef637b0340")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OpenPoseHumanList>)))
  "Returns full string definition for message of type '<OpenPoseHumanList>"
  (cl:format cl:nil "Header header~%Header image_header~%int32 num_humans~%OpenPoseHuman[] human_list~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: openpose_ros_msgs/OpenPoseHuman~%int32 num_body_key_points_with_non_zero_prob~%int32 num_face_key_points_with_non_zero_prob~%int32 num_right_hand_key_points_with_non_zero_prob~%int32 num_left_hand_key_points_with_non_zero_prob~%BoundingBox body_bounding_box~%BoundingBox face_bounding_box~%PointWithProb[25] body_key_points_with_prob~%PointWithProb[70] face_key_points_with_prob~%PointWithProb[21] right_hand_key_points_with_prob~%PointWithProb[21] left_hand_key_points_with_prob~%================================================================================~%MSG: openpose_ros_msgs/BoundingBox~%float64 x~%float64 y~%float64 width~%float64 height~%================================================================================~%MSG: openpose_ros_msgs/PointWithProb~%float64 x~%float64 y~%float64 prob~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OpenPoseHumanList)))
  "Returns full string definition for message of type 'OpenPoseHumanList"
  (cl:format cl:nil "Header header~%Header image_header~%int32 num_humans~%OpenPoseHuman[] human_list~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: openpose_ros_msgs/OpenPoseHuman~%int32 num_body_key_points_with_non_zero_prob~%int32 num_face_key_points_with_non_zero_prob~%int32 num_right_hand_key_points_with_non_zero_prob~%int32 num_left_hand_key_points_with_non_zero_prob~%BoundingBox body_bounding_box~%BoundingBox face_bounding_box~%PointWithProb[25] body_key_points_with_prob~%PointWithProb[70] face_key_points_with_prob~%PointWithProb[21] right_hand_key_points_with_prob~%PointWithProb[21] left_hand_key_points_with_prob~%================================================================================~%MSG: openpose_ros_msgs/BoundingBox~%float64 x~%float64 y~%float64 width~%float64 height~%================================================================================~%MSG: openpose_ros_msgs/PointWithProb~%float64 x~%float64 y~%float64 prob~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OpenPoseHumanList>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image_header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'human_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OpenPoseHumanList>))
  "Converts a ROS message object to a list"
  (cl:list 'OpenPoseHumanList
    (cl:cons ':header (header msg))
    (cl:cons ':image_header (image_header msg))
    (cl:cons ':num_humans (num_humans msg))
    (cl:cons ':human_list (human_list msg))
))
