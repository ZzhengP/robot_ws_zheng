; Auto-generated. Do not edit!


(cl:in-package openni2_tracker-msg)


;//! \htmlinclude TrackerUserArray.msg.html

(cl:defclass <TrackerUserArray> (roslisp-msg-protocol:ros-message)
  ((users
    :reader users
    :initarg :users
    :type (cl:vector openni2_tracker-msg:TrackerUser)
   :initform (cl:make-array 0 :element-type 'openni2_tracker-msg:TrackerUser :initial-element (cl:make-instance 'openni2_tracker-msg:TrackerUser)))
   (numUsers
    :reader numUsers
    :initarg :numUsers
    :type cl:fixnum
    :initform 0))
)

(cl:defclass TrackerUserArray (<TrackerUserArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackerUserArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackerUserArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name openni2_tracker-msg:<TrackerUserArray> is deprecated: use openni2_tracker-msg:TrackerUserArray instead.")))

(cl:ensure-generic-function 'users-val :lambda-list '(m))
(cl:defmethod users-val ((m <TrackerUserArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openni2_tracker-msg:users-val is deprecated.  Use openni2_tracker-msg:users instead.")
  (users m))

(cl:ensure-generic-function 'numUsers-val :lambda-list '(m))
(cl:defmethod numUsers-val ((m <TrackerUserArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openni2_tracker-msg:numUsers-val is deprecated.  Use openni2_tracker-msg:numUsers instead.")
  (numUsers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackerUserArray>) ostream)
  "Serializes a message object of type '<TrackerUserArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'users))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'users))
  (cl:let* ((signed (cl:slot-value msg 'numUsers)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackerUserArray>) istream)
  "Deserializes a message object of type '<TrackerUserArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'users) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'users)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'openni2_tracker-msg:TrackerUser))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'numUsers) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackerUserArray>)))
  "Returns string type for a message object of type '<TrackerUserArray>"
  "openni2_tracker/TrackerUserArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackerUserArray)))
  "Returns string type for a message object of type 'TrackerUserArray"
  "openni2_tracker/TrackerUserArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackerUserArray>)))
  "Returns md5sum for a message object of type '<TrackerUserArray>"
  "825d95b7c6acaad9dc439d2148963f2e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackerUserArray)))
  "Returns md5sum for a message object of type 'TrackerUserArray"
  "825d95b7c6acaad9dc439d2148963f2e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackerUserArray>)))
  "Returns full string definition for message of type '<TrackerUserArray>"
  (cl:format cl:nil "# TrackerUserArray: This message contains a vector of openni2_tracker/TrackerUser messages.~%openni2_tracker/TrackerUser[] users~%int8 numUsers~%~%================================================================================~%MSG: openni2_tracker/TrackerUser~%# TrackerUser: Contains a snapshot of a single users tracking data. an ID, a vector of frames corresponding to each joint, a vector of confidences corresponding to each joint, and a vector of geometry_msgs/Transform messages corresponding to each joint. ~%Header header~%uint8 uid~%string tracker_id~%string[] frames~%float64[] confs~%geometry_msgs/Transform[] transforms~%geometry_msgs/Vector3[] projective~%geometry_msgs/Vector3 center_of_mass~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackerUserArray)))
  "Returns full string definition for message of type 'TrackerUserArray"
  (cl:format cl:nil "# TrackerUserArray: This message contains a vector of openni2_tracker/TrackerUser messages.~%openni2_tracker/TrackerUser[] users~%int8 numUsers~%~%================================================================================~%MSG: openni2_tracker/TrackerUser~%# TrackerUser: Contains a snapshot of a single users tracking data. an ID, a vector of frames corresponding to each joint, a vector of confidences corresponding to each joint, and a vector of geometry_msgs/Transform messages corresponding to each joint. ~%Header header~%uint8 uid~%string tracker_id~%string[] frames~%float64[] confs~%geometry_msgs/Transform[] transforms~%geometry_msgs/Vector3[] projective~%geometry_msgs/Vector3 center_of_mass~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackerUserArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'users) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackerUserArray>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackerUserArray
    (cl:cons ':users (users msg))
    (cl:cons ':numUsers (numUsers msg))
))
