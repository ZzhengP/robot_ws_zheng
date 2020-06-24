; Auto-generated. Do not edit!


(cl:in-package kinect2_tracker-msg)


;//! \htmlinclude bounding_box.msg.html

(cl:defclass <bounding_box> (roslisp-msg-protocol:ros-message)
  ((min
    :reader min
    :initarg :min
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped))
   (max
    :reader max
    :initarg :max
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped)))
)

(cl:defclass bounding_box (<bounding_box>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bounding_box>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bounding_box)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kinect2_tracker-msg:<bounding_box> is deprecated: use kinect2_tracker-msg:bounding_box instead.")))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <bounding_box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect2_tracker-msg:min-val is deprecated.  Use kinect2_tracker-msg:min instead.")
  (min m))

(cl:ensure-generic-function 'max-val :lambda-list '(m))
(cl:defmethod max-val ((m <bounding_box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect2_tracker-msg:max-val is deprecated.  Use kinect2_tracker-msg:max instead.")
  (max m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bounding_box>) ostream)
  "Serializes a message object of type '<bounding_box>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bounding_box>) istream)
  "Deserializes a message object of type '<bounding_box>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bounding_box>)))
  "Returns string type for a message object of type '<bounding_box>"
  "kinect2_tracker/bounding_box")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bounding_box)))
  "Returns string type for a message object of type 'bounding_box"
  "kinect2_tracker/bounding_box")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bounding_box>)))
  "Returns md5sum for a message object of type '<bounding_box>"
  "5a9f041aa663bad83df903fb4bed6c5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bounding_box)))
  "Returns md5sum for a message object of type 'bounding_box"
  "5a9f041aa663bad83df903fb4bed6c5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bounding_box>)))
  "Returns full string definition for message of type '<bounding_box>"
  (cl:format cl:nil "geometry_msgs/PointStamped min~%geometry_msgs/PointStamped max~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bounding_box)))
  "Returns full string definition for message of type 'bounding_box"
  (cl:format cl:nil "geometry_msgs/PointStamped min~%geometry_msgs/PointStamped max~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bounding_box>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bounding_box>))
  "Converts a ROS message object to a list"
  (cl:list 'bounding_box
    (cl:cons ':min (min msg))
    (cl:cons ':max (max msg))
))
