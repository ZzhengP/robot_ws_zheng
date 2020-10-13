; Auto-generated. Do not edit!


(cl:in-package kinect2_tracker-msg)


;//! \htmlinclude user_points.msg.html

(cl:defclass <user_points> (roslisp-msg-protocol:ros-message)
  ((users
    :reader users
    :initarg :users
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (people_points
    :reader people_points
    :initarg :people_points
    :type (cl:vector geometry_msgs-msg:PointStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PointStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PointStamped)))
   (boxes
    :reader boxes
    :initarg :boxes
    :type (cl:vector kinect2_tracker-msg:bounding_box)
   :initform (cl:make-array 0 :element-type 'kinect2_tracker-msg:bounding_box :initial-element (cl:make-instance 'kinect2_tracker-msg:bounding_box))))
)

(cl:defclass user_points (<user_points>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <user_points>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'user_points)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kinect2_tracker-msg:<user_points> is deprecated: use kinect2_tracker-msg:user_points instead.")))

(cl:ensure-generic-function 'users-val :lambda-list '(m))
(cl:defmethod users-val ((m <user_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect2_tracker-msg:users-val is deprecated.  Use kinect2_tracker-msg:users instead.")
  (users m))

(cl:ensure-generic-function 'people_points-val :lambda-list '(m))
(cl:defmethod people_points-val ((m <user_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect2_tracker-msg:people_points-val is deprecated.  Use kinect2_tracker-msg:people_points instead.")
  (people_points m))

(cl:ensure-generic-function 'boxes-val :lambda-list '(m))
(cl:defmethod boxes-val ((m <user_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect2_tracker-msg:boxes-val is deprecated.  Use kinect2_tracker-msg:boxes instead.")
  (boxes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <user_points>) ostream)
  "Serializes a message object of type '<user_points>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'users))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'users))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'people_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'people_points))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'boxes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'boxes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <user_points>) istream)
  "Deserializes a message object of type '<user_points>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'users) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'users)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'people_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'people_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PointStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'boxes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'boxes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kinect2_tracker-msg:bounding_box))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<user_points>)))
  "Returns string type for a message object of type '<user_points>"
  "kinect2_tracker/user_points")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'user_points)))
  "Returns string type for a message object of type 'user_points"
  "kinect2_tracker/user_points")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<user_points>)))
  "Returns md5sum for a message object of type '<user_points>"
  "6624cab0be89cfdee984fa8acaafad2a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'user_points)))
  "Returns md5sum for a message object of type 'user_points"
  "6624cab0be89cfdee984fa8acaafad2a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<user_points>)))
  "Returns full string definition for message of type '<user_points>"
  (cl:format cl:nil "# Message storing the points of users.~%# Header header~%uint8[] users~%geometry_msgs/PointStamped[] people_points~%bounding_box[] boxes~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: kinect2_tracker/bounding_box~%geometry_msgs/PointStamped min~%geometry_msgs/PointStamped max~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'user_points)))
  "Returns full string definition for message of type 'user_points"
  (cl:format cl:nil "# Message storing the points of users.~%# Header header~%uint8[] users~%geometry_msgs/PointStamped[] people_points~%bounding_box[] boxes~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: kinect2_tracker/bounding_box~%geometry_msgs/PointStamped min~%geometry_msgs/PointStamped max~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <user_points>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'users) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'people_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'boxes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <user_points>))
  "Converts a ROS message object to a list"
  (cl:list 'user_points
    (cl:cons ':users (users msg))
    (cl:cons ':people_points (people_points msg))
    (cl:cons ':boxes (boxes msg))
))
