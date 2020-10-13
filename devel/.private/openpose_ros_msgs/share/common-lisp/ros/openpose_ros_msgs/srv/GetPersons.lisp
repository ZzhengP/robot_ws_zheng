; Auto-generated. Do not edit!


(cl:in-package openpose_ros_msgs-srv)


;//! \htmlinclude GetPersons-request.msg.html

(cl:defclass <GetPersons-request> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass GetPersons-request (<GetPersons-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPersons-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPersons-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name openpose_ros_msgs-srv:<GetPersons-request> is deprecated: use openpose_ros_msgs-srv:GetPersons-request instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <GetPersons-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-srv:image-val is deprecated.  Use openpose_ros_msgs-srv:image instead.")
  (image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPersons-request>) ostream)
  "Serializes a message object of type '<GetPersons-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPersons-request>) istream)
  "Deserializes a message object of type '<GetPersons-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPersons-request>)))
  "Returns string type for a service object of type '<GetPersons-request>"
  "openpose_ros_msgs/GetPersonsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPersons-request)))
  "Returns string type for a service object of type 'GetPersons-request"
  "openpose_ros_msgs/GetPersonsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPersons-request>)))
  "Returns md5sum for a message object of type '<GetPersons-request>"
  "b372c9a3a4112a64a284d512401cc11f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPersons-request)))
  "Returns md5sum for a message object of type 'GetPersons-request"
  "b372c9a3a4112a64a284d512401cc11f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPersons-request>)))
  "Returns full string definition for message of type '<GetPersons-request>"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPersons-request)))
  "Returns full string definition for message of type 'GetPersons-request"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPersons-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPersons-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPersons-request
    (cl:cons ':image (image msg))
))
;//! \htmlinclude GetPersons-response.msg.html

(cl:defclass <GetPersons-response> (roslisp-msg-protocol:ros-message)
  ((detections
    :reader detections
    :initarg :detections
    :type (cl:vector openpose_ros_msgs-msg:PersonDetection)
   :initform (cl:make-array 0 :element-type 'openpose_ros_msgs-msg:PersonDetection :initial-element (cl:make-instance 'openpose_ros_msgs-msg:PersonDetection))))
)

(cl:defclass GetPersons-response (<GetPersons-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPersons-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPersons-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name openpose_ros_msgs-srv:<GetPersons-response> is deprecated: use openpose_ros_msgs-srv:GetPersons-response instead.")))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <GetPersons-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-srv:detections-val is deprecated.  Use openpose_ros_msgs-srv:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPersons-response>) ostream)
  "Serializes a message object of type '<GetPersons-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPersons-response>) istream)
  "Deserializes a message object of type '<GetPersons-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'openpose_ros_msgs-msg:PersonDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPersons-response>)))
  "Returns string type for a service object of type '<GetPersons-response>"
  "openpose_ros_msgs/GetPersonsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPersons-response)))
  "Returns string type for a service object of type 'GetPersons-response"
  "openpose_ros_msgs/GetPersonsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPersons-response>)))
  "Returns md5sum for a message object of type '<GetPersons-response>"
  "b372c9a3a4112a64a284d512401cc11f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPersons-response)))
  "Returns md5sum for a message object of type 'GetPersons-response"
  "b372c9a3a4112a64a284d512401cc11f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPersons-response>)))
  "Returns full string definition for message of type '<GetPersons-response>"
  (cl:format cl:nil "PersonDetection[] detections~%~%================================================================================~%MSG: openpose_ros_msgs/PersonDetection~%uint32 num_people_detected~%uint32 person_ID~%BodypartDetection nose~%BodypartDetection neck~%BodypartDetection right_shoulder~%BodypartDetection right_elbow~%BodypartDetection right_wrist~%BodypartDetection left_shoulder~%BodypartDetection left_elbow~%BodypartDetection left_wrist~%BodypartDetection right_hip~%BodypartDetection right_knee~%BodypartDetection right_ankle~%BodypartDetection left_hip~%BodypartDetection left_knee~%BodypartDetection left_ankle~%BodypartDetection right_eye~%BodypartDetection left_eye~%BodypartDetection right_ear~%BodypartDetection left_ear~%BodypartDetection chest~%~%================================================================================~%MSG: openpose_ros_msgs/BodypartDetection~%uint32 x~%uint32 y~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPersons-response)))
  "Returns full string definition for message of type 'GetPersons-response"
  (cl:format cl:nil "PersonDetection[] detections~%~%================================================================================~%MSG: openpose_ros_msgs/PersonDetection~%uint32 num_people_detected~%uint32 person_ID~%BodypartDetection nose~%BodypartDetection neck~%BodypartDetection right_shoulder~%BodypartDetection right_elbow~%BodypartDetection right_wrist~%BodypartDetection left_shoulder~%BodypartDetection left_elbow~%BodypartDetection left_wrist~%BodypartDetection right_hip~%BodypartDetection right_knee~%BodypartDetection right_ankle~%BodypartDetection left_hip~%BodypartDetection left_knee~%BodypartDetection left_ankle~%BodypartDetection right_eye~%BodypartDetection left_eye~%BodypartDetection right_ear~%BodypartDetection left_ear~%BodypartDetection chest~%~%================================================================================~%MSG: openpose_ros_msgs/BodypartDetection~%uint32 x~%uint32 y~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPersons-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPersons-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPersons-response
    (cl:cons ':detections (detections msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPersons)))
  'GetPersons-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPersons)))
  'GetPersons-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPersons)))
  "Returns string type for a service object of type '<GetPersons>"
  "openpose_ros_msgs/GetPersons")