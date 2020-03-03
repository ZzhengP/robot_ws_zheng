; Auto-generated. Do not edit!


(cl:in-package openpose_ros_msgs-msg)


;//! \htmlinclude PersonDetection.msg.html

(cl:defclass <PersonDetection> (roslisp-msg-protocol:ros-message)
  ((face_landmark
    :reader face_landmark
    :initarg :face_landmark
    :type (cl:vector openpose_ros_msgs-msg:BodyPartDetection)
   :initform (cl:make-array 0 :element-type 'openpose_ros_msgs-msg:BodyPartDetection :initial-element (cl:make-instance 'openpose_ros_msgs-msg:BodyPartDetection)))
   (body_part
    :reader body_part
    :initarg :body_part
    :type (cl:vector openpose_ros_msgs-msg:BodyPartDetection)
   :initform (cl:make-array 0 :element-type 'openpose_ros_msgs-msg:BodyPartDetection :initial-element (cl:make-instance 'openpose_ros_msgs-msg:BodyPartDetection))))
)

(cl:defclass PersonDetection (<PersonDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PersonDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PersonDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name openpose_ros_msgs-msg:<PersonDetection> is deprecated: use openpose_ros_msgs-msg:PersonDetection instead.")))

(cl:ensure-generic-function 'face_landmark-val :lambda-list '(m))
(cl:defmethod face_landmark-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:face_landmark-val is deprecated.  Use openpose_ros_msgs-msg:face_landmark instead.")
  (face_landmark m))

(cl:ensure-generic-function 'body_part-val :lambda-list '(m))
(cl:defmethod body_part-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:body_part-val is deprecated.  Use openpose_ros_msgs-msg:body_part instead.")
  (body_part m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PersonDetection>) ostream)
  "Serializes a message object of type '<PersonDetection>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'face_landmark))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'face_landmark))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'body_part))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'body_part))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PersonDetection>) istream)
  "Deserializes a message object of type '<PersonDetection>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'face_landmark) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'face_landmark)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'openpose_ros_msgs-msg:BodyPartDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'body_part) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'body_part)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'openpose_ros_msgs-msg:BodyPartDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PersonDetection>)))
  "Returns string type for a message object of type '<PersonDetection>"
  "openpose_ros_msgs/PersonDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PersonDetection)))
  "Returns string type for a message object of type 'PersonDetection"
  "openpose_ros_msgs/PersonDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PersonDetection>)))
  "Returns md5sum for a message object of type '<PersonDetection>"
  "5cf57469f872bf144c62ed7904d5c159")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PersonDetection)))
  "Returns md5sum for a message object of type 'PersonDetection"
  "5cf57469f872bf144c62ed7904d5c159")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PersonDetection>)))
  "Returns full string definition for message of type '<PersonDetection>"
  (cl:format cl:nil "BodyPartDetection[] face_landmark~%BodyPartDetection[] body_part~%~%================================================================================~%MSG: openpose_ros_msgs/BodyPartDetection~%uint32 part_id~%uint32 x~%uint32 y~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PersonDetection)))
  "Returns full string definition for message of type 'PersonDetection"
  (cl:format cl:nil "BodyPartDetection[] face_landmark~%BodyPartDetection[] body_part~%~%================================================================================~%MSG: openpose_ros_msgs/BodyPartDetection~%uint32 part_id~%uint32 x~%uint32 y~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PersonDetection>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'face_landmark) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'body_part) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PersonDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'PersonDetection
    (cl:cons ':face_landmark (face_landmark msg))
    (cl:cons ':body_part (body_part msg))
))
