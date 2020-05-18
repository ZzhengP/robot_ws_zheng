; Auto-generated. Do not edit!


(cl:in-package openpose_ros_msgs-msg)


;//! \htmlinclude BodypartDetection.msg.html

(cl:defclass <BodypartDetection> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass BodypartDetection (<BodypartDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BodypartDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BodypartDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name openpose_ros_msgs-msg:<BodypartDetection> is deprecated: use openpose_ros_msgs-msg:BodypartDetection instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <BodypartDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:x-val is deprecated.  Use openpose_ros_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <BodypartDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:y-val is deprecated.  Use openpose_ros_msgs-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <BodypartDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:confidence-val is deprecated.  Use openpose_ros_msgs-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BodypartDetection>) ostream)
  "Serializes a message object of type '<BodypartDetection>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'y)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BodypartDetection>) istream)
  "Deserializes a message object of type '<BodypartDetection>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BodypartDetection>)))
  "Returns string type for a message object of type '<BodypartDetection>"
  "openpose_ros_msgs/BodypartDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BodypartDetection)))
  "Returns string type for a message object of type 'BodypartDetection"
  "openpose_ros_msgs/BodypartDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BodypartDetection>)))
  "Returns md5sum for a message object of type '<BodypartDetection>"
  "2edd10e314180ad374d9e70737f3fc60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BodypartDetection)))
  "Returns md5sum for a message object of type 'BodypartDetection"
  "2edd10e314180ad374d9e70737f3fc60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BodypartDetection>)))
  "Returns full string definition for message of type '<BodypartDetection>"
  (cl:format cl:nil "uint32 x~%uint32 y~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BodypartDetection)))
  "Returns full string definition for message of type 'BodypartDetection"
  (cl:format cl:nil "uint32 x~%uint32 y~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BodypartDetection>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BodypartDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'BodypartDetection
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':confidence (confidence msg))
))
