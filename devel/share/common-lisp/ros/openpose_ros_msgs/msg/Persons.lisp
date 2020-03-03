; Auto-generated. Do not edit!


(cl:in-package openpose_ros_msgs-msg)


;//! \htmlinclude Persons.msg.html

(cl:defclass <Persons> (roslisp-msg-protocol:ros-message)
  ((persons
    :reader persons
    :initarg :persons
    :type (cl:vector openpose_ros_msgs-msg:PersonDetection)
   :initform (cl:make-array 0 :element-type 'openpose_ros_msgs-msg:PersonDetection :initial-element (cl:make-instance 'openpose_ros_msgs-msg:PersonDetection)))
   (rostime
    :reader rostime
    :initarg :rostime
    :type cl:real
    :initform 0)
   (image_w
    :reader image_w
    :initarg :image_w
    :type cl:integer
    :initform 0)
   (image_h
    :reader image_h
    :initarg :image_h
    :type cl:integer
    :initform 0))
)

(cl:defclass Persons (<Persons>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Persons>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Persons)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name openpose_ros_msgs-msg:<Persons> is deprecated: use openpose_ros_msgs-msg:Persons instead.")))

(cl:ensure-generic-function 'persons-val :lambda-list '(m))
(cl:defmethod persons-val ((m <Persons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:persons-val is deprecated.  Use openpose_ros_msgs-msg:persons instead.")
  (persons m))

(cl:ensure-generic-function 'rostime-val :lambda-list '(m))
(cl:defmethod rostime-val ((m <Persons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:rostime-val is deprecated.  Use openpose_ros_msgs-msg:rostime instead.")
  (rostime m))

(cl:ensure-generic-function 'image_w-val :lambda-list '(m))
(cl:defmethod image_w-val ((m <Persons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:image_w-val is deprecated.  Use openpose_ros_msgs-msg:image_w instead.")
  (image_w m))

(cl:ensure-generic-function 'image_h-val :lambda-list '(m))
(cl:defmethod image_h-val ((m <Persons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:image_h-val is deprecated.  Use openpose_ros_msgs-msg:image_h instead.")
  (image_h m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Persons>) ostream)
  "Serializes a message object of type '<Persons>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'persons))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'persons))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'rostime)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'rostime) (cl:floor (cl:slot-value msg 'rostime)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_w)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_w)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_w)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_w)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_h)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_h)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_h)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_h)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Persons>) istream)
  "Deserializes a message object of type '<Persons>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'persons) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'persons)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'openpose_ros_msgs-msg:PersonDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rostime) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_w)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_w)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_w)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_w)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_h)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_h)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_h)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_h)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Persons>)))
  "Returns string type for a message object of type '<Persons>"
  "openpose_ros_msgs/Persons")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Persons)))
  "Returns string type for a message object of type 'Persons"
  "openpose_ros_msgs/Persons")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Persons>)))
  "Returns md5sum for a message object of type '<Persons>"
  "364e552085331e28feea9ffef91f2929")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Persons)))
  "Returns md5sum for a message object of type 'Persons"
  "364e552085331e28feea9ffef91f2929")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Persons>)))
  "Returns full string definition for message of type '<Persons>"
  (cl:format cl:nil "PersonDetection[] persons~%time rostime~%uint32 image_w~%uint32 image_h~%~%================================================================================~%MSG: openpose_ros_msgs/PersonDetection~%BodyPartDetection[] face_landmark~%BodyPartDetection[] body_part~%~%================================================================================~%MSG: openpose_ros_msgs/BodyPartDetection~%uint32 part_id~%uint32 x~%uint32 y~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Persons)))
  "Returns full string definition for message of type 'Persons"
  (cl:format cl:nil "PersonDetection[] persons~%time rostime~%uint32 image_w~%uint32 image_h~%~%================================================================================~%MSG: openpose_ros_msgs/PersonDetection~%BodyPartDetection[] face_landmark~%BodyPartDetection[] body_part~%~%================================================================================~%MSG: openpose_ros_msgs/BodyPartDetection~%uint32 part_id~%uint32 x~%uint32 y~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Persons>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'persons) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     8
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Persons>))
  "Converts a ROS message object to a list"
  (cl:list 'Persons
    (cl:cons ':persons (persons msg))
    (cl:cons ':rostime (rostime msg))
    (cl:cons ':image_w (image_w msg))
    (cl:cons ':image_h (image_h msg))
))
