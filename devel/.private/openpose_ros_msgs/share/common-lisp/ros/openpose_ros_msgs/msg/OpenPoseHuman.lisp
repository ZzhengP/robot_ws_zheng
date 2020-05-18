; Auto-generated. Do not edit!


(cl:in-package openpose_ros_msgs-msg)


;//! \htmlinclude OpenPoseHuman.msg.html

(cl:defclass <OpenPoseHuman> (roslisp-msg-protocol:ros-message)
  ((num_body_key_points_with_non_zero_prob
    :reader num_body_key_points_with_non_zero_prob
    :initarg :num_body_key_points_with_non_zero_prob
    :type cl:integer
    :initform 0)
   (num_face_key_points_with_non_zero_prob
    :reader num_face_key_points_with_non_zero_prob
    :initarg :num_face_key_points_with_non_zero_prob
    :type cl:integer
    :initform 0)
   (num_right_hand_key_points_with_non_zero_prob
    :reader num_right_hand_key_points_with_non_zero_prob
    :initarg :num_right_hand_key_points_with_non_zero_prob
    :type cl:integer
    :initform 0)
   (num_left_hand_key_points_with_non_zero_prob
    :reader num_left_hand_key_points_with_non_zero_prob
    :initarg :num_left_hand_key_points_with_non_zero_prob
    :type cl:integer
    :initform 0)
   (body_bounding_box
    :reader body_bounding_box
    :initarg :body_bounding_box
    :type openpose_ros_msgs-msg:BoundingBox
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BoundingBox))
   (face_bounding_box
    :reader face_bounding_box
    :initarg :face_bounding_box
    :type openpose_ros_msgs-msg:BoundingBox
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BoundingBox))
   (body_key_points_with_prob
    :reader body_key_points_with_prob
    :initarg :body_key_points_with_prob
    :type (cl:vector openpose_ros_msgs-msg:PointWithProb)
   :initform (cl:make-array 25 :element-type 'openpose_ros_msgs-msg:PointWithProb :initial-element (cl:make-instance 'openpose_ros_msgs-msg:PointWithProb)))
   (face_key_points_with_prob
    :reader face_key_points_with_prob
    :initarg :face_key_points_with_prob
    :type (cl:vector openpose_ros_msgs-msg:PointWithProb)
   :initform (cl:make-array 70 :element-type 'openpose_ros_msgs-msg:PointWithProb :initial-element (cl:make-instance 'openpose_ros_msgs-msg:PointWithProb)))
   (right_hand_key_points_with_prob
    :reader right_hand_key_points_with_prob
    :initarg :right_hand_key_points_with_prob
    :type (cl:vector openpose_ros_msgs-msg:PointWithProb)
   :initform (cl:make-array 21 :element-type 'openpose_ros_msgs-msg:PointWithProb :initial-element (cl:make-instance 'openpose_ros_msgs-msg:PointWithProb)))
   (left_hand_key_points_with_prob
    :reader left_hand_key_points_with_prob
    :initarg :left_hand_key_points_with_prob
    :type (cl:vector openpose_ros_msgs-msg:PointWithProb)
   :initform (cl:make-array 21 :element-type 'openpose_ros_msgs-msg:PointWithProb :initial-element (cl:make-instance 'openpose_ros_msgs-msg:PointWithProb))))
)

(cl:defclass OpenPoseHuman (<OpenPoseHuman>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OpenPoseHuman>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OpenPoseHuman)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name openpose_ros_msgs-msg:<OpenPoseHuman> is deprecated: use openpose_ros_msgs-msg:OpenPoseHuman instead.")))

(cl:ensure-generic-function 'num_body_key_points_with_non_zero_prob-val :lambda-list '(m))
(cl:defmethod num_body_key_points_with_non_zero_prob-val ((m <OpenPoseHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:num_body_key_points_with_non_zero_prob-val is deprecated.  Use openpose_ros_msgs-msg:num_body_key_points_with_non_zero_prob instead.")
  (num_body_key_points_with_non_zero_prob m))

(cl:ensure-generic-function 'num_face_key_points_with_non_zero_prob-val :lambda-list '(m))
(cl:defmethod num_face_key_points_with_non_zero_prob-val ((m <OpenPoseHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:num_face_key_points_with_non_zero_prob-val is deprecated.  Use openpose_ros_msgs-msg:num_face_key_points_with_non_zero_prob instead.")
  (num_face_key_points_with_non_zero_prob m))

(cl:ensure-generic-function 'num_right_hand_key_points_with_non_zero_prob-val :lambda-list '(m))
(cl:defmethod num_right_hand_key_points_with_non_zero_prob-val ((m <OpenPoseHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:num_right_hand_key_points_with_non_zero_prob-val is deprecated.  Use openpose_ros_msgs-msg:num_right_hand_key_points_with_non_zero_prob instead.")
  (num_right_hand_key_points_with_non_zero_prob m))

(cl:ensure-generic-function 'num_left_hand_key_points_with_non_zero_prob-val :lambda-list '(m))
(cl:defmethod num_left_hand_key_points_with_non_zero_prob-val ((m <OpenPoseHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:num_left_hand_key_points_with_non_zero_prob-val is deprecated.  Use openpose_ros_msgs-msg:num_left_hand_key_points_with_non_zero_prob instead.")
  (num_left_hand_key_points_with_non_zero_prob m))

(cl:ensure-generic-function 'body_bounding_box-val :lambda-list '(m))
(cl:defmethod body_bounding_box-val ((m <OpenPoseHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:body_bounding_box-val is deprecated.  Use openpose_ros_msgs-msg:body_bounding_box instead.")
  (body_bounding_box m))

(cl:ensure-generic-function 'face_bounding_box-val :lambda-list '(m))
(cl:defmethod face_bounding_box-val ((m <OpenPoseHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:face_bounding_box-val is deprecated.  Use openpose_ros_msgs-msg:face_bounding_box instead.")
  (face_bounding_box m))

(cl:ensure-generic-function 'body_key_points_with_prob-val :lambda-list '(m))
(cl:defmethod body_key_points_with_prob-val ((m <OpenPoseHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:body_key_points_with_prob-val is deprecated.  Use openpose_ros_msgs-msg:body_key_points_with_prob instead.")
  (body_key_points_with_prob m))

(cl:ensure-generic-function 'face_key_points_with_prob-val :lambda-list '(m))
(cl:defmethod face_key_points_with_prob-val ((m <OpenPoseHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:face_key_points_with_prob-val is deprecated.  Use openpose_ros_msgs-msg:face_key_points_with_prob instead.")
  (face_key_points_with_prob m))

(cl:ensure-generic-function 'right_hand_key_points_with_prob-val :lambda-list '(m))
(cl:defmethod right_hand_key_points_with_prob-val ((m <OpenPoseHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:right_hand_key_points_with_prob-val is deprecated.  Use openpose_ros_msgs-msg:right_hand_key_points_with_prob instead.")
  (right_hand_key_points_with_prob m))

(cl:ensure-generic-function 'left_hand_key_points_with_prob-val :lambda-list '(m))
(cl:defmethod left_hand_key_points_with_prob-val ((m <OpenPoseHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:left_hand_key_points_with_prob-val is deprecated.  Use openpose_ros_msgs-msg:left_hand_key_points_with_prob instead.")
  (left_hand_key_points_with_prob m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OpenPoseHuman>) ostream)
  "Serializes a message object of type '<OpenPoseHuman>"
  (cl:let* ((signed (cl:slot-value msg 'num_body_key_points_with_non_zero_prob)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num_face_key_points_with_non_zero_prob)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num_right_hand_key_points_with_non_zero_prob)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num_left_hand_key_points_with_non_zero_prob)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'body_bounding_box) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'face_bounding_box) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'body_key_points_with_prob))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'face_key_points_with_prob))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'right_hand_key_points_with_prob))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'left_hand_key_points_with_prob))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OpenPoseHuman>) istream)
  "Deserializes a message object of type '<OpenPoseHuman>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_body_key_points_with_non_zero_prob) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_face_key_points_with_non_zero_prob) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_right_hand_key_points_with_non_zero_prob) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_left_hand_key_points_with_non_zero_prob) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'body_bounding_box) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'face_bounding_box) istream)
  (cl:setf (cl:slot-value msg 'body_key_points_with_prob) (cl:make-array 25))
  (cl:let ((vals (cl:slot-value msg 'body_key_points_with_prob)))
    (cl:dotimes (i 25)
    (cl:setf (cl:aref vals i) (cl:make-instance 'openpose_ros_msgs-msg:PointWithProb))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'face_key_points_with_prob) (cl:make-array 70))
  (cl:let ((vals (cl:slot-value msg 'face_key_points_with_prob)))
    (cl:dotimes (i 70)
    (cl:setf (cl:aref vals i) (cl:make-instance 'openpose_ros_msgs-msg:PointWithProb))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'right_hand_key_points_with_prob) (cl:make-array 21))
  (cl:let ((vals (cl:slot-value msg 'right_hand_key_points_with_prob)))
    (cl:dotimes (i 21)
    (cl:setf (cl:aref vals i) (cl:make-instance 'openpose_ros_msgs-msg:PointWithProb))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'left_hand_key_points_with_prob) (cl:make-array 21))
  (cl:let ((vals (cl:slot-value msg 'left_hand_key_points_with_prob)))
    (cl:dotimes (i 21)
    (cl:setf (cl:aref vals i) (cl:make-instance 'openpose_ros_msgs-msg:PointWithProb))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OpenPoseHuman>)))
  "Returns string type for a message object of type '<OpenPoseHuman>"
  "openpose_ros_msgs/OpenPoseHuman")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OpenPoseHuman)))
  "Returns string type for a message object of type 'OpenPoseHuman"
  "openpose_ros_msgs/OpenPoseHuman")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OpenPoseHuman>)))
  "Returns md5sum for a message object of type '<OpenPoseHuman>"
  "4e291ca1063e4a96ca852f33fa35d22b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OpenPoseHuman)))
  "Returns md5sum for a message object of type 'OpenPoseHuman"
  "4e291ca1063e4a96ca852f33fa35d22b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OpenPoseHuman>)))
  "Returns full string definition for message of type '<OpenPoseHuman>"
  (cl:format cl:nil "int32 num_body_key_points_with_non_zero_prob~%int32 num_face_key_points_with_non_zero_prob~%int32 num_right_hand_key_points_with_non_zero_prob~%int32 num_left_hand_key_points_with_non_zero_prob~%BoundingBox body_bounding_box~%BoundingBox face_bounding_box~%PointWithProb[25] body_key_points_with_prob~%PointWithProb[70] face_key_points_with_prob~%PointWithProb[21] right_hand_key_points_with_prob~%PointWithProb[21] left_hand_key_points_with_prob~%================================================================================~%MSG: openpose_ros_msgs/BoundingBox~%float64 x~%float64 y~%float64 width~%float64 height~%================================================================================~%MSG: openpose_ros_msgs/PointWithProb~%float64 x~%float64 y~%float64 prob~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OpenPoseHuman)))
  "Returns full string definition for message of type 'OpenPoseHuman"
  (cl:format cl:nil "int32 num_body_key_points_with_non_zero_prob~%int32 num_face_key_points_with_non_zero_prob~%int32 num_right_hand_key_points_with_non_zero_prob~%int32 num_left_hand_key_points_with_non_zero_prob~%BoundingBox body_bounding_box~%BoundingBox face_bounding_box~%PointWithProb[25] body_key_points_with_prob~%PointWithProb[70] face_key_points_with_prob~%PointWithProb[21] right_hand_key_points_with_prob~%PointWithProb[21] left_hand_key_points_with_prob~%================================================================================~%MSG: openpose_ros_msgs/BoundingBox~%float64 x~%float64 y~%float64 width~%float64 height~%================================================================================~%MSG: openpose_ros_msgs/PointWithProb~%float64 x~%float64 y~%float64 prob~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OpenPoseHuman>))
  (cl:+ 0
     4
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'body_bounding_box))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'face_bounding_box))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'body_key_points_with_prob) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'face_key_points_with_prob) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'right_hand_key_points_with_prob) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'left_hand_key_points_with_prob) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OpenPoseHuman>))
  "Converts a ROS message object to a list"
  (cl:list 'OpenPoseHuman
    (cl:cons ':num_body_key_points_with_non_zero_prob (num_body_key_points_with_non_zero_prob msg))
    (cl:cons ':num_face_key_points_with_non_zero_prob (num_face_key_points_with_non_zero_prob msg))
    (cl:cons ':num_right_hand_key_points_with_non_zero_prob (num_right_hand_key_points_with_non_zero_prob msg))
    (cl:cons ':num_left_hand_key_points_with_non_zero_prob (num_left_hand_key_points_with_non_zero_prob msg))
    (cl:cons ':body_bounding_box (body_bounding_box msg))
    (cl:cons ':face_bounding_box (face_bounding_box msg))
    (cl:cons ':body_key_points_with_prob (body_key_points_with_prob msg))
    (cl:cons ':face_key_points_with_prob (face_key_points_with_prob msg))
    (cl:cons ':right_hand_key_points_with_prob (right_hand_key_points_with_prob msg))
    (cl:cons ':left_hand_key_points_with_prob (left_hand_key_points_with_prob msg))
))
