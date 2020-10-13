; Auto-generated. Do not edit!


(cl:in-package openpose_ros_msgs-msg)


;//! \htmlinclude PersonDetection.msg.html

(cl:defclass <PersonDetection> (roslisp-msg-protocol:ros-message)
  ((num_people_detected
    :reader num_people_detected
    :initarg :num_people_detected
    :type cl:integer
    :initform 0)
   (person_ID
    :reader person_ID
    :initarg :person_ID
    :type cl:integer
    :initform 0)
   (nose
    :reader nose
    :initarg :nose
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (neck
    :reader neck
    :initarg :neck
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (right_shoulder
    :reader right_shoulder
    :initarg :right_shoulder
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (right_elbow
    :reader right_elbow
    :initarg :right_elbow
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (right_wrist
    :reader right_wrist
    :initarg :right_wrist
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (left_shoulder
    :reader left_shoulder
    :initarg :left_shoulder
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (left_elbow
    :reader left_elbow
    :initarg :left_elbow
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (left_wrist
    :reader left_wrist
    :initarg :left_wrist
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (right_hip
    :reader right_hip
    :initarg :right_hip
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (right_knee
    :reader right_knee
    :initarg :right_knee
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (right_ankle
    :reader right_ankle
    :initarg :right_ankle
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (left_hip
    :reader left_hip
    :initarg :left_hip
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (left_knee
    :reader left_knee
    :initarg :left_knee
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (left_ankle
    :reader left_ankle
    :initarg :left_ankle
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (right_eye
    :reader right_eye
    :initarg :right_eye
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (left_eye
    :reader left_eye
    :initarg :left_eye
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (right_ear
    :reader right_ear
    :initarg :right_ear
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (left_ear
    :reader left_ear
    :initarg :left_ear
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection))
   (chest
    :reader chest
    :initarg :chest
    :type openpose_ros_msgs-msg:BodypartDetection
    :initform (cl:make-instance 'openpose_ros_msgs-msg:BodypartDetection)))
)

(cl:defclass PersonDetection (<PersonDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PersonDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PersonDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name openpose_ros_msgs-msg:<PersonDetection> is deprecated: use openpose_ros_msgs-msg:PersonDetection instead.")))

(cl:ensure-generic-function 'num_people_detected-val :lambda-list '(m))
(cl:defmethod num_people_detected-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:num_people_detected-val is deprecated.  Use openpose_ros_msgs-msg:num_people_detected instead.")
  (num_people_detected m))

(cl:ensure-generic-function 'person_ID-val :lambda-list '(m))
(cl:defmethod person_ID-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:person_ID-val is deprecated.  Use openpose_ros_msgs-msg:person_ID instead.")
  (person_ID m))

(cl:ensure-generic-function 'nose-val :lambda-list '(m))
(cl:defmethod nose-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:nose-val is deprecated.  Use openpose_ros_msgs-msg:nose instead.")
  (nose m))

(cl:ensure-generic-function 'neck-val :lambda-list '(m))
(cl:defmethod neck-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:neck-val is deprecated.  Use openpose_ros_msgs-msg:neck instead.")
  (neck m))

(cl:ensure-generic-function 'right_shoulder-val :lambda-list '(m))
(cl:defmethod right_shoulder-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:right_shoulder-val is deprecated.  Use openpose_ros_msgs-msg:right_shoulder instead.")
  (right_shoulder m))

(cl:ensure-generic-function 'right_elbow-val :lambda-list '(m))
(cl:defmethod right_elbow-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:right_elbow-val is deprecated.  Use openpose_ros_msgs-msg:right_elbow instead.")
  (right_elbow m))

(cl:ensure-generic-function 'right_wrist-val :lambda-list '(m))
(cl:defmethod right_wrist-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:right_wrist-val is deprecated.  Use openpose_ros_msgs-msg:right_wrist instead.")
  (right_wrist m))

(cl:ensure-generic-function 'left_shoulder-val :lambda-list '(m))
(cl:defmethod left_shoulder-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:left_shoulder-val is deprecated.  Use openpose_ros_msgs-msg:left_shoulder instead.")
  (left_shoulder m))

(cl:ensure-generic-function 'left_elbow-val :lambda-list '(m))
(cl:defmethod left_elbow-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:left_elbow-val is deprecated.  Use openpose_ros_msgs-msg:left_elbow instead.")
  (left_elbow m))

(cl:ensure-generic-function 'left_wrist-val :lambda-list '(m))
(cl:defmethod left_wrist-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:left_wrist-val is deprecated.  Use openpose_ros_msgs-msg:left_wrist instead.")
  (left_wrist m))

(cl:ensure-generic-function 'right_hip-val :lambda-list '(m))
(cl:defmethod right_hip-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:right_hip-val is deprecated.  Use openpose_ros_msgs-msg:right_hip instead.")
  (right_hip m))

(cl:ensure-generic-function 'right_knee-val :lambda-list '(m))
(cl:defmethod right_knee-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:right_knee-val is deprecated.  Use openpose_ros_msgs-msg:right_knee instead.")
  (right_knee m))

(cl:ensure-generic-function 'right_ankle-val :lambda-list '(m))
(cl:defmethod right_ankle-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:right_ankle-val is deprecated.  Use openpose_ros_msgs-msg:right_ankle instead.")
  (right_ankle m))

(cl:ensure-generic-function 'left_hip-val :lambda-list '(m))
(cl:defmethod left_hip-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:left_hip-val is deprecated.  Use openpose_ros_msgs-msg:left_hip instead.")
  (left_hip m))

(cl:ensure-generic-function 'left_knee-val :lambda-list '(m))
(cl:defmethod left_knee-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:left_knee-val is deprecated.  Use openpose_ros_msgs-msg:left_knee instead.")
  (left_knee m))

(cl:ensure-generic-function 'left_ankle-val :lambda-list '(m))
(cl:defmethod left_ankle-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:left_ankle-val is deprecated.  Use openpose_ros_msgs-msg:left_ankle instead.")
  (left_ankle m))

(cl:ensure-generic-function 'right_eye-val :lambda-list '(m))
(cl:defmethod right_eye-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:right_eye-val is deprecated.  Use openpose_ros_msgs-msg:right_eye instead.")
  (right_eye m))

(cl:ensure-generic-function 'left_eye-val :lambda-list '(m))
(cl:defmethod left_eye-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:left_eye-val is deprecated.  Use openpose_ros_msgs-msg:left_eye instead.")
  (left_eye m))

(cl:ensure-generic-function 'right_ear-val :lambda-list '(m))
(cl:defmethod right_ear-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:right_ear-val is deprecated.  Use openpose_ros_msgs-msg:right_ear instead.")
  (right_ear m))

(cl:ensure-generic-function 'left_ear-val :lambda-list '(m))
(cl:defmethod left_ear-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:left_ear-val is deprecated.  Use openpose_ros_msgs-msg:left_ear instead.")
  (left_ear m))

(cl:ensure-generic-function 'chest-val :lambda-list '(m))
(cl:defmethod chest-val ((m <PersonDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openpose_ros_msgs-msg:chest-val is deprecated.  Use openpose_ros_msgs-msg:chest instead.")
  (chest m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PersonDetection>) ostream)
  "Serializes a message object of type '<PersonDetection>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_people_detected)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_people_detected)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_people_detected)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_people_detected)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'person_ID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'person_ID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'person_ID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'person_ID)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'nose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'neck) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'right_shoulder) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'right_elbow) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'right_wrist) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'left_shoulder) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'left_elbow) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'left_wrist) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'right_hip) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'right_knee) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'right_ankle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'left_hip) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'left_knee) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'left_ankle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'right_eye) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'left_eye) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'right_ear) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'left_ear) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'chest) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PersonDetection>) istream)
  "Deserializes a message object of type '<PersonDetection>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_people_detected)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_people_detected)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_people_detected)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_people_detected)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'person_ID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'person_ID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'person_ID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'person_ID)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'nose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'neck) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'right_shoulder) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'right_elbow) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'right_wrist) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'left_shoulder) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'left_elbow) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'left_wrist) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'right_hip) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'right_knee) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'right_ankle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'left_hip) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'left_knee) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'left_ankle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'right_eye) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'left_eye) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'right_ear) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'left_ear) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'chest) istream)
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
  "d01c2d124a49b2cef7015366ea4e0099")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PersonDetection)))
  "Returns md5sum for a message object of type 'PersonDetection"
  "d01c2d124a49b2cef7015366ea4e0099")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PersonDetection>)))
  "Returns full string definition for message of type '<PersonDetection>"
  (cl:format cl:nil "uint32 num_people_detected~%uint32 person_ID~%BodypartDetection nose~%BodypartDetection neck~%BodypartDetection right_shoulder~%BodypartDetection right_elbow~%BodypartDetection right_wrist~%BodypartDetection left_shoulder~%BodypartDetection left_elbow~%BodypartDetection left_wrist~%BodypartDetection right_hip~%BodypartDetection right_knee~%BodypartDetection right_ankle~%BodypartDetection left_hip~%BodypartDetection left_knee~%BodypartDetection left_ankle~%BodypartDetection right_eye~%BodypartDetection left_eye~%BodypartDetection right_ear~%BodypartDetection left_ear~%BodypartDetection chest~%~%================================================================================~%MSG: openpose_ros_msgs/BodypartDetection~%uint32 x~%uint32 y~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PersonDetection)))
  "Returns full string definition for message of type 'PersonDetection"
  (cl:format cl:nil "uint32 num_people_detected~%uint32 person_ID~%BodypartDetection nose~%BodypartDetection neck~%BodypartDetection right_shoulder~%BodypartDetection right_elbow~%BodypartDetection right_wrist~%BodypartDetection left_shoulder~%BodypartDetection left_elbow~%BodypartDetection left_wrist~%BodypartDetection right_hip~%BodypartDetection right_knee~%BodypartDetection right_ankle~%BodypartDetection left_hip~%BodypartDetection left_knee~%BodypartDetection left_ankle~%BodypartDetection right_eye~%BodypartDetection left_eye~%BodypartDetection right_ear~%BodypartDetection left_ear~%BodypartDetection chest~%~%================================================================================~%MSG: openpose_ros_msgs/BodypartDetection~%uint32 x~%uint32 y~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PersonDetection>))
  (cl:+ 0
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'nose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'neck))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'right_shoulder))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'right_elbow))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'right_wrist))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'left_shoulder))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'left_elbow))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'left_wrist))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'right_hip))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'right_knee))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'right_ankle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'left_hip))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'left_knee))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'left_ankle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'right_eye))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'left_eye))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'right_ear))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'left_ear))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'chest))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PersonDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'PersonDetection
    (cl:cons ':num_people_detected (num_people_detected msg))
    (cl:cons ':person_ID (person_ID msg))
    (cl:cons ':nose (nose msg))
    (cl:cons ':neck (neck msg))
    (cl:cons ':right_shoulder (right_shoulder msg))
    (cl:cons ':right_elbow (right_elbow msg))
    (cl:cons ':right_wrist (right_wrist msg))
    (cl:cons ':left_shoulder (left_shoulder msg))
    (cl:cons ':left_elbow (left_elbow msg))
    (cl:cons ':left_wrist (left_wrist msg))
    (cl:cons ':right_hip (right_hip msg))
    (cl:cons ':right_knee (right_knee msg))
    (cl:cons ':right_ankle (right_ankle msg))
    (cl:cons ':left_hip (left_hip msg))
    (cl:cons ':left_knee (left_knee msg))
    (cl:cons ':left_ankle (left_ankle msg))
    (cl:cons ':right_eye (right_eye msg))
    (cl:cons ':left_eye (left_eye msg))
    (cl:cons ':right_ear (right_ear msg))
    (cl:cons ':left_ear (left_ear msg))
    (cl:cons ':chest (chest msg))
))
