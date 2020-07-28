; Auto-generated. Do not edit!


(cl:in-package rail_pose_estimation_msgs-msg)


;//! \htmlinclude Keypoints.msg.html

(cl:defclass <Keypoints> (roslisp-msg-protocol:ros-message)
  ((neck_x
    :reader neck_x
    :initarg :neck_x
    :type cl:float
    :initform 0.0)
   (neck_y
    :reader neck_y
    :initarg :neck_y
    :type cl:float
    :initform 0.0)
   (nose_x
    :reader nose_x
    :initarg :nose_x
    :type cl:float
    :initform 0.0)
   (nose_y
    :reader nose_y
    :initarg :nose_y
    :type cl:float
    :initform 0.0)
   (right_shoulder_x
    :reader right_shoulder_x
    :initarg :right_shoulder_x
    :type cl:float
    :initform 0.0)
   (right_shoulder_y
    :reader right_shoulder_y
    :initarg :right_shoulder_y
    :type cl:float
    :initform 0.0)
   (left_shoulder_x
    :reader left_shoulder_x
    :initarg :left_shoulder_x
    :type cl:float
    :initform 0.0)
   (left_shoulder_y
    :reader left_shoulder_y
    :initarg :left_shoulder_y
    :type cl:float
    :initform 0.0)
   (right_elbow_x
    :reader right_elbow_x
    :initarg :right_elbow_x
    :type cl:float
    :initform 0.0)
   (right_elbow_y
    :reader right_elbow_y
    :initarg :right_elbow_y
    :type cl:float
    :initform 0.0)
   (left_elbow_x
    :reader left_elbow_x
    :initarg :left_elbow_x
    :type cl:float
    :initform 0.0)
   (left_elbow_y
    :reader left_elbow_y
    :initarg :left_elbow_y
    :type cl:float
    :initform 0.0)
   (right_wrist_x
    :reader right_wrist_x
    :initarg :right_wrist_x
    :type cl:float
    :initform 0.0)
   (right_wrist_y
    :reader right_wrist_y
    :initarg :right_wrist_y
    :type cl:float
    :initform 0.0)
   (left_wrist_x
    :reader left_wrist_x
    :initarg :left_wrist_x
    :type cl:float
    :initform 0.0)
   (left_wrist_y
    :reader left_wrist_y
    :initarg :left_wrist_y
    :type cl:float
    :initform 0.0)
   (right_hip_x
    :reader right_hip_x
    :initarg :right_hip_x
    :type cl:float
    :initform 0.0)
   (right_hip_y
    :reader right_hip_y
    :initarg :right_hip_y
    :type cl:float
    :initform 0.0)
   (left_hip_x
    :reader left_hip_x
    :initarg :left_hip_x
    :type cl:float
    :initform 0.0)
   (left_hip_y
    :reader left_hip_y
    :initarg :left_hip_y
    :type cl:float
    :initform 0.0)
   (right_knee_x
    :reader right_knee_x
    :initarg :right_knee_x
    :type cl:float
    :initform 0.0)
   (right_knee_y
    :reader right_knee_y
    :initarg :right_knee_y
    :type cl:float
    :initform 0.0)
   (left_knee_x
    :reader left_knee_x
    :initarg :left_knee_x
    :type cl:float
    :initform 0.0)
   (left_knee_y
    :reader left_knee_y
    :initarg :left_knee_y
    :type cl:float
    :initform 0.0)
   (right_ankle_x
    :reader right_ankle_x
    :initarg :right_ankle_x
    :type cl:float
    :initform 0.0)
   (right_ankle_y
    :reader right_ankle_y
    :initarg :right_ankle_y
    :type cl:float
    :initform 0.0)
   (left_ankle_x
    :reader left_ankle_x
    :initarg :left_ankle_x
    :type cl:float
    :initform 0.0)
   (left_ankle_y
    :reader left_ankle_y
    :initarg :left_ankle_y
    :type cl:float
    :initform 0.0)
   (right_eye_x
    :reader right_eye_x
    :initarg :right_eye_x
    :type cl:float
    :initform 0.0)
   (right_eye_y
    :reader right_eye_y
    :initarg :right_eye_y
    :type cl:float
    :initform 0.0)
   (left_eye_x
    :reader left_eye_x
    :initarg :left_eye_x
    :type cl:float
    :initform 0.0)
   (left_eye_y
    :reader left_eye_y
    :initarg :left_eye_y
    :type cl:float
    :initform 0.0)
   (right_ear_x
    :reader right_ear_x
    :initarg :right_ear_x
    :type cl:float
    :initform 0.0)
   (right_ear_y
    :reader right_ear_y
    :initarg :right_ear_y
    :type cl:float
    :initform 0.0)
   (left_ear_x
    :reader left_ear_x
    :initarg :left_ear_x
    :type cl:float
    :initform 0.0)
   (left_ear_y
    :reader left_ear_y
    :initarg :left_ear_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass Keypoints (<Keypoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Keypoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Keypoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rail_pose_estimation_msgs-msg:<Keypoints> is deprecated: use rail_pose_estimation_msgs-msg:Keypoints instead.")))

(cl:ensure-generic-function 'neck_x-val :lambda-list '(m))
(cl:defmethod neck_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:neck_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:neck_x instead.")
  (neck_x m))

(cl:ensure-generic-function 'neck_y-val :lambda-list '(m))
(cl:defmethod neck_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:neck_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:neck_y instead.")
  (neck_y m))

(cl:ensure-generic-function 'nose_x-val :lambda-list '(m))
(cl:defmethod nose_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:nose_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:nose_x instead.")
  (nose_x m))

(cl:ensure-generic-function 'nose_y-val :lambda-list '(m))
(cl:defmethod nose_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:nose_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:nose_y instead.")
  (nose_y m))

(cl:ensure-generic-function 'right_shoulder_x-val :lambda-list '(m))
(cl:defmethod right_shoulder_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_shoulder_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_shoulder_x instead.")
  (right_shoulder_x m))

(cl:ensure-generic-function 'right_shoulder_y-val :lambda-list '(m))
(cl:defmethod right_shoulder_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_shoulder_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_shoulder_y instead.")
  (right_shoulder_y m))

(cl:ensure-generic-function 'left_shoulder_x-val :lambda-list '(m))
(cl:defmethod left_shoulder_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_shoulder_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_shoulder_x instead.")
  (left_shoulder_x m))

(cl:ensure-generic-function 'left_shoulder_y-val :lambda-list '(m))
(cl:defmethod left_shoulder_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_shoulder_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_shoulder_y instead.")
  (left_shoulder_y m))

(cl:ensure-generic-function 'right_elbow_x-val :lambda-list '(m))
(cl:defmethod right_elbow_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_elbow_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_elbow_x instead.")
  (right_elbow_x m))

(cl:ensure-generic-function 'right_elbow_y-val :lambda-list '(m))
(cl:defmethod right_elbow_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_elbow_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_elbow_y instead.")
  (right_elbow_y m))

(cl:ensure-generic-function 'left_elbow_x-val :lambda-list '(m))
(cl:defmethod left_elbow_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_elbow_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_elbow_x instead.")
  (left_elbow_x m))

(cl:ensure-generic-function 'left_elbow_y-val :lambda-list '(m))
(cl:defmethod left_elbow_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_elbow_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_elbow_y instead.")
  (left_elbow_y m))

(cl:ensure-generic-function 'right_wrist_x-val :lambda-list '(m))
(cl:defmethod right_wrist_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_wrist_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_wrist_x instead.")
  (right_wrist_x m))

(cl:ensure-generic-function 'right_wrist_y-val :lambda-list '(m))
(cl:defmethod right_wrist_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_wrist_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_wrist_y instead.")
  (right_wrist_y m))

(cl:ensure-generic-function 'left_wrist_x-val :lambda-list '(m))
(cl:defmethod left_wrist_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_wrist_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_wrist_x instead.")
  (left_wrist_x m))

(cl:ensure-generic-function 'left_wrist_y-val :lambda-list '(m))
(cl:defmethod left_wrist_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_wrist_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_wrist_y instead.")
  (left_wrist_y m))

(cl:ensure-generic-function 'right_hip_x-val :lambda-list '(m))
(cl:defmethod right_hip_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_hip_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_hip_x instead.")
  (right_hip_x m))

(cl:ensure-generic-function 'right_hip_y-val :lambda-list '(m))
(cl:defmethod right_hip_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_hip_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_hip_y instead.")
  (right_hip_y m))

(cl:ensure-generic-function 'left_hip_x-val :lambda-list '(m))
(cl:defmethod left_hip_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_hip_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_hip_x instead.")
  (left_hip_x m))

(cl:ensure-generic-function 'left_hip_y-val :lambda-list '(m))
(cl:defmethod left_hip_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_hip_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_hip_y instead.")
  (left_hip_y m))

(cl:ensure-generic-function 'right_knee_x-val :lambda-list '(m))
(cl:defmethod right_knee_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_knee_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_knee_x instead.")
  (right_knee_x m))

(cl:ensure-generic-function 'right_knee_y-val :lambda-list '(m))
(cl:defmethod right_knee_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_knee_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_knee_y instead.")
  (right_knee_y m))

(cl:ensure-generic-function 'left_knee_x-val :lambda-list '(m))
(cl:defmethod left_knee_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_knee_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_knee_x instead.")
  (left_knee_x m))

(cl:ensure-generic-function 'left_knee_y-val :lambda-list '(m))
(cl:defmethod left_knee_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_knee_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_knee_y instead.")
  (left_knee_y m))

(cl:ensure-generic-function 'right_ankle_x-val :lambda-list '(m))
(cl:defmethod right_ankle_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_ankle_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_ankle_x instead.")
  (right_ankle_x m))

(cl:ensure-generic-function 'right_ankle_y-val :lambda-list '(m))
(cl:defmethod right_ankle_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_ankle_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_ankle_y instead.")
  (right_ankle_y m))

(cl:ensure-generic-function 'left_ankle_x-val :lambda-list '(m))
(cl:defmethod left_ankle_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_ankle_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_ankle_x instead.")
  (left_ankle_x m))

(cl:ensure-generic-function 'left_ankle_y-val :lambda-list '(m))
(cl:defmethod left_ankle_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_ankle_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_ankle_y instead.")
  (left_ankle_y m))

(cl:ensure-generic-function 'right_eye_x-val :lambda-list '(m))
(cl:defmethod right_eye_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_eye_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_eye_x instead.")
  (right_eye_x m))

(cl:ensure-generic-function 'right_eye_y-val :lambda-list '(m))
(cl:defmethod right_eye_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_eye_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_eye_y instead.")
  (right_eye_y m))

(cl:ensure-generic-function 'left_eye_x-val :lambda-list '(m))
(cl:defmethod left_eye_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_eye_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_eye_x instead.")
  (left_eye_x m))

(cl:ensure-generic-function 'left_eye_y-val :lambda-list '(m))
(cl:defmethod left_eye_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_eye_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_eye_y instead.")
  (left_eye_y m))

(cl:ensure-generic-function 'right_ear_x-val :lambda-list '(m))
(cl:defmethod right_ear_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_ear_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_ear_x instead.")
  (right_ear_x m))

(cl:ensure-generic-function 'right_ear_y-val :lambda-list '(m))
(cl:defmethod right_ear_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:right_ear_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:right_ear_y instead.")
  (right_ear_y m))

(cl:ensure-generic-function 'left_ear_x-val :lambda-list '(m))
(cl:defmethod left_ear_x-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_ear_x-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_ear_x instead.")
  (left_ear_x m))

(cl:ensure-generic-function 'left_ear_y-val :lambda-list '(m))
(cl:defmethod left_ear_y-val ((m <Keypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:left_ear_y-val is deprecated.  Use rail_pose_estimation_msgs-msg:left_ear_y instead.")
  (left_ear_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Keypoints>) ostream)
  "Serializes a message object of type '<Keypoints>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'neck_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'neck_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'nose_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'nose_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_shoulder_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_shoulder_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_shoulder_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_shoulder_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_elbow_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_elbow_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_elbow_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_elbow_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_wrist_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_wrist_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_wrist_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_wrist_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_hip_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_hip_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_hip_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_hip_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_knee_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_knee_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_knee_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_knee_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_ankle_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_ankle_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_ankle_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_ankle_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_eye_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_eye_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_eye_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_eye_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_ear_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_ear_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_ear_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_ear_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Keypoints>) istream)
  "Deserializes a message object of type '<Keypoints>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'neck_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'neck_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'nose_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'nose_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_shoulder_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_shoulder_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_shoulder_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_shoulder_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_elbow_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_elbow_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_elbow_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_elbow_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_wrist_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_wrist_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_wrist_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_wrist_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_hip_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_hip_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_hip_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_hip_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_knee_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_knee_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_knee_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_knee_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_ankle_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_ankle_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_ankle_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_ankle_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_eye_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_eye_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_eye_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_eye_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_ear_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_ear_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_ear_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_ear_y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Keypoints>)))
  "Returns string type for a message object of type '<Keypoints>"
  "rail_pose_estimation_msgs/Keypoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Keypoints)))
  "Returns string type for a message object of type 'Keypoints"
  "rail_pose_estimation_msgs/Keypoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Keypoints>)))
  "Returns md5sum for a message object of type '<Keypoints>"
  "3d1804a99352b413ee0c9ca364640114")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Keypoints)))
  "Returns md5sum for a message object of type 'Keypoints"
  "3d1804a99352b413ee0c9ca364640114")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Keypoints>)))
  "Returns full string definition for message of type '<Keypoints>"
  (cl:format cl:nil "float32 neck_x                  # x coord of neck~%float32 neck_y                  # y coord of neck~%float32 nose_x                  # x coord of nose~%float32 nose_y                  # y coord of nose~%float32 right_shoulder_x        # x coord of right shoulder~%float32 right_shoulder_y        # y coord of right shoulder~%float32 left_shoulder_x         # x coord of left shoulder~%float32 left_shoulder_y         # y coord of left shoulder~%float32 right_elbow_x           # x coord of right elbow~%float32 right_elbow_y           # y coord of right elbow~%float32 left_elbow_x            # x coord of left elbow~%float32 left_elbow_y            # y coord of left elbow~%float32 right_wrist_x           # x coord of right wrist~%float32 right_wrist_y           # y coord of right wrist~%float32 left_wrist_x            # x coord of left wrist~%float32 left_wrist_y            # y coord of left wrist~%float32 right_hip_x             # x coord of right hip~%float32 right_hip_y             # y coord of right hip~%float32 left_hip_x              # x coord of left hip~%float32 left_hip_y              # y coord of left hip~%float32 right_knee_x            # x coord of right knee~%float32 right_knee_y            # y coord of right knee~%float32 left_knee_x             # x coord of left knee~%float32 left_knee_y             # y coord of left knee~%float32 right_ankle_x           # x coord of right ankle~%float32 right_ankle_y           # y coord of right ankle~%float32 left_ankle_x            # x coord of left ankle~%float32 left_ankle_y            # y coord of left ankle~%float32 right_eye_x             # x coord of right eye~%float32 right_eye_y             # y coord of right eye~%float32 left_eye_x              # x coord of left eye~%float32 left_eye_y              # y coord of left eye~%float32 right_ear_x             # x coord of right ear~%float32 right_ear_y             # y coord of right ear~%float32 left_ear_x              # x coord of left ear~%float32 left_ear_y              # y coord of left ear~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Keypoints)))
  "Returns full string definition for message of type 'Keypoints"
  (cl:format cl:nil "float32 neck_x                  # x coord of neck~%float32 neck_y                  # y coord of neck~%float32 nose_x                  # x coord of nose~%float32 nose_y                  # y coord of nose~%float32 right_shoulder_x        # x coord of right shoulder~%float32 right_shoulder_y        # y coord of right shoulder~%float32 left_shoulder_x         # x coord of left shoulder~%float32 left_shoulder_y         # y coord of left shoulder~%float32 right_elbow_x           # x coord of right elbow~%float32 right_elbow_y           # y coord of right elbow~%float32 left_elbow_x            # x coord of left elbow~%float32 left_elbow_y            # y coord of left elbow~%float32 right_wrist_x           # x coord of right wrist~%float32 right_wrist_y           # y coord of right wrist~%float32 left_wrist_x            # x coord of left wrist~%float32 left_wrist_y            # y coord of left wrist~%float32 right_hip_x             # x coord of right hip~%float32 right_hip_y             # y coord of right hip~%float32 left_hip_x              # x coord of left hip~%float32 left_hip_y              # y coord of left hip~%float32 right_knee_x            # x coord of right knee~%float32 right_knee_y            # y coord of right knee~%float32 left_knee_x             # x coord of left knee~%float32 left_knee_y             # y coord of left knee~%float32 right_ankle_x           # x coord of right ankle~%float32 right_ankle_y           # y coord of right ankle~%float32 left_ankle_x            # x coord of left ankle~%float32 left_ankle_y            # y coord of left ankle~%float32 right_eye_x             # x coord of right eye~%float32 right_eye_y             # y coord of right eye~%float32 left_eye_x              # x coord of left eye~%float32 left_eye_y              # y coord of left eye~%float32 right_ear_x             # x coord of right ear~%float32 right_ear_y             # y coord of right ear~%float32 left_ear_x              # x coord of left ear~%float32 left_ear_y              # y coord of left ear~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Keypoints>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Keypoints>))
  "Converts a ROS message object to a list"
  (cl:list 'Keypoints
    (cl:cons ':neck_x (neck_x msg))
    (cl:cons ':neck_y (neck_y msg))
    (cl:cons ':nose_x (nose_x msg))
    (cl:cons ':nose_y (nose_y msg))
    (cl:cons ':right_shoulder_x (right_shoulder_x msg))
    (cl:cons ':right_shoulder_y (right_shoulder_y msg))
    (cl:cons ':left_shoulder_x (left_shoulder_x msg))
    (cl:cons ':left_shoulder_y (left_shoulder_y msg))
    (cl:cons ':right_elbow_x (right_elbow_x msg))
    (cl:cons ':right_elbow_y (right_elbow_y msg))
    (cl:cons ':left_elbow_x (left_elbow_x msg))
    (cl:cons ':left_elbow_y (left_elbow_y msg))
    (cl:cons ':right_wrist_x (right_wrist_x msg))
    (cl:cons ':right_wrist_y (right_wrist_y msg))
    (cl:cons ':left_wrist_x (left_wrist_x msg))
    (cl:cons ':left_wrist_y (left_wrist_y msg))
    (cl:cons ':right_hip_x (right_hip_x msg))
    (cl:cons ':right_hip_y (right_hip_y msg))
    (cl:cons ':left_hip_x (left_hip_x msg))
    (cl:cons ':left_hip_y (left_hip_y msg))
    (cl:cons ':right_knee_x (right_knee_x msg))
    (cl:cons ':right_knee_y (right_knee_y msg))
    (cl:cons ':left_knee_x (left_knee_x msg))
    (cl:cons ':left_knee_y (left_knee_y msg))
    (cl:cons ':right_ankle_x (right_ankle_x msg))
    (cl:cons ':right_ankle_y (right_ankle_y msg))
    (cl:cons ':left_ankle_x (left_ankle_x msg))
    (cl:cons ':left_ankle_y (left_ankle_y msg))
    (cl:cons ':right_eye_x (right_eye_x msg))
    (cl:cons ':right_eye_y (right_eye_y msg))
    (cl:cons ':left_eye_x (left_eye_x msg))
    (cl:cons ':left_eye_y (left_eye_y msg))
    (cl:cons ':right_ear_x (right_ear_x msg))
    (cl:cons ':right_ear_y (right_ear_y msg))
    (cl:cons ':left_ear_x (left_ear_x msg))
    (cl:cons ':left_ear_y (left_ear_y msg))
))
