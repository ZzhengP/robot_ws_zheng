; Auto-generated. Do not edit!


(cl:in-package rail_pose_estimation_msgs-msg)


;//! \htmlinclude Poses.msg.html

(cl:defclass <Poses> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (people
    :reader people
    :initarg :people
    :type (cl:vector rail_pose_estimation_msgs-msg:Keypoints)
   :initform (cl:make-array 0 :element-type 'rail_pose_estimation_msgs-msg:Keypoints :initial-element (cl:make-instance 'rail_pose_estimation_msgs-msg:Keypoints))))
)

(cl:defclass Poses (<Poses>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Poses>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Poses)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rail_pose_estimation_msgs-msg:<Poses> is deprecated: use rail_pose_estimation_msgs-msg:Poses instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Poses>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:header-val is deprecated.  Use rail_pose_estimation_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'people-val :lambda-list '(m))
(cl:defmethod people-val ((m <Poses>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rail_pose_estimation_msgs-msg:people-val is deprecated.  Use rail_pose_estimation_msgs-msg:people instead.")
  (people m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Poses>) ostream)
  "Serializes a message object of type '<Poses>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'people))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'people))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Poses>) istream)
  "Deserializes a message object of type '<Poses>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'people) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'people)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'rail_pose_estimation_msgs-msg:Keypoints))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Poses>)))
  "Returns string type for a message object of type '<Poses>"
  "rail_pose_estimation_msgs/Poses")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Poses)))
  "Returns string type for a message object of type 'Poses"
  "rail_pose_estimation_msgs/Poses")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Poses>)))
  "Returns md5sum for a message object of type '<Poses>"
  "f3ac5a5326099357214a8296eb96e5d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Poses)))
  "Returns md5sum for a message object of type 'Poses"
  "f3ac5a5326099357214a8296eb96e5d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Poses>)))
  "Returns full string definition for message of type '<Poses>"
  (cl:format cl:nil "std_msgs/Header header 	# Message header including timestamp of detection & img frame~%Keypoints[] people 			# Array of detected objects~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: rail_pose_estimation_msgs/Keypoints~%float32 neck_x                  # x coord of neck~%float32 neck_y                  # y coord of neck~%float32 nose_x                  # x coord of nose~%float32 nose_y                  # y coord of nose~%float32 right_shoulder_x        # x coord of right shoulder~%float32 right_shoulder_y        # y coord of right shoulder~%float32 left_shoulder_x         # x coord of left shoulder~%float32 left_shoulder_y         # y coord of left shoulder~%float32 right_elbow_x           # x coord of right elbow~%float32 right_elbow_y           # y coord of right elbow~%float32 left_elbow_x            # x coord of left elbow~%float32 left_elbow_y            # y coord of left elbow~%float32 right_wrist_x           # x coord of right wrist~%float32 right_wrist_y           # y coord of right wrist~%float32 left_wrist_x            # x coord of left wrist~%float32 left_wrist_y            # y coord of left wrist~%float32 right_hip_x             # x coord of right hip~%float32 right_hip_y             # y coord of right hip~%float32 left_hip_x              # x coord of left hip~%float32 left_hip_y              # y coord of left hip~%float32 right_knee_x            # x coord of right knee~%float32 right_knee_y            # y coord of right knee~%float32 left_knee_x             # x coord of left knee~%float32 left_knee_y             # y coord of left knee~%float32 right_ankle_x           # x coord of right ankle~%float32 right_ankle_y           # y coord of right ankle~%float32 left_ankle_x            # x coord of left ankle~%float32 left_ankle_y            # y coord of left ankle~%float32 right_eye_x             # x coord of right eye~%float32 right_eye_y             # y coord of right eye~%float32 left_eye_x              # x coord of left eye~%float32 left_eye_y              # y coord of left eye~%float32 right_ear_x             # x coord of right ear~%float32 right_ear_y             # y coord of right ear~%float32 left_ear_x              # x coord of left ear~%float32 left_ear_y              # y coord of left ear~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Poses)))
  "Returns full string definition for message of type 'Poses"
  (cl:format cl:nil "std_msgs/Header header 	# Message header including timestamp of detection & img frame~%Keypoints[] people 			# Array of detected objects~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: rail_pose_estimation_msgs/Keypoints~%float32 neck_x                  # x coord of neck~%float32 neck_y                  # y coord of neck~%float32 nose_x                  # x coord of nose~%float32 nose_y                  # y coord of nose~%float32 right_shoulder_x        # x coord of right shoulder~%float32 right_shoulder_y        # y coord of right shoulder~%float32 left_shoulder_x         # x coord of left shoulder~%float32 left_shoulder_y         # y coord of left shoulder~%float32 right_elbow_x           # x coord of right elbow~%float32 right_elbow_y           # y coord of right elbow~%float32 left_elbow_x            # x coord of left elbow~%float32 left_elbow_y            # y coord of left elbow~%float32 right_wrist_x           # x coord of right wrist~%float32 right_wrist_y           # y coord of right wrist~%float32 left_wrist_x            # x coord of left wrist~%float32 left_wrist_y            # y coord of left wrist~%float32 right_hip_x             # x coord of right hip~%float32 right_hip_y             # y coord of right hip~%float32 left_hip_x              # x coord of left hip~%float32 left_hip_y              # y coord of left hip~%float32 right_knee_x            # x coord of right knee~%float32 right_knee_y            # y coord of right knee~%float32 left_knee_x             # x coord of left knee~%float32 left_knee_y             # y coord of left knee~%float32 right_ankle_x           # x coord of right ankle~%float32 right_ankle_y           # y coord of right ankle~%float32 left_ankle_x            # x coord of left ankle~%float32 left_ankle_y            # y coord of left ankle~%float32 right_eye_x             # x coord of right eye~%float32 right_eye_y             # y coord of right eye~%float32 left_eye_x              # x coord of left eye~%float32 left_eye_y              # y coord of left eye~%float32 right_ear_x             # x coord of right ear~%float32 right_ear_y             # y coord of right ear~%float32 left_ear_x              # x coord of left ear~%float32 left_ear_y              # y coord of left ear~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Poses>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'people) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Poses>))
  "Converts a ROS message object to a list"
  (cl:list 'Poses
    (cl:cons ':header (header msg))
    (cl:cons ':people (people msg))
))
