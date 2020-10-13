;; Auto-generated. Do not edit!


(when (boundp 'rail_pose_estimation_msgs::Poses)
  (if (not (find-package "RAIL_POSE_ESTIMATION_MSGS"))
    (make-package "RAIL_POSE_ESTIMATION_MSGS"))
  (shadow 'Poses (find-package "RAIL_POSE_ESTIMATION_MSGS")))
(unless (find-package "RAIL_POSE_ESTIMATION_MSGS::POSES")
  (make-package "RAIL_POSE_ESTIMATION_MSGS::POSES"))

(in-package "ROS")
;;//! \htmlinclude Poses.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rail_pose_estimation_msgs::Poses
  :super ros::object
  :slots (_header _people ))

(defmethod rail_pose_estimation_msgs::Poses
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:people __people) (let (r) (dotimes (i 0) (push (instance rail_pose_estimation_msgs::Keypoints :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _people __people)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:people
   (&rest __people)
   (if (keywordp (car __people))
       (send* _people __people)
     (progn
       (if __people (setq _people (car __people)))
       _people)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; rail_pose_estimation_msgs/Keypoints[] _people
    (apply #'+ (send-all _people :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; rail_pose_estimation_msgs/Keypoints[] _people
     (write-long (length _people) s)
     (dolist (elem _people)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; rail_pose_estimation_msgs/Keypoints[] _people
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _people (let (r) (dotimes (i n) (push (instance rail_pose_estimation_msgs::Keypoints :init) r)) r))
     (dolist (elem- _people)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get rail_pose_estimation_msgs::Poses :md5sum-) "f3ac5a5326099357214a8296eb96e5d2")
(setf (get rail_pose_estimation_msgs::Poses :datatype-) "rail_pose_estimation_msgs/Poses")
(setf (get rail_pose_estimation_msgs::Poses :definition-)
      "std_msgs/Header header 	# Message header including timestamp of detection & img frame
Keypoints[] people 			# Array of detected objects
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: rail_pose_estimation_msgs/Keypoints
float32 neck_x                  # x coord of neck
float32 neck_y                  # y coord of neck
float32 nose_x                  # x coord of nose
float32 nose_y                  # y coord of nose
float32 right_shoulder_x        # x coord of right shoulder
float32 right_shoulder_y        # y coord of right shoulder
float32 left_shoulder_x         # x coord of left shoulder
float32 left_shoulder_y         # y coord of left shoulder
float32 right_elbow_x           # x coord of right elbow
float32 right_elbow_y           # y coord of right elbow
float32 left_elbow_x            # x coord of left elbow
float32 left_elbow_y            # y coord of left elbow
float32 right_wrist_x           # x coord of right wrist
float32 right_wrist_y           # y coord of right wrist
float32 left_wrist_x            # x coord of left wrist
float32 left_wrist_y            # y coord of left wrist
float32 right_hip_x             # x coord of right hip
float32 right_hip_y             # y coord of right hip
float32 left_hip_x              # x coord of left hip
float32 left_hip_y              # y coord of left hip
float32 right_knee_x            # x coord of right knee
float32 right_knee_y            # y coord of right knee
float32 left_knee_x             # x coord of left knee
float32 left_knee_y             # y coord of left knee
float32 right_ankle_x           # x coord of right ankle
float32 right_ankle_y           # y coord of right ankle
float32 left_ankle_x            # x coord of left ankle
float32 left_ankle_y            # y coord of left ankle
float32 right_eye_x             # x coord of right eye
float32 right_eye_y             # y coord of right eye
float32 left_eye_x              # x coord of left eye
float32 left_eye_y              # y coord of left eye
float32 right_ear_x             # x coord of right ear
float32 right_ear_y             # y coord of right ear
float32 left_ear_x              # x coord of left ear
float32 left_ear_y              # y coord of left ear
")



(provide :rail_pose_estimation_msgs/Poses "f3ac5a5326099357214a8296eb96e5d2")


