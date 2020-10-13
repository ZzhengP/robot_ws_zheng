;; Auto-generated. Do not edit!


(when (boundp 'openpose_ros_msgs::OpenPoseHumanList)
  (if (not (find-package "OPENPOSE_ROS_MSGS"))
    (make-package "OPENPOSE_ROS_MSGS"))
  (shadow 'OpenPoseHumanList (find-package "OPENPOSE_ROS_MSGS")))
(unless (find-package "OPENPOSE_ROS_MSGS::OPENPOSEHUMANLIST")
  (make-package "OPENPOSE_ROS_MSGS::OPENPOSEHUMANLIST"))

(in-package "ROS")
;;//! \htmlinclude OpenPoseHumanList.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass openpose_ros_msgs::OpenPoseHumanList
  :super ros::object
  :slots (_header _image_header _num_humans _human_list ))

(defmethod openpose_ros_msgs::OpenPoseHumanList
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:image_header __image_header) (instance std_msgs::Header :init))
    ((:num_humans __num_humans) 0)
    ((:human_list __human_list) (let (r) (dotimes (i 0) (push (instance openpose_ros_msgs::OpenPoseHuman :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _image_header __image_header)
   (setq _num_humans (round __num_humans))
   (setq _human_list __human_list)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:image_header
   (&rest __image_header)
   (if (keywordp (car __image_header))
       (send* _image_header __image_header)
     (progn
       (if __image_header (setq _image_header (car __image_header)))
       _image_header)))
  (:num_humans
   (&optional __num_humans)
   (if __num_humans (setq _num_humans __num_humans)) _num_humans)
  (:human_list
   (&rest __human_list)
   (if (keywordp (car __human_list))
       (send* _human_list __human_list)
     (progn
       (if __human_list (setq _human_list (car __human_list)))
       _human_list)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/Header _image_header
    (send _image_header :serialization-length)
    ;; int32 _num_humans
    4
    ;; openpose_ros_msgs/OpenPoseHuman[] _human_list
    (apply #'+ (send-all _human_list :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/Header _image_header
       (send _image_header :serialize s)
     ;; int32 _num_humans
       (write-long _num_humans s)
     ;; openpose_ros_msgs/OpenPoseHuman[] _human_list
     (write-long (length _human_list) s)
     (dolist (elem _human_list)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/Header _image_header
     (send _image_header :deserialize buf ptr-) (incf ptr- (send _image_header :serialization-length))
   ;; int32 _num_humans
     (setq _num_humans (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; openpose_ros_msgs/OpenPoseHuman[] _human_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _human_list (let (r) (dotimes (i n) (push (instance openpose_ros_msgs::OpenPoseHuman :init) r)) r))
     (dolist (elem- _human_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get openpose_ros_msgs::OpenPoseHumanList :md5sum-) "520d81a8dc226a535b021aef637b0340")
(setf (get openpose_ros_msgs::OpenPoseHumanList :datatype-) "openpose_ros_msgs/OpenPoseHumanList")
(setf (get openpose_ros_msgs::OpenPoseHumanList :definition-)
      "Header header
Header image_header
int32 num_humans
OpenPoseHuman[] human_list
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
MSG: openpose_ros_msgs/OpenPoseHuman
int32 num_body_key_points_with_non_zero_prob
int32 num_face_key_points_with_non_zero_prob
int32 num_right_hand_key_points_with_non_zero_prob
int32 num_left_hand_key_points_with_non_zero_prob
BoundingBox body_bounding_box
BoundingBox face_bounding_box
PointWithProb[25] body_key_points_with_prob
PointWithProb[70] face_key_points_with_prob
PointWithProb[21] right_hand_key_points_with_prob
PointWithProb[21] left_hand_key_points_with_prob
================================================================================
MSG: openpose_ros_msgs/BoundingBox
float64 x
float64 y
float64 width
float64 height
================================================================================
MSG: openpose_ros_msgs/PointWithProb
float64 x
float64 y
float64 prob
")



(provide :openpose_ros_msgs/OpenPoseHumanList "520d81a8dc226a535b021aef637b0340")


