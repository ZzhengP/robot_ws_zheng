;; Auto-generated. Do not edit!


(when (boundp 'human_moveit_config::GetHumanIK)
  (if (not (find-package "HUMAN_MOVEIT_CONFIG"))
    (make-package "HUMAN_MOVEIT_CONFIG"))
  (shadow 'GetHumanIK (find-package "HUMAN_MOVEIT_CONFIG")))
(unless (find-package "HUMAN_MOVEIT_CONFIG::GETHUMANIK")
  (make-package "HUMAN_MOVEIT_CONFIG::GETHUMANIK"))
(unless (find-package "HUMAN_MOVEIT_CONFIG::GETHUMANIKREQUEST")
  (make-package "HUMAN_MOVEIT_CONFIG::GETHUMANIKREQUEST"))
(unless (find-package "HUMAN_MOVEIT_CONFIG::GETHUMANIKRESPONSE")
  (make-package "HUMAN_MOVEIT_CONFIG::GETHUMANIKRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass human_moveit_config::GetHumanIKRequest
  :super ros::object
  :slots (_desired_poses _fixed_joints _tolerance _group_names _seed ))

(defmethod human_moveit_config::GetHumanIKRequest
  (:init
   (&key
    ((:desired_poses __desired_poses) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PoseStamped :init) r)) r))
    ((:fixed_joints __fixed_joints) (instance sensor_msgs::JointState :init))
    ((:tolerance __tolerance) 0.0)
    ((:group_names __group_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:seed __seed) (instance sensor_msgs::JointState :init))
    )
   (send-super :init)
   (setq _desired_poses __desired_poses)
   (setq _fixed_joints __fixed_joints)
   (setq _tolerance (float __tolerance))
   (setq _group_names __group_names)
   (setq _seed __seed)
   self)
  (:desired_poses
   (&rest __desired_poses)
   (if (keywordp (car __desired_poses))
       (send* _desired_poses __desired_poses)
     (progn
       (if __desired_poses (setq _desired_poses (car __desired_poses)))
       _desired_poses)))
  (:fixed_joints
   (&rest __fixed_joints)
   (if (keywordp (car __fixed_joints))
       (send* _fixed_joints __fixed_joints)
     (progn
       (if __fixed_joints (setq _fixed_joints (car __fixed_joints)))
       _fixed_joints)))
  (:tolerance
   (&optional __tolerance)
   (if __tolerance (setq _tolerance __tolerance)) _tolerance)
  (:group_names
   (&optional __group_names)
   (if __group_names (setq _group_names __group_names)) _group_names)
  (:seed
   (&rest __seed)
   (if (keywordp (car __seed))
       (send* _seed __seed)
     (progn
       (if __seed (setq _seed (car __seed)))
       _seed)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped[] _desired_poses
    (apply #'+ (send-all _desired_poses :serialization-length)) 4
    ;; sensor_msgs/JointState _fixed_joints
    (send _fixed_joints :serialization-length)
    ;; float32 _tolerance
    4
    ;; string[] _group_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _group_names)) 4
    ;; sensor_msgs/JointState _seed
    (send _seed :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped[] _desired_poses
     (write-long (length _desired_poses) s)
     (dolist (elem _desired_poses)
       (send elem :serialize s)
       )
     ;; sensor_msgs/JointState _fixed_joints
       (send _fixed_joints :serialize s)
     ;; float32 _tolerance
       (sys::poke _tolerance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string[] _group_names
     (write-long (length _group_names) s)
     (dolist (elem _group_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; sensor_msgs/JointState _seed
       (send _seed :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped[] _desired_poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _desired_poses (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _desired_poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; sensor_msgs/JointState _fixed_joints
     (send _fixed_joints :deserialize buf ptr-) (incf ptr- (send _fixed_joints :serialization-length))
   ;; float32 _tolerance
     (setq _tolerance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string[] _group_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _group_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _group_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; sensor_msgs/JointState _seed
     (send _seed :deserialize buf ptr-) (incf ptr- (send _seed :serialization-length))
   ;;
   self)
  )

(defclass human_moveit_config::GetHumanIKResponse
  :super ros::object
  :slots (_joint_state ))

(defmethod human_moveit_config::GetHumanIKResponse
  (:init
   (&key
    ((:joint_state __joint_state) (instance sensor_msgs::JointState :init))
    )
   (send-super :init)
   (setq _joint_state __joint_state)
   self)
  (:joint_state
   (&rest __joint_state)
   (if (keywordp (car __joint_state))
       (send* _joint_state __joint_state)
     (progn
       (if __joint_state (setq _joint_state (car __joint_state)))
       _joint_state)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/JointState _joint_state
    (send _joint_state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/JointState _joint_state
       (send _joint_state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/JointState _joint_state
     (send _joint_state :deserialize buf ptr-) (incf ptr- (send _joint_state :serialization-length))
   ;;
   self)
  )

(defclass human_moveit_config::GetHumanIK
  :super ros::object
  :slots ())

(setf (get human_moveit_config::GetHumanIK :md5sum-) "2602509d5e7f9ded6d3747732576ab30")
(setf (get human_moveit_config::GetHumanIK :datatype-) "human_moveit_config/GetHumanIK")
(setf (get human_moveit_config::GetHumanIK :request) human_moveit_config::GetHumanIKRequest)
(setf (get human_moveit_config::GetHumanIK :response) human_moveit_config::GetHumanIKResponse)

(defmethod human_moveit_config::GetHumanIKRequest
  (:response () (instance human_moveit_config::GetHumanIKResponse :init)))

(setf (get human_moveit_config::GetHumanIKRequest :md5sum-) "2602509d5e7f9ded6d3747732576ab30")
(setf (get human_moveit_config::GetHumanIKRequest :datatype-) "human_moveit_config/GetHumanIKRequest")
(setf (get human_moveit_config::GetHumanIKRequest :definition-)
      "









geometry_msgs/PoseStamped[] desired_poses
sensor_msgs/JointState fixed_joints
float32 tolerance
string[] group_names
sensor_msgs/JointState seed

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort
---
sensor_msgs/JointState joint_state

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

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
")

(setf (get human_moveit_config::GetHumanIKResponse :md5sum-) "2602509d5e7f9ded6d3747732576ab30")
(setf (get human_moveit_config::GetHumanIKResponse :datatype-) "human_moveit_config/GetHumanIKResponse")
(setf (get human_moveit_config::GetHumanIKResponse :definition-)
      "









geometry_msgs/PoseStamped[] desired_poses
sensor_msgs/JointState fixed_joints
float32 tolerance
string[] group_names
sensor_msgs/JointState seed

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort
---
sensor_msgs/JointState joint_state

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

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
")



(provide :human_moveit_config/GetHumanIK "2602509d5e7f9ded6d3747732576ab30")


