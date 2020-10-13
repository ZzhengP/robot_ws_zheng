;; Auto-generated. Do not edit!


(when (boundp 'openni2_tracker::TrackerUserArray)
  (if (not (find-package "OPENNI2_TRACKER"))
    (make-package "OPENNI2_TRACKER"))
  (shadow 'TrackerUserArray (find-package "OPENNI2_TRACKER")))
(unless (find-package "OPENNI2_TRACKER::TRACKERUSERARRAY")
  (make-package "OPENNI2_TRACKER::TRACKERUSERARRAY"))

(in-package "ROS")
;;//! \htmlinclude TrackerUserArray.msg.html


(defclass openni2_tracker::TrackerUserArray
  :super ros::object
  :slots (_users _numUsers ))

(defmethod openni2_tracker::TrackerUserArray
  (:init
   (&key
    ((:users __users) (let (r) (dotimes (i 0) (push (instance openni2_tracker::TrackerUser :init) r)) r))
    ((:numUsers __numUsers) 0)
    )
   (send-super :init)
   (setq _users __users)
   (setq _numUsers (round __numUsers))
   self)
  (:users
   (&rest __users)
   (if (keywordp (car __users))
       (send* _users __users)
     (progn
       (if __users (setq _users (car __users)))
       _users)))
  (:numUsers
   (&optional __numUsers)
   (if __numUsers (setq _numUsers __numUsers)) _numUsers)
  (:serialization-length
   ()
   (+
    ;; openni2_tracker/TrackerUser[] _users
    (apply #'+ (send-all _users :serialization-length)) 4
    ;; int8 _numUsers
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; openni2_tracker/TrackerUser[] _users
     (write-long (length _users) s)
     (dolist (elem _users)
       (send elem :serialize s)
       )
     ;; int8 _numUsers
       (write-byte _numUsers s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; openni2_tracker/TrackerUser[] _users
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _users (let (r) (dotimes (i n) (push (instance openni2_tracker::TrackerUser :init) r)) r))
     (dolist (elem- _users)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int8 _numUsers
     (setq _numUsers (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _numUsers 127) (setq _numUsers (- _numUsers 256)))
   ;;
   self)
  )

(setf (get openni2_tracker::TrackerUserArray :md5sum-) "825d95b7c6acaad9dc439d2148963f2e")
(setf (get openni2_tracker::TrackerUserArray :datatype-) "openni2_tracker/TrackerUserArray")
(setf (get openni2_tracker::TrackerUserArray :definition-)
      "# TrackerUserArray: This message contains a vector of openni2_tracker/TrackerUser messages.
openni2_tracker/TrackerUser[] users
int8 numUsers

================================================================================
MSG: openni2_tracker/TrackerUser
# TrackerUser: Contains a snapshot of a single users tracking data. an ID, a vector of frames corresponding to each joint, a vector of confidences corresponding to each joint, and a vector of geometry_msgs/Transform messages corresponding to each joint. 
Header header
uint8 uid
string tracker_id
string[] frames
float64[] confs
geometry_msgs/Transform[] transforms
geometry_msgs/Vector3[] projective
geometry_msgs/Vector3 center_of_mass

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
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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

")



(provide :openni2_tracker/TrackerUserArray "825d95b7c6acaad9dc439d2148963f2e")


