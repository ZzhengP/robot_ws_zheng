;; Auto-generated. Do not edit!


(when (boundp 'openni2_tracker::TrackerUser)
  (if (not (find-package "OPENNI2_TRACKER"))
    (make-package "OPENNI2_TRACKER"))
  (shadow 'TrackerUser (find-package "OPENNI2_TRACKER")))
(unless (find-package "OPENNI2_TRACKER::TRACKERUSER")
  (make-package "OPENNI2_TRACKER::TRACKERUSER"))

(in-package "ROS")
;;//! \htmlinclude TrackerUser.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass openni2_tracker::TrackerUser
  :super ros::object
  :slots (_header _uid _tracker_id _frames _confs _transforms _projective _center_of_mass ))

(defmethod openni2_tracker::TrackerUser
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:uid __uid) 0)
    ((:tracker_id __tracker_id) "")
    ((:frames __frames) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:confs __confs) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:transforms __transforms) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Transform :init) r)) r))
    ((:projective __projective) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Vector3 :init) r)) r))
    ((:center_of_mass __center_of_mass) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _uid (round __uid))
   (setq _tracker_id (string __tracker_id))
   (setq _frames __frames)
   (setq _confs __confs)
   (setq _transforms __transforms)
   (setq _projective __projective)
   (setq _center_of_mass __center_of_mass)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:uid
   (&optional __uid)
   (if __uid (setq _uid __uid)) _uid)
  (:tracker_id
   (&optional __tracker_id)
   (if __tracker_id (setq _tracker_id __tracker_id)) _tracker_id)
  (:frames
   (&optional __frames)
   (if __frames (setq _frames __frames)) _frames)
  (:confs
   (&optional __confs)
   (if __confs (setq _confs __confs)) _confs)
  (:transforms
   (&rest __transforms)
   (if (keywordp (car __transforms))
       (send* _transforms __transforms)
     (progn
       (if __transforms (setq _transforms (car __transforms)))
       _transforms)))
  (:projective
   (&rest __projective)
   (if (keywordp (car __projective))
       (send* _projective __projective)
     (progn
       (if __projective (setq _projective (car __projective)))
       _projective)))
  (:center_of_mass
   (&rest __center_of_mass)
   (if (keywordp (car __center_of_mass))
       (send* _center_of_mass __center_of_mass)
     (progn
       (if __center_of_mass (setq _center_of_mass (car __center_of_mass)))
       _center_of_mass)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _uid
    1
    ;; string _tracker_id
    4 (length _tracker_id)
    ;; string[] _frames
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _frames)) 4
    ;; float64[] _confs
    (* 8    (length _confs)) 4
    ;; geometry_msgs/Transform[] _transforms
    (apply #'+ (send-all _transforms :serialization-length)) 4
    ;; geometry_msgs/Vector3[] _projective
    (apply #'+ (send-all _projective :serialization-length)) 4
    ;; geometry_msgs/Vector3 _center_of_mass
    (send _center_of_mass :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _uid
       (write-byte _uid s)
     ;; string _tracker_id
       (write-long (length _tracker_id) s) (princ _tracker_id s)
     ;; string[] _frames
     (write-long (length _frames) s)
     (dolist (elem _frames)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _confs
     (write-long (length _confs) s)
     (dotimes (i (length _confs))
       (sys::poke (elt _confs i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; geometry_msgs/Transform[] _transforms
     (write-long (length _transforms) s)
     (dolist (elem _transforms)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3[] _projective
     (write-long (length _projective) s)
     (dolist (elem _projective)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3 _center_of_mass
       (send _center_of_mass :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _uid
     (setq _uid (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _tracker_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _tracker_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _frames
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _frames (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _frames i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _confs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _confs (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _confs i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; geometry_msgs/Transform[] _transforms
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _transforms (let (r) (dotimes (i n) (push (instance geometry_msgs::Transform :init) r)) r))
     (dolist (elem- _transforms)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Vector3[] _projective
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _projective (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _projective)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Vector3 _center_of_mass
     (send _center_of_mass :deserialize buf ptr-) (incf ptr- (send _center_of_mass :serialization-length))
   ;;
   self)
  )

(setf (get openni2_tracker::TrackerUser :md5sum-) "c4f1e9a80fc94f2519473798dffa838a")
(setf (get openni2_tracker::TrackerUser :datatype-) "openni2_tracker/TrackerUser")
(setf (get openni2_tracker::TrackerUser :definition-)
      "# TrackerUser: Contains a snapshot of a single users tracking data. an ID, a vector of frames corresponding to each joint, a vector of confidences corresponding to each joint, and a vector of geometry_msgs/Transform messages corresponding to each joint. 
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



(provide :openni2_tracker/TrackerUser "c4f1e9a80fc94f2519473798dffa838a")


