;; Auto-generated. Do not edit!


(when (boundp 'kinect2_tracker::user_points)
  (if (not (find-package "KINECT2_TRACKER"))
    (make-package "KINECT2_TRACKER"))
  (shadow 'user_points (find-package "KINECT2_TRACKER")))
(unless (find-package "KINECT2_TRACKER::USER_POINTS")
  (make-package "KINECT2_TRACKER::USER_POINTS"))

(in-package "ROS")
;;//! \htmlinclude user_points.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass kinect2_tracker::user_points
  :super ros::object
  :slots (_users _people_points _boxes ))

(defmethod kinect2_tracker::user_points
  (:init
   (&key
    ((:users __users) (make-array 0 :initial-element 0 :element-type :char))
    ((:people_points __people_points) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PointStamped :init) r)) r))
    ((:boxes __boxes) (let (r) (dotimes (i 0) (push (instance kinect2_tracker::bounding_box :init) r)) r))
    )
   (send-super :init)
   (setq _users __users)
   (setq _people_points __people_points)
   (setq _boxes __boxes)
   self)
  (:users
   (&optional __users)
   (if __users (setq _users __users)) _users)
  (:people_points
   (&rest __people_points)
   (if (keywordp (car __people_points))
       (send* _people_points __people_points)
     (progn
       (if __people_points (setq _people_points (car __people_points)))
       _people_points)))
  (:boxes
   (&rest __boxes)
   (if (keywordp (car __boxes))
       (send* _boxes __boxes)
     (progn
       (if __boxes (setq _boxes (car __boxes)))
       _boxes)))
  (:serialization-length
   ()
   (+
    ;; uint8[] _users
    (* 1    (length _users)) 4
    ;; geometry_msgs/PointStamped[] _people_points
    (apply #'+ (send-all _people_points :serialization-length)) 4
    ;; kinect2_tracker/bounding_box[] _boxes
    (apply #'+ (send-all _boxes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _users
     (write-long (length _users) s)
     (princ _users s)
     ;; geometry_msgs/PointStamped[] _people_points
     (write-long (length _people_points) s)
     (dolist (elem _people_points)
       (send elem :serialize s)
       )
     ;; kinect2_tracker/bounding_box[] _boxes
     (write-long (length _boxes) s)
     (dolist (elem _boxes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _users
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _users (make-array n :element-type :char))
     (replace _users buf :start2 ptr-) (incf ptr- n))
   ;; geometry_msgs/PointStamped[] _people_points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _people_points (let (r) (dotimes (i n) (push (instance geometry_msgs::PointStamped :init) r)) r))
     (dolist (elem- _people_points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kinect2_tracker/bounding_box[] _boxes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _boxes (let (r) (dotimes (i n) (push (instance kinect2_tracker::bounding_box :init) r)) r))
     (dolist (elem- _boxes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kinect2_tracker::user_points :md5sum-) "6624cab0be89cfdee984fa8acaafad2a")
(setf (get kinect2_tracker::user_points :datatype-) "kinect2_tracker/user_points")
(setf (get kinect2_tracker::user_points :definition-)
      "# Message storing the points of users.
# Header header
uint8[] users
geometry_msgs/PointStamped[] people_points
bounding_box[] boxes
================================================================================
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: kinect2_tracker/bounding_box
geometry_msgs/PointStamped min
geometry_msgs/PointStamped max
")



(provide :kinect2_tracker/user_points "6624cab0be89cfdee984fa8acaafad2a")


