;; Auto-generated. Do not edit!


(when (boundp 'kinect2_tracker::bounding_box)
  (if (not (find-package "KINECT2_TRACKER"))
    (make-package "KINECT2_TRACKER"))
  (shadow 'bounding_box (find-package "KINECT2_TRACKER")))
(unless (find-package "KINECT2_TRACKER::BOUNDING_BOX")
  (make-package "KINECT2_TRACKER::BOUNDING_BOX"))

(in-package "ROS")
;;//! \htmlinclude bounding_box.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass kinect2_tracker::bounding_box
  :super ros::object
  :slots (_min _max ))

(defmethod kinect2_tracker::bounding_box
  (:init
   (&key
    ((:min __min) (instance geometry_msgs::PointStamped :init))
    ((:max __max) (instance geometry_msgs::PointStamped :init))
    )
   (send-super :init)
   (setq _min __min)
   (setq _max __max)
   self)
  (:min
   (&rest __min)
   (if (keywordp (car __min))
       (send* _min __min)
     (progn
       (if __min (setq _min (car __min)))
       _min)))
  (:max
   (&rest __max)
   (if (keywordp (car __max))
       (send* _max __max)
     (progn
       (if __max (setq _max (car __max)))
       _max)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PointStamped _min
    (send _min :serialization-length)
    ;; geometry_msgs/PointStamped _max
    (send _max :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PointStamped _min
       (send _min :serialize s)
     ;; geometry_msgs/PointStamped _max
       (send _max :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PointStamped _min
     (send _min :deserialize buf ptr-) (incf ptr- (send _min :serialization-length))
   ;; geometry_msgs/PointStamped _max
     (send _max :deserialize buf ptr-) (incf ptr- (send _max :serialization-length))
   ;;
   self)
  )

(setf (get kinect2_tracker::bounding_box :md5sum-) "5a9f041aa663bad83df903fb4bed6c5c")
(setf (get kinect2_tracker::bounding_box :datatype-) "kinect2_tracker/bounding_box")
(setf (get kinect2_tracker::bounding_box :definition-)
      "geometry_msgs/PointStamped min
geometry_msgs/PointStamped max
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

")



(provide :kinect2_tracker/bounding_box "5a9f041aa663bad83df903fb4bed6c5c")


