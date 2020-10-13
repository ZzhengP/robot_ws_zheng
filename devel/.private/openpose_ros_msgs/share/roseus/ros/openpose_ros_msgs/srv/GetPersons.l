;; Auto-generated. Do not edit!


(when (boundp 'openpose_ros_msgs::GetPersons)
  (if (not (find-package "OPENPOSE_ROS_MSGS"))
    (make-package "OPENPOSE_ROS_MSGS"))
  (shadow 'GetPersons (find-package "OPENPOSE_ROS_MSGS")))
(unless (find-package "OPENPOSE_ROS_MSGS::GETPERSONS")
  (make-package "OPENPOSE_ROS_MSGS::GETPERSONS"))
(unless (find-package "OPENPOSE_ROS_MSGS::GETPERSONSREQUEST")
  (make-package "OPENPOSE_ROS_MSGS::GETPERSONSREQUEST"))
(unless (find-package "OPENPOSE_ROS_MSGS::GETPERSONSRESPONSE")
  (make-package "OPENPOSE_ROS_MSGS::GETPERSONSRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))




(defclass openpose_ros_msgs::GetPersonsRequest
  :super ros::object
  :slots (_image ))

(defmethod openpose_ros_msgs::GetPersonsRequest
  (:init
   (&key
    ((:image __image) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _image __image)
   self)
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;;
   self)
  )

(defclass openpose_ros_msgs::GetPersonsResponse
  :super ros::object
  :slots (_detections ))

(defmethod openpose_ros_msgs::GetPersonsResponse
  (:init
   (&key
    ((:detections __detections) (let (r) (dotimes (i 0) (push (instance openpose_ros_msgs::PersonDetection :init) r)) r))
    )
   (send-super :init)
   (setq _detections __detections)
   self)
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:serialization-length
   ()
   (+
    ;; openpose_ros_msgs/PersonDetection[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; openpose_ros_msgs/PersonDetection[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; openpose_ros_msgs/PersonDetection[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance openpose_ros_msgs::PersonDetection :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass openpose_ros_msgs::GetPersons
  :super ros::object
  :slots ())

(setf (get openpose_ros_msgs::GetPersons :md5sum-) "b372c9a3a4112a64a284d512401cc11f")
(setf (get openpose_ros_msgs::GetPersons :datatype-) "openpose_ros_msgs/GetPersons")
(setf (get openpose_ros_msgs::GetPersons :request) openpose_ros_msgs::GetPersonsRequest)
(setf (get openpose_ros_msgs::GetPersons :response) openpose_ros_msgs::GetPersonsResponse)

(defmethod openpose_ros_msgs::GetPersonsRequest
  (:response () (instance openpose_ros_msgs::GetPersonsResponse :init)))

(setf (get openpose_ros_msgs::GetPersonsRequest :md5sum-) "b372c9a3a4112a64a284d512401cc11f")
(setf (get openpose_ros_msgs::GetPersonsRequest :datatype-) "openpose_ros_msgs/GetPersonsRequest")
(setf (get openpose_ros_msgs::GetPersonsRequest :definition-)
      "sensor_msgs/Image image

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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
---
PersonDetection[] detections

================================================================================
MSG: openpose_ros_msgs/PersonDetection
uint32 num_people_detected
uint32 person_ID
BodypartDetection nose
BodypartDetection neck
BodypartDetection right_shoulder
BodypartDetection right_elbow
BodypartDetection right_wrist
BodypartDetection left_shoulder
BodypartDetection left_elbow
BodypartDetection left_wrist
BodypartDetection right_hip
BodypartDetection right_knee
BodypartDetection right_ankle
BodypartDetection left_hip
BodypartDetection left_knee
BodypartDetection left_ankle
BodypartDetection right_eye
BodypartDetection left_eye
BodypartDetection right_ear
BodypartDetection left_ear
BodypartDetection chest

================================================================================
MSG: openpose_ros_msgs/BodypartDetection
uint32 x
uint32 y
float32 confidence
")

(setf (get openpose_ros_msgs::GetPersonsResponse :md5sum-) "b372c9a3a4112a64a284d512401cc11f")
(setf (get openpose_ros_msgs::GetPersonsResponse :datatype-) "openpose_ros_msgs/GetPersonsResponse")
(setf (get openpose_ros_msgs::GetPersonsResponse :definition-)
      "sensor_msgs/Image image

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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
---
PersonDetection[] detections

================================================================================
MSG: openpose_ros_msgs/PersonDetection
uint32 num_people_detected
uint32 person_ID
BodypartDetection nose
BodypartDetection neck
BodypartDetection right_shoulder
BodypartDetection right_elbow
BodypartDetection right_wrist
BodypartDetection left_shoulder
BodypartDetection left_elbow
BodypartDetection left_wrist
BodypartDetection right_hip
BodypartDetection right_knee
BodypartDetection right_ankle
BodypartDetection left_hip
BodypartDetection left_knee
BodypartDetection left_ankle
BodypartDetection right_eye
BodypartDetection left_eye
BodypartDetection right_ear
BodypartDetection left_ear
BodypartDetection chest

================================================================================
MSG: openpose_ros_msgs/BodypartDetection
uint32 x
uint32 y
float32 confidence
")



(provide :openpose_ros_msgs/GetPersons "b372c9a3a4112a64a284d512401cc11f")


