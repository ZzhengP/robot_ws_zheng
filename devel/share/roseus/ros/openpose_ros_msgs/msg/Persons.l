;; Auto-generated. Do not edit!


(when (boundp 'openpose_ros_msgs::Persons)
  (if (not (find-package "OPENPOSE_ROS_MSGS"))
    (make-package "OPENPOSE_ROS_MSGS"))
  (shadow 'Persons (find-package "OPENPOSE_ROS_MSGS")))
(unless (find-package "OPENPOSE_ROS_MSGS::PERSONS")
  (make-package "OPENPOSE_ROS_MSGS::PERSONS"))

(in-package "ROS")
;;//! \htmlinclude Persons.msg.html


(defclass openpose_ros_msgs::Persons
  :super ros::object
  :slots (_persons _rostime _image_w _image_h ))

(defmethod openpose_ros_msgs::Persons
  (:init
   (&key
    ((:persons __persons) (let (r) (dotimes (i 0) (push (instance openpose_ros_msgs::PersonDetection :init) r)) r))
    ((:rostime __rostime) (instance ros::time :init))
    ((:image_w __image_w) 0)
    ((:image_h __image_h) 0)
    )
   (send-super :init)
   (setq _persons __persons)
   (setq _rostime __rostime)
   (setq _image_w (round __image_w))
   (setq _image_h (round __image_h))
   self)
  (:persons
   (&rest __persons)
   (if (keywordp (car __persons))
       (send* _persons __persons)
     (progn
       (if __persons (setq _persons (car __persons)))
       _persons)))
  (:rostime
   (&optional __rostime)
   (if __rostime (setq _rostime __rostime)) _rostime)
  (:image_w
   (&optional __image_w)
   (if __image_w (setq _image_w __image_w)) _image_w)
  (:image_h
   (&optional __image_h)
   (if __image_h (setq _image_h __image_h)) _image_h)
  (:serialization-length
   ()
   (+
    ;; openpose_ros_msgs/PersonDetection[] _persons
    (apply #'+ (send-all _persons :serialization-length)) 4
    ;; time _rostime
    8
    ;; uint32 _image_w
    4
    ;; uint32 _image_h
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; openpose_ros_msgs/PersonDetection[] _persons
     (write-long (length _persons) s)
     (dolist (elem _persons)
       (send elem :serialize s)
       )
     ;; time _rostime
       (write-long (send _rostime :sec) s) (write-long (send _rostime :nsec) s)
     ;; uint32 _image_w
       (write-long _image_w s)
     ;; uint32 _image_h
       (write-long _image_h s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; openpose_ros_msgs/PersonDetection[] _persons
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _persons (let (r) (dotimes (i n) (push (instance openpose_ros_msgs::PersonDetection :init) r)) r))
     (dolist (elem- _persons)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; time _rostime
     (send _rostime :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _rostime :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _image_w
     (setq _image_w (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _image_h
     (setq _image_h (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get openpose_ros_msgs::Persons :md5sum-) "364e552085331e28feea9ffef91f2929")
(setf (get openpose_ros_msgs::Persons :datatype-) "openpose_ros_msgs/Persons")
(setf (get openpose_ros_msgs::Persons :definition-)
      "PersonDetection[] persons
time rostime
uint32 image_w
uint32 image_h

================================================================================
MSG: openpose_ros_msgs/PersonDetection
BodyPartDetection[] face_landmark
BodyPartDetection[] body_part

================================================================================
MSG: openpose_ros_msgs/BodyPartDetection
uint32 part_id
uint32 x
uint32 y
float32 confidence

")



(provide :openpose_ros_msgs/Persons "364e552085331e28feea9ffef91f2929")


