;; Auto-generated. Do not edit!


(when (boundp 'openpose_ros_msgs::BodypartDetection)
  (if (not (find-package "OPENPOSE_ROS_MSGS"))
    (make-package "OPENPOSE_ROS_MSGS"))
  (shadow 'BodypartDetection (find-package "OPENPOSE_ROS_MSGS")))
(unless (find-package "OPENPOSE_ROS_MSGS::BODYPARTDETECTION")
  (make-package "OPENPOSE_ROS_MSGS::BODYPARTDETECTION"))

(in-package "ROS")
;;//! \htmlinclude BodypartDetection.msg.html


(defclass openpose_ros_msgs::BodypartDetection
  :super ros::object
  :slots (_x _y _confidence ))

(defmethod openpose_ros_msgs::BodypartDetection
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _confidence (float __confidence))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; uint32 _x
    4
    ;; uint32 _y
    4
    ;; float32 _confidence
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _x
       (write-long _x s)
     ;; uint32 _y
       (write-long _y s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get openpose_ros_msgs::BodypartDetection :md5sum-) "2edd10e314180ad374d9e70737f3fc60")
(setf (get openpose_ros_msgs::BodypartDetection :datatype-) "openpose_ros_msgs/BodypartDetection")
(setf (get openpose_ros_msgs::BodypartDetection :definition-)
      "uint32 x
uint32 y
float32 confidence

")



(provide :openpose_ros_msgs/BodypartDetection "2edd10e314180ad374d9e70737f3fc60")


