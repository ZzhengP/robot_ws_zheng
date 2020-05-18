;; Auto-generated. Do not edit!


(when (boundp 'openpose_ros_msgs::BodypartDetection_3d)
  (if (not (find-package "OPENPOSE_ROS_MSGS"))
    (make-package "OPENPOSE_ROS_MSGS"))
  (shadow 'BodypartDetection_3d (find-package "OPENPOSE_ROS_MSGS")))
(unless (find-package "OPENPOSE_ROS_MSGS::BODYPARTDETECTION_3D")
  (make-package "OPENPOSE_ROS_MSGS::BODYPARTDETECTION_3D"))

(in-package "ROS")
;;//! \htmlinclude BodypartDetection_3d.msg.html


(defclass openpose_ros_msgs::BodypartDetection_3d
  :super ros::object
  :slots (_x _y _z _confidence ))

(defmethod openpose_ros_msgs::BodypartDetection_3d
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _confidence (float __confidence))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ;; float32 _confidence
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get openpose_ros_msgs::BodypartDetection_3d :md5sum-) "a5093144649598135f5a86950009b258")
(setf (get openpose_ros_msgs::BodypartDetection_3d :datatype-) "openpose_ros_msgs/BodypartDetection_3d")
(setf (get openpose_ros_msgs::BodypartDetection_3d :definition-)
      "float32 x
float32 y
float32 z
float32 confidence

")



(provide :openpose_ros_msgs/BodypartDetection_3d "a5093144649598135f5a86950009b258")


