;; Auto-generated. Do not edit!


(when (boundp 'openpose_ros_msgs::PointWithProb)
  (if (not (find-package "OPENPOSE_ROS_MSGS"))
    (make-package "OPENPOSE_ROS_MSGS"))
  (shadow 'PointWithProb (find-package "OPENPOSE_ROS_MSGS")))
(unless (find-package "OPENPOSE_ROS_MSGS::POINTWITHPROB")
  (make-package "OPENPOSE_ROS_MSGS::POINTWITHPROB"))

(in-package "ROS")
;;//! \htmlinclude PointWithProb.msg.html


(defclass openpose_ros_msgs::PointWithProb
  :super ros::object
  :slots (_x _y _prob ))

(defmethod openpose_ros_msgs::PointWithProb
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:prob __prob) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _prob (float __prob))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:prob
   (&optional __prob)
   (if __prob (setq _prob __prob)) _prob)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _prob
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _prob
       (sys::poke _prob (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _prob
     (setq _prob (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get openpose_ros_msgs::PointWithProb :md5sum-) "a9aa8eca031892a98235f2ba7dab5caf")
(setf (get openpose_ros_msgs::PointWithProb :datatype-) "openpose_ros_msgs/PointWithProb")
(setf (get openpose_ros_msgs::PointWithProb :definition-)
      "float64 x
float64 y
float64 prob
")



(provide :openpose_ros_msgs/PointWithProb "a9aa8eca031892a98235f2ba7dab5caf")


