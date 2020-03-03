;; Auto-generated. Do not edit!


(when (boundp 'openpose_ros_msgs::BodyPartDetection)
  (if (not (find-package "OPENPOSE_ROS_MSGS"))
    (make-package "OPENPOSE_ROS_MSGS"))
  (shadow 'BodyPartDetection (find-package "OPENPOSE_ROS_MSGS")))
(unless (find-package "OPENPOSE_ROS_MSGS::BODYPARTDETECTION")
  (make-package "OPENPOSE_ROS_MSGS::BODYPARTDETECTION"))

(in-package "ROS")
;;//! \htmlinclude BodyPartDetection.msg.html


(defclass openpose_ros_msgs::BodyPartDetection
  :super ros::object
  :slots (_part_id _x _y _confidence ))

(defmethod openpose_ros_msgs::BodyPartDetection
  (:init
   (&key
    ((:part_id __part_id) 0)
    ((:x __x) 0)
    ((:y __y) 0)
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _part_id (round __part_id))
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _confidence (float __confidence))
   self)
  (:part_id
   (&optional __part_id)
   (if __part_id (setq _part_id __part_id)) _part_id)
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
    ;; uint32 _part_id
    4
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
     ;; uint32 _part_id
       (write-long _part_id s)
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
   ;; uint32 _part_id
     (setq _part_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get openpose_ros_msgs::BodyPartDetection :md5sum-) "8d74904ee28d2aa6d3a7a77de5e2711e")
(setf (get openpose_ros_msgs::BodyPartDetection :datatype-) "openpose_ros_msgs/BodyPartDetection")
(setf (get openpose_ros_msgs::BodyPartDetection :definition-)
      "uint32 part_id
uint32 x
uint32 y
float32 confidence

")



(provide :openpose_ros_msgs/BodyPartDetection "8d74904ee28d2aa6d3a7a77de5e2711e")


