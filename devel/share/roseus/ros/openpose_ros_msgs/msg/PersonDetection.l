;; Auto-generated. Do not edit!


(when (boundp 'openpose_ros_msgs::PersonDetection)
  (if (not (find-package "OPENPOSE_ROS_MSGS"))
    (make-package "OPENPOSE_ROS_MSGS"))
  (shadow 'PersonDetection (find-package "OPENPOSE_ROS_MSGS")))
(unless (find-package "OPENPOSE_ROS_MSGS::PERSONDETECTION")
  (make-package "OPENPOSE_ROS_MSGS::PERSONDETECTION"))

(in-package "ROS")
;;//! \htmlinclude PersonDetection.msg.html


(defclass openpose_ros_msgs::PersonDetection
  :super ros::object
  :slots (_face_landmark _body_part ))

(defmethod openpose_ros_msgs::PersonDetection
  (:init
   (&key
    ((:face_landmark __face_landmark) (let (r) (dotimes (i 0) (push (instance openpose_ros_msgs::BodyPartDetection :init) r)) r))
    ((:body_part __body_part) (let (r) (dotimes (i 0) (push (instance openpose_ros_msgs::BodyPartDetection :init) r)) r))
    )
   (send-super :init)
   (setq _face_landmark __face_landmark)
   (setq _body_part __body_part)
   self)
  (:face_landmark
   (&rest __face_landmark)
   (if (keywordp (car __face_landmark))
       (send* _face_landmark __face_landmark)
     (progn
       (if __face_landmark (setq _face_landmark (car __face_landmark)))
       _face_landmark)))
  (:body_part
   (&rest __body_part)
   (if (keywordp (car __body_part))
       (send* _body_part __body_part)
     (progn
       (if __body_part (setq _body_part (car __body_part)))
       _body_part)))
  (:serialization-length
   ()
   (+
    ;; openpose_ros_msgs/BodyPartDetection[] _face_landmark
    (apply #'+ (send-all _face_landmark :serialization-length)) 4
    ;; openpose_ros_msgs/BodyPartDetection[] _body_part
    (apply #'+ (send-all _body_part :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; openpose_ros_msgs/BodyPartDetection[] _face_landmark
     (write-long (length _face_landmark) s)
     (dolist (elem _face_landmark)
       (send elem :serialize s)
       )
     ;; openpose_ros_msgs/BodyPartDetection[] _body_part
     (write-long (length _body_part) s)
     (dolist (elem _body_part)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; openpose_ros_msgs/BodyPartDetection[] _face_landmark
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _face_landmark (let (r) (dotimes (i n) (push (instance openpose_ros_msgs::BodyPartDetection :init) r)) r))
     (dolist (elem- _face_landmark)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; openpose_ros_msgs/BodyPartDetection[] _body_part
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _body_part (let (r) (dotimes (i n) (push (instance openpose_ros_msgs::BodyPartDetection :init) r)) r))
     (dolist (elem- _body_part)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get openpose_ros_msgs::PersonDetection :md5sum-) "5cf57469f872bf144c62ed7904d5c159")
(setf (get openpose_ros_msgs::PersonDetection :datatype-) "openpose_ros_msgs/PersonDetection")
(setf (get openpose_ros_msgs::PersonDetection :definition-)
      "BodyPartDetection[] face_landmark
BodyPartDetection[] body_part

================================================================================
MSG: openpose_ros_msgs/BodyPartDetection
uint32 part_id
uint32 x
uint32 y
float32 confidence

")



(provide :openpose_ros_msgs/PersonDetection "5cf57469f872bf144c62ed7904d5c159")


