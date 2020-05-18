;; Auto-generated. Do not edit!


(when (boundp 'openpose_ros_msgs::PersonDetection_3d)
  (if (not (find-package "OPENPOSE_ROS_MSGS"))
    (make-package "OPENPOSE_ROS_MSGS"))
  (shadow 'PersonDetection_3d (find-package "OPENPOSE_ROS_MSGS")))
(unless (find-package "OPENPOSE_ROS_MSGS::PERSONDETECTION_3D")
  (make-package "OPENPOSE_ROS_MSGS::PERSONDETECTION_3D"))

(in-package "ROS")
;;//! \htmlinclude PersonDetection_3d.msg.html


(defclass openpose_ros_msgs::PersonDetection_3d
  :super ros::object
  :slots (_num_people_detected _person_ID _nose _neck _right_shoulder _right_elbow _right_wrist _left_shoulder _left_elbow _left_wrist _right_hip _right_knee _right_ankle _left_hip _left_knee _left_ankle _right_eye _left_eye _right_ear _left_ear _chest ))

(defmethod openpose_ros_msgs::PersonDetection_3d
  (:init
   (&key
    ((:num_people_detected __num_people_detected) 0)
    ((:person_ID __person_ID) 0)
    ((:nose __nose) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:neck __neck) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:right_shoulder __right_shoulder) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:right_elbow __right_elbow) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:right_wrist __right_wrist) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:left_shoulder __left_shoulder) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:left_elbow __left_elbow) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:left_wrist __left_wrist) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:right_hip __right_hip) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:right_knee __right_knee) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:right_ankle __right_ankle) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:left_hip __left_hip) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:left_knee __left_knee) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:left_ankle __left_ankle) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:right_eye __right_eye) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:left_eye __left_eye) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:right_ear __right_ear) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:left_ear __left_ear) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    ((:chest __chest) (instance openpose_ros_msgs::BodypartDetection_3d :init))
    )
   (send-super :init)
   (setq _num_people_detected (round __num_people_detected))
   (setq _person_ID (round __person_ID))
   (setq _nose __nose)
   (setq _neck __neck)
   (setq _right_shoulder __right_shoulder)
   (setq _right_elbow __right_elbow)
   (setq _right_wrist __right_wrist)
   (setq _left_shoulder __left_shoulder)
   (setq _left_elbow __left_elbow)
   (setq _left_wrist __left_wrist)
   (setq _right_hip __right_hip)
   (setq _right_knee __right_knee)
   (setq _right_ankle __right_ankle)
   (setq _left_hip __left_hip)
   (setq _left_knee __left_knee)
   (setq _left_ankle __left_ankle)
   (setq _right_eye __right_eye)
   (setq _left_eye __left_eye)
   (setq _right_ear __right_ear)
   (setq _left_ear __left_ear)
   (setq _chest __chest)
   self)
  (:num_people_detected
   (&optional __num_people_detected)
   (if __num_people_detected (setq _num_people_detected __num_people_detected)) _num_people_detected)
  (:person_ID
   (&optional __person_ID)
   (if __person_ID (setq _person_ID __person_ID)) _person_ID)
  (:nose
   (&rest __nose)
   (if (keywordp (car __nose))
       (send* _nose __nose)
     (progn
       (if __nose (setq _nose (car __nose)))
       _nose)))
  (:neck
   (&rest __neck)
   (if (keywordp (car __neck))
       (send* _neck __neck)
     (progn
       (if __neck (setq _neck (car __neck)))
       _neck)))
  (:right_shoulder
   (&rest __right_shoulder)
   (if (keywordp (car __right_shoulder))
       (send* _right_shoulder __right_shoulder)
     (progn
       (if __right_shoulder (setq _right_shoulder (car __right_shoulder)))
       _right_shoulder)))
  (:right_elbow
   (&rest __right_elbow)
   (if (keywordp (car __right_elbow))
       (send* _right_elbow __right_elbow)
     (progn
       (if __right_elbow (setq _right_elbow (car __right_elbow)))
       _right_elbow)))
  (:right_wrist
   (&rest __right_wrist)
   (if (keywordp (car __right_wrist))
       (send* _right_wrist __right_wrist)
     (progn
       (if __right_wrist (setq _right_wrist (car __right_wrist)))
       _right_wrist)))
  (:left_shoulder
   (&rest __left_shoulder)
   (if (keywordp (car __left_shoulder))
       (send* _left_shoulder __left_shoulder)
     (progn
       (if __left_shoulder (setq _left_shoulder (car __left_shoulder)))
       _left_shoulder)))
  (:left_elbow
   (&rest __left_elbow)
   (if (keywordp (car __left_elbow))
       (send* _left_elbow __left_elbow)
     (progn
       (if __left_elbow (setq _left_elbow (car __left_elbow)))
       _left_elbow)))
  (:left_wrist
   (&rest __left_wrist)
   (if (keywordp (car __left_wrist))
       (send* _left_wrist __left_wrist)
     (progn
       (if __left_wrist (setq _left_wrist (car __left_wrist)))
       _left_wrist)))
  (:right_hip
   (&rest __right_hip)
   (if (keywordp (car __right_hip))
       (send* _right_hip __right_hip)
     (progn
       (if __right_hip (setq _right_hip (car __right_hip)))
       _right_hip)))
  (:right_knee
   (&rest __right_knee)
   (if (keywordp (car __right_knee))
       (send* _right_knee __right_knee)
     (progn
       (if __right_knee (setq _right_knee (car __right_knee)))
       _right_knee)))
  (:right_ankle
   (&rest __right_ankle)
   (if (keywordp (car __right_ankle))
       (send* _right_ankle __right_ankle)
     (progn
       (if __right_ankle (setq _right_ankle (car __right_ankle)))
       _right_ankle)))
  (:left_hip
   (&rest __left_hip)
   (if (keywordp (car __left_hip))
       (send* _left_hip __left_hip)
     (progn
       (if __left_hip (setq _left_hip (car __left_hip)))
       _left_hip)))
  (:left_knee
   (&rest __left_knee)
   (if (keywordp (car __left_knee))
       (send* _left_knee __left_knee)
     (progn
       (if __left_knee (setq _left_knee (car __left_knee)))
       _left_knee)))
  (:left_ankle
   (&rest __left_ankle)
   (if (keywordp (car __left_ankle))
       (send* _left_ankle __left_ankle)
     (progn
       (if __left_ankle (setq _left_ankle (car __left_ankle)))
       _left_ankle)))
  (:right_eye
   (&rest __right_eye)
   (if (keywordp (car __right_eye))
       (send* _right_eye __right_eye)
     (progn
       (if __right_eye (setq _right_eye (car __right_eye)))
       _right_eye)))
  (:left_eye
   (&rest __left_eye)
   (if (keywordp (car __left_eye))
       (send* _left_eye __left_eye)
     (progn
       (if __left_eye (setq _left_eye (car __left_eye)))
       _left_eye)))
  (:right_ear
   (&rest __right_ear)
   (if (keywordp (car __right_ear))
       (send* _right_ear __right_ear)
     (progn
       (if __right_ear (setq _right_ear (car __right_ear)))
       _right_ear)))
  (:left_ear
   (&rest __left_ear)
   (if (keywordp (car __left_ear))
       (send* _left_ear __left_ear)
     (progn
       (if __left_ear (setq _left_ear (car __left_ear)))
       _left_ear)))
  (:chest
   (&rest __chest)
   (if (keywordp (car __chest))
       (send* _chest __chest)
     (progn
       (if __chest (setq _chest (car __chest)))
       _chest)))
  (:serialization-length
   ()
   (+
    ;; uint32 _num_people_detected
    4
    ;; uint32 _person_ID
    4
    ;; openpose_ros_msgs/BodypartDetection_3d _nose
    (send _nose :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _neck
    (send _neck :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _right_shoulder
    (send _right_shoulder :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _right_elbow
    (send _right_elbow :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _right_wrist
    (send _right_wrist :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _left_shoulder
    (send _left_shoulder :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _left_elbow
    (send _left_elbow :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _left_wrist
    (send _left_wrist :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _right_hip
    (send _right_hip :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _right_knee
    (send _right_knee :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _right_ankle
    (send _right_ankle :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _left_hip
    (send _left_hip :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _left_knee
    (send _left_knee :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _left_ankle
    (send _left_ankle :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _right_eye
    (send _right_eye :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _left_eye
    (send _left_eye :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _right_ear
    (send _right_ear :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _left_ear
    (send _left_ear :serialization-length)
    ;; openpose_ros_msgs/BodypartDetection_3d _chest
    (send _chest :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _num_people_detected
       (write-long _num_people_detected s)
     ;; uint32 _person_ID
       (write-long _person_ID s)
     ;; openpose_ros_msgs/BodypartDetection_3d _nose
       (send _nose :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _neck
       (send _neck :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _right_shoulder
       (send _right_shoulder :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _right_elbow
       (send _right_elbow :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _right_wrist
       (send _right_wrist :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _left_shoulder
       (send _left_shoulder :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _left_elbow
       (send _left_elbow :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _left_wrist
       (send _left_wrist :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _right_hip
       (send _right_hip :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _right_knee
       (send _right_knee :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _right_ankle
       (send _right_ankle :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _left_hip
       (send _left_hip :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _left_knee
       (send _left_knee :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _left_ankle
       (send _left_ankle :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _right_eye
       (send _right_eye :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _left_eye
       (send _left_eye :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _right_ear
       (send _right_ear :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _left_ear
       (send _left_ear :serialize s)
     ;; openpose_ros_msgs/BodypartDetection_3d _chest
       (send _chest :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _num_people_detected
     (setq _num_people_detected (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _person_ID
     (setq _person_ID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; openpose_ros_msgs/BodypartDetection_3d _nose
     (send _nose :deserialize buf ptr-) (incf ptr- (send _nose :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _neck
     (send _neck :deserialize buf ptr-) (incf ptr- (send _neck :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _right_shoulder
     (send _right_shoulder :deserialize buf ptr-) (incf ptr- (send _right_shoulder :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _right_elbow
     (send _right_elbow :deserialize buf ptr-) (incf ptr- (send _right_elbow :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _right_wrist
     (send _right_wrist :deserialize buf ptr-) (incf ptr- (send _right_wrist :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _left_shoulder
     (send _left_shoulder :deserialize buf ptr-) (incf ptr- (send _left_shoulder :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _left_elbow
     (send _left_elbow :deserialize buf ptr-) (incf ptr- (send _left_elbow :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _left_wrist
     (send _left_wrist :deserialize buf ptr-) (incf ptr- (send _left_wrist :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _right_hip
     (send _right_hip :deserialize buf ptr-) (incf ptr- (send _right_hip :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _right_knee
     (send _right_knee :deserialize buf ptr-) (incf ptr- (send _right_knee :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _right_ankle
     (send _right_ankle :deserialize buf ptr-) (incf ptr- (send _right_ankle :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _left_hip
     (send _left_hip :deserialize buf ptr-) (incf ptr- (send _left_hip :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _left_knee
     (send _left_knee :deserialize buf ptr-) (incf ptr- (send _left_knee :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _left_ankle
     (send _left_ankle :deserialize buf ptr-) (incf ptr- (send _left_ankle :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _right_eye
     (send _right_eye :deserialize buf ptr-) (incf ptr- (send _right_eye :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _left_eye
     (send _left_eye :deserialize buf ptr-) (incf ptr- (send _left_eye :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _right_ear
     (send _right_ear :deserialize buf ptr-) (incf ptr- (send _right_ear :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _left_ear
     (send _left_ear :deserialize buf ptr-) (incf ptr- (send _left_ear :serialization-length))
   ;; openpose_ros_msgs/BodypartDetection_3d _chest
     (send _chest :deserialize buf ptr-) (incf ptr- (send _chest :serialization-length))
   ;;
   self)
  )

(setf (get openpose_ros_msgs::PersonDetection_3d :md5sum-) "d0a8af45cfc195d0d86994e1fdd9377d")
(setf (get openpose_ros_msgs::PersonDetection_3d :datatype-) "openpose_ros_msgs/PersonDetection_3d")
(setf (get openpose_ros_msgs::PersonDetection_3d :definition-)
      "uint32 num_people_detected
uint32 person_ID
BodypartDetection_3d nose
BodypartDetection_3d neck
BodypartDetection_3d right_shoulder
BodypartDetection_3d right_elbow
BodypartDetection_3d right_wrist
BodypartDetection_3d left_shoulder
BodypartDetection_3d left_elbow
BodypartDetection_3d left_wrist
BodypartDetection_3d right_hip
BodypartDetection_3d right_knee
BodypartDetection_3d right_ankle
BodypartDetection_3d left_hip
BodypartDetection_3d left_knee
BodypartDetection_3d left_ankle
BodypartDetection_3d right_eye
BodypartDetection_3d left_eye
BodypartDetection_3d right_ear
BodypartDetection_3d left_ear
BodypartDetection_3d chest

================================================================================
MSG: openpose_ros_msgs/BodypartDetection_3d
float32 x
float32 y
float32 z
float32 confidence

")



(provide :openpose_ros_msgs/PersonDetection_3d "d0a8af45cfc195d0d86994e1fdd9377d")


