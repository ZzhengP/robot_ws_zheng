;; Auto-generated. Do not edit!


(when (boundp 'rail_pose_estimation_msgs::Keypoints)
  (if (not (find-package "RAIL_POSE_ESTIMATION_MSGS"))
    (make-package "RAIL_POSE_ESTIMATION_MSGS"))
  (shadow 'Keypoints (find-package "RAIL_POSE_ESTIMATION_MSGS")))
(unless (find-package "RAIL_POSE_ESTIMATION_MSGS::KEYPOINTS")
  (make-package "RAIL_POSE_ESTIMATION_MSGS::KEYPOINTS"))

(in-package "ROS")
;;//! \htmlinclude Keypoints.msg.html


(defclass rail_pose_estimation_msgs::Keypoints
  :super ros::object
  :slots (_neck_x _neck_y _nose_x _nose_y _right_shoulder_x _right_shoulder_y _left_shoulder_x _left_shoulder_y _right_elbow_x _right_elbow_y _left_elbow_x _left_elbow_y _right_wrist_x _right_wrist_y _left_wrist_x _left_wrist_y _right_hip_x _right_hip_y _left_hip_x _left_hip_y _right_knee_x _right_knee_y _left_knee_x _left_knee_y _right_ankle_x _right_ankle_y _left_ankle_x _left_ankle_y _right_eye_x _right_eye_y _left_eye_x _left_eye_y _right_ear_x _right_ear_y _left_ear_x _left_ear_y ))

(defmethod rail_pose_estimation_msgs::Keypoints
  (:init
   (&key
    ((:neck_x __neck_x) 0.0)
    ((:neck_y __neck_y) 0.0)
    ((:nose_x __nose_x) 0.0)
    ((:nose_y __nose_y) 0.0)
    ((:right_shoulder_x __right_shoulder_x) 0.0)
    ((:right_shoulder_y __right_shoulder_y) 0.0)
    ((:left_shoulder_x __left_shoulder_x) 0.0)
    ((:left_shoulder_y __left_shoulder_y) 0.0)
    ((:right_elbow_x __right_elbow_x) 0.0)
    ((:right_elbow_y __right_elbow_y) 0.0)
    ((:left_elbow_x __left_elbow_x) 0.0)
    ((:left_elbow_y __left_elbow_y) 0.0)
    ((:right_wrist_x __right_wrist_x) 0.0)
    ((:right_wrist_y __right_wrist_y) 0.0)
    ((:left_wrist_x __left_wrist_x) 0.0)
    ((:left_wrist_y __left_wrist_y) 0.0)
    ((:right_hip_x __right_hip_x) 0.0)
    ((:right_hip_y __right_hip_y) 0.0)
    ((:left_hip_x __left_hip_x) 0.0)
    ((:left_hip_y __left_hip_y) 0.0)
    ((:right_knee_x __right_knee_x) 0.0)
    ((:right_knee_y __right_knee_y) 0.0)
    ((:left_knee_x __left_knee_x) 0.0)
    ((:left_knee_y __left_knee_y) 0.0)
    ((:right_ankle_x __right_ankle_x) 0.0)
    ((:right_ankle_y __right_ankle_y) 0.0)
    ((:left_ankle_x __left_ankle_x) 0.0)
    ((:left_ankle_y __left_ankle_y) 0.0)
    ((:right_eye_x __right_eye_x) 0.0)
    ((:right_eye_y __right_eye_y) 0.0)
    ((:left_eye_x __left_eye_x) 0.0)
    ((:left_eye_y __left_eye_y) 0.0)
    ((:right_ear_x __right_ear_x) 0.0)
    ((:right_ear_y __right_ear_y) 0.0)
    ((:left_ear_x __left_ear_x) 0.0)
    ((:left_ear_y __left_ear_y) 0.0)
    )
   (send-super :init)
   (setq _neck_x (float __neck_x))
   (setq _neck_y (float __neck_y))
   (setq _nose_x (float __nose_x))
   (setq _nose_y (float __nose_y))
   (setq _right_shoulder_x (float __right_shoulder_x))
   (setq _right_shoulder_y (float __right_shoulder_y))
   (setq _left_shoulder_x (float __left_shoulder_x))
   (setq _left_shoulder_y (float __left_shoulder_y))
   (setq _right_elbow_x (float __right_elbow_x))
   (setq _right_elbow_y (float __right_elbow_y))
   (setq _left_elbow_x (float __left_elbow_x))
   (setq _left_elbow_y (float __left_elbow_y))
   (setq _right_wrist_x (float __right_wrist_x))
   (setq _right_wrist_y (float __right_wrist_y))
   (setq _left_wrist_x (float __left_wrist_x))
   (setq _left_wrist_y (float __left_wrist_y))
   (setq _right_hip_x (float __right_hip_x))
   (setq _right_hip_y (float __right_hip_y))
   (setq _left_hip_x (float __left_hip_x))
   (setq _left_hip_y (float __left_hip_y))
   (setq _right_knee_x (float __right_knee_x))
   (setq _right_knee_y (float __right_knee_y))
   (setq _left_knee_x (float __left_knee_x))
   (setq _left_knee_y (float __left_knee_y))
   (setq _right_ankle_x (float __right_ankle_x))
   (setq _right_ankle_y (float __right_ankle_y))
   (setq _left_ankle_x (float __left_ankle_x))
   (setq _left_ankle_y (float __left_ankle_y))
   (setq _right_eye_x (float __right_eye_x))
   (setq _right_eye_y (float __right_eye_y))
   (setq _left_eye_x (float __left_eye_x))
   (setq _left_eye_y (float __left_eye_y))
   (setq _right_ear_x (float __right_ear_x))
   (setq _right_ear_y (float __right_ear_y))
   (setq _left_ear_x (float __left_ear_x))
   (setq _left_ear_y (float __left_ear_y))
   self)
  (:neck_x
   (&optional __neck_x)
   (if __neck_x (setq _neck_x __neck_x)) _neck_x)
  (:neck_y
   (&optional __neck_y)
   (if __neck_y (setq _neck_y __neck_y)) _neck_y)
  (:nose_x
   (&optional __nose_x)
   (if __nose_x (setq _nose_x __nose_x)) _nose_x)
  (:nose_y
   (&optional __nose_y)
   (if __nose_y (setq _nose_y __nose_y)) _nose_y)
  (:right_shoulder_x
   (&optional __right_shoulder_x)
   (if __right_shoulder_x (setq _right_shoulder_x __right_shoulder_x)) _right_shoulder_x)
  (:right_shoulder_y
   (&optional __right_shoulder_y)
   (if __right_shoulder_y (setq _right_shoulder_y __right_shoulder_y)) _right_shoulder_y)
  (:left_shoulder_x
   (&optional __left_shoulder_x)
   (if __left_shoulder_x (setq _left_shoulder_x __left_shoulder_x)) _left_shoulder_x)
  (:left_shoulder_y
   (&optional __left_shoulder_y)
   (if __left_shoulder_y (setq _left_shoulder_y __left_shoulder_y)) _left_shoulder_y)
  (:right_elbow_x
   (&optional __right_elbow_x)
   (if __right_elbow_x (setq _right_elbow_x __right_elbow_x)) _right_elbow_x)
  (:right_elbow_y
   (&optional __right_elbow_y)
   (if __right_elbow_y (setq _right_elbow_y __right_elbow_y)) _right_elbow_y)
  (:left_elbow_x
   (&optional __left_elbow_x)
   (if __left_elbow_x (setq _left_elbow_x __left_elbow_x)) _left_elbow_x)
  (:left_elbow_y
   (&optional __left_elbow_y)
   (if __left_elbow_y (setq _left_elbow_y __left_elbow_y)) _left_elbow_y)
  (:right_wrist_x
   (&optional __right_wrist_x)
   (if __right_wrist_x (setq _right_wrist_x __right_wrist_x)) _right_wrist_x)
  (:right_wrist_y
   (&optional __right_wrist_y)
   (if __right_wrist_y (setq _right_wrist_y __right_wrist_y)) _right_wrist_y)
  (:left_wrist_x
   (&optional __left_wrist_x)
   (if __left_wrist_x (setq _left_wrist_x __left_wrist_x)) _left_wrist_x)
  (:left_wrist_y
   (&optional __left_wrist_y)
   (if __left_wrist_y (setq _left_wrist_y __left_wrist_y)) _left_wrist_y)
  (:right_hip_x
   (&optional __right_hip_x)
   (if __right_hip_x (setq _right_hip_x __right_hip_x)) _right_hip_x)
  (:right_hip_y
   (&optional __right_hip_y)
   (if __right_hip_y (setq _right_hip_y __right_hip_y)) _right_hip_y)
  (:left_hip_x
   (&optional __left_hip_x)
   (if __left_hip_x (setq _left_hip_x __left_hip_x)) _left_hip_x)
  (:left_hip_y
   (&optional __left_hip_y)
   (if __left_hip_y (setq _left_hip_y __left_hip_y)) _left_hip_y)
  (:right_knee_x
   (&optional __right_knee_x)
   (if __right_knee_x (setq _right_knee_x __right_knee_x)) _right_knee_x)
  (:right_knee_y
   (&optional __right_knee_y)
   (if __right_knee_y (setq _right_knee_y __right_knee_y)) _right_knee_y)
  (:left_knee_x
   (&optional __left_knee_x)
   (if __left_knee_x (setq _left_knee_x __left_knee_x)) _left_knee_x)
  (:left_knee_y
   (&optional __left_knee_y)
   (if __left_knee_y (setq _left_knee_y __left_knee_y)) _left_knee_y)
  (:right_ankle_x
   (&optional __right_ankle_x)
   (if __right_ankle_x (setq _right_ankle_x __right_ankle_x)) _right_ankle_x)
  (:right_ankle_y
   (&optional __right_ankle_y)
   (if __right_ankle_y (setq _right_ankle_y __right_ankle_y)) _right_ankle_y)
  (:left_ankle_x
   (&optional __left_ankle_x)
   (if __left_ankle_x (setq _left_ankle_x __left_ankle_x)) _left_ankle_x)
  (:left_ankle_y
   (&optional __left_ankle_y)
   (if __left_ankle_y (setq _left_ankle_y __left_ankle_y)) _left_ankle_y)
  (:right_eye_x
   (&optional __right_eye_x)
   (if __right_eye_x (setq _right_eye_x __right_eye_x)) _right_eye_x)
  (:right_eye_y
   (&optional __right_eye_y)
   (if __right_eye_y (setq _right_eye_y __right_eye_y)) _right_eye_y)
  (:left_eye_x
   (&optional __left_eye_x)
   (if __left_eye_x (setq _left_eye_x __left_eye_x)) _left_eye_x)
  (:left_eye_y
   (&optional __left_eye_y)
   (if __left_eye_y (setq _left_eye_y __left_eye_y)) _left_eye_y)
  (:right_ear_x
   (&optional __right_ear_x)
   (if __right_ear_x (setq _right_ear_x __right_ear_x)) _right_ear_x)
  (:right_ear_y
   (&optional __right_ear_y)
   (if __right_ear_y (setq _right_ear_y __right_ear_y)) _right_ear_y)
  (:left_ear_x
   (&optional __left_ear_x)
   (if __left_ear_x (setq _left_ear_x __left_ear_x)) _left_ear_x)
  (:left_ear_y
   (&optional __left_ear_y)
   (if __left_ear_y (setq _left_ear_y __left_ear_y)) _left_ear_y)
  (:serialization-length
   ()
   (+
    ;; float32 _neck_x
    4
    ;; float32 _neck_y
    4
    ;; float32 _nose_x
    4
    ;; float32 _nose_y
    4
    ;; float32 _right_shoulder_x
    4
    ;; float32 _right_shoulder_y
    4
    ;; float32 _left_shoulder_x
    4
    ;; float32 _left_shoulder_y
    4
    ;; float32 _right_elbow_x
    4
    ;; float32 _right_elbow_y
    4
    ;; float32 _left_elbow_x
    4
    ;; float32 _left_elbow_y
    4
    ;; float32 _right_wrist_x
    4
    ;; float32 _right_wrist_y
    4
    ;; float32 _left_wrist_x
    4
    ;; float32 _left_wrist_y
    4
    ;; float32 _right_hip_x
    4
    ;; float32 _right_hip_y
    4
    ;; float32 _left_hip_x
    4
    ;; float32 _left_hip_y
    4
    ;; float32 _right_knee_x
    4
    ;; float32 _right_knee_y
    4
    ;; float32 _left_knee_x
    4
    ;; float32 _left_knee_y
    4
    ;; float32 _right_ankle_x
    4
    ;; float32 _right_ankle_y
    4
    ;; float32 _left_ankle_x
    4
    ;; float32 _left_ankle_y
    4
    ;; float32 _right_eye_x
    4
    ;; float32 _right_eye_y
    4
    ;; float32 _left_eye_x
    4
    ;; float32 _left_eye_y
    4
    ;; float32 _right_ear_x
    4
    ;; float32 _right_ear_y
    4
    ;; float32 _left_ear_x
    4
    ;; float32 _left_ear_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _neck_x
       (sys::poke _neck_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _neck_y
       (sys::poke _neck_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _nose_x
       (sys::poke _nose_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _nose_y
       (sys::poke _nose_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_shoulder_x
       (sys::poke _right_shoulder_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_shoulder_y
       (sys::poke _right_shoulder_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_shoulder_x
       (sys::poke _left_shoulder_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_shoulder_y
       (sys::poke _left_shoulder_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_elbow_x
       (sys::poke _right_elbow_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_elbow_y
       (sys::poke _right_elbow_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_elbow_x
       (sys::poke _left_elbow_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_elbow_y
       (sys::poke _left_elbow_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_wrist_x
       (sys::poke _right_wrist_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_wrist_y
       (sys::poke _right_wrist_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_wrist_x
       (sys::poke _left_wrist_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_wrist_y
       (sys::poke _left_wrist_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_hip_x
       (sys::poke _right_hip_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_hip_y
       (sys::poke _right_hip_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_hip_x
       (sys::poke _left_hip_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_hip_y
       (sys::poke _left_hip_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_knee_x
       (sys::poke _right_knee_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_knee_y
       (sys::poke _right_knee_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_knee_x
       (sys::poke _left_knee_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_knee_y
       (sys::poke _left_knee_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_ankle_x
       (sys::poke _right_ankle_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_ankle_y
       (sys::poke _right_ankle_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_ankle_x
       (sys::poke _left_ankle_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_ankle_y
       (sys::poke _left_ankle_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_eye_x
       (sys::poke _right_eye_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_eye_y
       (sys::poke _right_eye_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_eye_x
       (sys::poke _left_eye_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_eye_y
       (sys::poke _left_eye_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_ear_x
       (sys::poke _right_ear_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_ear_y
       (sys::poke _right_ear_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_ear_x
       (sys::poke _left_ear_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_ear_y
       (sys::poke _left_ear_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _neck_x
     (setq _neck_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _neck_y
     (setq _neck_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _nose_x
     (setq _nose_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _nose_y
     (setq _nose_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_shoulder_x
     (setq _right_shoulder_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_shoulder_y
     (setq _right_shoulder_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_shoulder_x
     (setq _left_shoulder_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_shoulder_y
     (setq _left_shoulder_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_elbow_x
     (setq _right_elbow_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_elbow_y
     (setq _right_elbow_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_elbow_x
     (setq _left_elbow_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_elbow_y
     (setq _left_elbow_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_wrist_x
     (setq _right_wrist_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_wrist_y
     (setq _right_wrist_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_wrist_x
     (setq _left_wrist_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_wrist_y
     (setq _left_wrist_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_hip_x
     (setq _right_hip_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_hip_y
     (setq _right_hip_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_hip_x
     (setq _left_hip_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_hip_y
     (setq _left_hip_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_knee_x
     (setq _right_knee_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_knee_y
     (setq _right_knee_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_knee_x
     (setq _left_knee_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_knee_y
     (setq _left_knee_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_ankle_x
     (setq _right_ankle_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_ankle_y
     (setq _right_ankle_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_ankle_x
     (setq _left_ankle_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_ankle_y
     (setq _left_ankle_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_eye_x
     (setq _right_eye_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_eye_y
     (setq _right_eye_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_eye_x
     (setq _left_eye_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_eye_y
     (setq _left_eye_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_ear_x
     (setq _right_ear_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_ear_y
     (setq _right_ear_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_ear_x
     (setq _left_ear_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_ear_y
     (setq _left_ear_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rail_pose_estimation_msgs::Keypoints :md5sum-) "3d1804a99352b413ee0c9ca364640114")
(setf (get rail_pose_estimation_msgs::Keypoints :datatype-) "rail_pose_estimation_msgs/Keypoints")
(setf (get rail_pose_estimation_msgs::Keypoints :definition-)
      "float32 neck_x                  # x coord of neck
float32 neck_y                  # y coord of neck
float32 nose_x                  # x coord of nose
float32 nose_y                  # y coord of nose
float32 right_shoulder_x        # x coord of right shoulder
float32 right_shoulder_y        # y coord of right shoulder
float32 left_shoulder_x         # x coord of left shoulder
float32 left_shoulder_y         # y coord of left shoulder
float32 right_elbow_x           # x coord of right elbow
float32 right_elbow_y           # y coord of right elbow
float32 left_elbow_x            # x coord of left elbow
float32 left_elbow_y            # y coord of left elbow
float32 right_wrist_x           # x coord of right wrist
float32 right_wrist_y           # y coord of right wrist
float32 left_wrist_x            # x coord of left wrist
float32 left_wrist_y            # y coord of left wrist
float32 right_hip_x             # x coord of right hip
float32 right_hip_y             # y coord of right hip
float32 left_hip_x              # x coord of left hip
float32 left_hip_y              # y coord of left hip
float32 right_knee_x            # x coord of right knee
float32 right_knee_y            # y coord of right knee
float32 left_knee_x             # x coord of left knee
float32 left_knee_y             # y coord of left knee
float32 right_ankle_x           # x coord of right ankle
float32 right_ankle_y           # y coord of right ankle
float32 left_ankle_x            # x coord of left ankle
float32 left_ankle_y            # y coord of left ankle
float32 right_eye_x             # x coord of right eye
float32 right_eye_y             # y coord of right eye
float32 left_eye_x              # x coord of left eye
float32 left_eye_y              # y coord of left eye
float32 right_ear_x             # x coord of right ear
float32 right_ear_y             # y coord of right ear
float32 left_ear_x              # x coord of left ear
float32 left_ear_y              # y coord of left ear
")



(provide :rail_pose_estimation_msgs/Keypoints "3d1804a99352b413ee0c9ca364640114")


