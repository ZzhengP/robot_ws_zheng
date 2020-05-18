;; Auto-generated. Do not edit!


(when (boundp 'openpose_ros_msgs::OpenPoseHuman)
  (if (not (find-package "OPENPOSE_ROS_MSGS"))
    (make-package "OPENPOSE_ROS_MSGS"))
  (shadow 'OpenPoseHuman (find-package "OPENPOSE_ROS_MSGS")))
(unless (find-package "OPENPOSE_ROS_MSGS::OPENPOSEHUMAN")
  (make-package "OPENPOSE_ROS_MSGS::OPENPOSEHUMAN"))

(in-package "ROS")
;;//! \htmlinclude OpenPoseHuman.msg.html


(defclass openpose_ros_msgs::OpenPoseHuman
  :super ros::object
  :slots (_num_body_key_points_with_non_zero_prob _num_face_key_points_with_non_zero_prob _num_right_hand_key_points_with_non_zero_prob _num_left_hand_key_points_with_non_zero_prob _body_bounding_box _face_bounding_box _body_key_points_with_prob _face_key_points_with_prob _right_hand_key_points_with_prob _left_hand_key_points_with_prob ))

(defmethod openpose_ros_msgs::OpenPoseHuman
  (:init
   (&key
    ((:num_body_key_points_with_non_zero_prob __num_body_key_points_with_non_zero_prob) 0)
    ((:num_face_key_points_with_non_zero_prob __num_face_key_points_with_non_zero_prob) 0)
    ((:num_right_hand_key_points_with_non_zero_prob __num_right_hand_key_points_with_non_zero_prob) 0)
    ((:num_left_hand_key_points_with_non_zero_prob __num_left_hand_key_points_with_non_zero_prob) 0)
    ((:body_bounding_box __body_bounding_box) (instance openpose_ros_msgs::BoundingBox :init))
    ((:face_bounding_box __face_bounding_box) (instance openpose_ros_msgs::BoundingBox :init))
    ((:body_key_points_with_prob __body_key_points_with_prob) (let (r) (dotimes (i 25) (push (instance openpose_ros_msgs::PointWithProb :init) r)) r))
    ((:face_key_points_with_prob __face_key_points_with_prob) (let (r) (dotimes (i 70) (push (instance openpose_ros_msgs::PointWithProb :init) r)) r))
    ((:right_hand_key_points_with_prob __right_hand_key_points_with_prob) (let (r) (dotimes (i 21) (push (instance openpose_ros_msgs::PointWithProb :init) r)) r))
    ((:left_hand_key_points_with_prob __left_hand_key_points_with_prob) (let (r) (dotimes (i 21) (push (instance openpose_ros_msgs::PointWithProb :init) r)) r))
    )
   (send-super :init)
   (setq _num_body_key_points_with_non_zero_prob (round __num_body_key_points_with_non_zero_prob))
   (setq _num_face_key_points_with_non_zero_prob (round __num_face_key_points_with_non_zero_prob))
   (setq _num_right_hand_key_points_with_non_zero_prob (round __num_right_hand_key_points_with_non_zero_prob))
   (setq _num_left_hand_key_points_with_non_zero_prob (round __num_left_hand_key_points_with_non_zero_prob))
   (setq _body_bounding_box __body_bounding_box)
   (setq _face_bounding_box __face_bounding_box)
   (setq _body_key_points_with_prob __body_key_points_with_prob)
   (setq _face_key_points_with_prob __face_key_points_with_prob)
   (setq _right_hand_key_points_with_prob __right_hand_key_points_with_prob)
   (setq _left_hand_key_points_with_prob __left_hand_key_points_with_prob)
   self)
  (:num_body_key_points_with_non_zero_prob
   (&optional __num_body_key_points_with_non_zero_prob)
   (if __num_body_key_points_with_non_zero_prob (setq _num_body_key_points_with_non_zero_prob __num_body_key_points_with_non_zero_prob)) _num_body_key_points_with_non_zero_prob)
  (:num_face_key_points_with_non_zero_prob
   (&optional __num_face_key_points_with_non_zero_prob)
   (if __num_face_key_points_with_non_zero_prob (setq _num_face_key_points_with_non_zero_prob __num_face_key_points_with_non_zero_prob)) _num_face_key_points_with_non_zero_prob)
  (:num_right_hand_key_points_with_non_zero_prob
   (&optional __num_right_hand_key_points_with_non_zero_prob)
   (if __num_right_hand_key_points_with_non_zero_prob (setq _num_right_hand_key_points_with_non_zero_prob __num_right_hand_key_points_with_non_zero_prob)) _num_right_hand_key_points_with_non_zero_prob)
  (:num_left_hand_key_points_with_non_zero_prob
   (&optional __num_left_hand_key_points_with_non_zero_prob)
   (if __num_left_hand_key_points_with_non_zero_prob (setq _num_left_hand_key_points_with_non_zero_prob __num_left_hand_key_points_with_non_zero_prob)) _num_left_hand_key_points_with_non_zero_prob)
  (:body_bounding_box
   (&rest __body_bounding_box)
   (if (keywordp (car __body_bounding_box))
       (send* _body_bounding_box __body_bounding_box)
     (progn
       (if __body_bounding_box (setq _body_bounding_box (car __body_bounding_box)))
       _body_bounding_box)))
  (:face_bounding_box
   (&rest __face_bounding_box)
   (if (keywordp (car __face_bounding_box))
       (send* _face_bounding_box __face_bounding_box)
     (progn
       (if __face_bounding_box (setq _face_bounding_box (car __face_bounding_box)))
       _face_bounding_box)))
  (:body_key_points_with_prob
   (&rest __body_key_points_with_prob)
   (if (keywordp (car __body_key_points_with_prob))
       (send* _body_key_points_with_prob __body_key_points_with_prob)
     (progn
       (if __body_key_points_with_prob (setq _body_key_points_with_prob (car __body_key_points_with_prob)))
       _body_key_points_with_prob)))
  (:face_key_points_with_prob
   (&rest __face_key_points_with_prob)
   (if (keywordp (car __face_key_points_with_prob))
       (send* _face_key_points_with_prob __face_key_points_with_prob)
     (progn
       (if __face_key_points_with_prob (setq _face_key_points_with_prob (car __face_key_points_with_prob)))
       _face_key_points_with_prob)))
  (:right_hand_key_points_with_prob
   (&rest __right_hand_key_points_with_prob)
   (if (keywordp (car __right_hand_key_points_with_prob))
       (send* _right_hand_key_points_with_prob __right_hand_key_points_with_prob)
     (progn
       (if __right_hand_key_points_with_prob (setq _right_hand_key_points_with_prob (car __right_hand_key_points_with_prob)))
       _right_hand_key_points_with_prob)))
  (:left_hand_key_points_with_prob
   (&rest __left_hand_key_points_with_prob)
   (if (keywordp (car __left_hand_key_points_with_prob))
       (send* _left_hand_key_points_with_prob __left_hand_key_points_with_prob)
     (progn
       (if __left_hand_key_points_with_prob (setq _left_hand_key_points_with_prob (car __left_hand_key_points_with_prob)))
       _left_hand_key_points_with_prob)))
  (:serialization-length
   ()
   (+
    ;; int32 _num_body_key_points_with_non_zero_prob
    4
    ;; int32 _num_face_key_points_with_non_zero_prob
    4
    ;; int32 _num_right_hand_key_points_with_non_zero_prob
    4
    ;; int32 _num_left_hand_key_points_with_non_zero_prob
    4
    ;; openpose_ros_msgs/BoundingBox _body_bounding_box
    (send _body_bounding_box :serialization-length)
    ;; openpose_ros_msgs/BoundingBox _face_bounding_box
    (send _face_bounding_box :serialization-length)
    ;; openpose_ros_msgs/PointWithProb[25] _body_key_points_with_prob
    (apply #'+ (send-all _body_key_points_with_prob :serialization-length))
    ;; openpose_ros_msgs/PointWithProb[70] _face_key_points_with_prob
    (apply #'+ (send-all _face_key_points_with_prob :serialization-length))
    ;; openpose_ros_msgs/PointWithProb[21] _right_hand_key_points_with_prob
    (apply #'+ (send-all _right_hand_key_points_with_prob :serialization-length))
    ;; openpose_ros_msgs/PointWithProb[21] _left_hand_key_points_with_prob
    (apply #'+ (send-all _left_hand_key_points_with_prob :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _num_body_key_points_with_non_zero_prob
       (write-long _num_body_key_points_with_non_zero_prob s)
     ;; int32 _num_face_key_points_with_non_zero_prob
       (write-long _num_face_key_points_with_non_zero_prob s)
     ;; int32 _num_right_hand_key_points_with_non_zero_prob
       (write-long _num_right_hand_key_points_with_non_zero_prob s)
     ;; int32 _num_left_hand_key_points_with_non_zero_prob
       (write-long _num_left_hand_key_points_with_non_zero_prob s)
     ;; openpose_ros_msgs/BoundingBox _body_bounding_box
       (send _body_bounding_box :serialize s)
     ;; openpose_ros_msgs/BoundingBox _face_bounding_box
       (send _face_bounding_box :serialize s)
     ;; openpose_ros_msgs/PointWithProb[25] _body_key_points_with_prob
     (dolist (elem _body_key_points_with_prob)
       (send elem :serialize s)
       )
     ;; openpose_ros_msgs/PointWithProb[70] _face_key_points_with_prob
     (dolist (elem _face_key_points_with_prob)
       (send elem :serialize s)
       )
     ;; openpose_ros_msgs/PointWithProb[21] _right_hand_key_points_with_prob
     (dolist (elem _right_hand_key_points_with_prob)
       (send elem :serialize s)
       )
     ;; openpose_ros_msgs/PointWithProb[21] _left_hand_key_points_with_prob
     (dolist (elem _left_hand_key_points_with_prob)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _num_body_key_points_with_non_zero_prob
     (setq _num_body_key_points_with_non_zero_prob (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num_face_key_points_with_non_zero_prob
     (setq _num_face_key_points_with_non_zero_prob (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num_right_hand_key_points_with_non_zero_prob
     (setq _num_right_hand_key_points_with_non_zero_prob (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num_left_hand_key_points_with_non_zero_prob
     (setq _num_left_hand_key_points_with_non_zero_prob (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; openpose_ros_msgs/BoundingBox _body_bounding_box
     (send _body_bounding_box :deserialize buf ptr-) (incf ptr- (send _body_bounding_box :serialization-length))
   ;; openpose_ros_msgs/BoundingBox _face_bounding_box
     (send _face_bounding_box :deserialize buf ptr-) (incf ptr- (send _face_bounding_box :serialization-length))
   ;; openpose_ros_msgs/PointWithProb[25] _body_key_points_with_prob
   (dotimes (i 25)
     (send (elt _body_key_points_with_prob i) :deserialize buf ptr-) (incf ptr- (send (elt _body_key_points_with_prob i) :serialization-length))
     )
   ;; openpose_ros_msgs/PointWithProb[70] _face_key_points_with_prob
   (dotimes (i 70)
     (send (elt _face_key_points_with_prob i) :deserialize buf ptr-) (incf ptr- (send (elt _face_key_points_with_prob i) :serialization-length))
     )
   ;; openpose_ros_msgs/PointWithProb[21] _right_hand_key_points_with_prob
   (dotimes (i 21)
     (send (elt _right_hand_key_points_with_prob i) :deserialize buf ptr-) (incf ptr- (send (elt _right_hand_key_points_with_prob i) :serialization-length))
     )
   ;; openpose_ros_msgs/PointWithProb[21] _left_hand_key_points_with_prob
   (dotimes (i 21)
     (send (elt _left_hand_key_points_with_prob i) :deserialize buf ptr-) (incf ptr- (send (elt _left_hand_key_points_with_prob i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get openpose_ros_msgs::OpenPoseHuman :md5sum-) "4e291ca1063e4a96ca852f33fa35d22b")
(setf (get openpose_ros_msgs::OpenPoseHuman :datatype-) "openpose_ros_msgs/OpenPoseHuman")
(setf (get openpose_ros_msgs::OpenPoseHuman :definition-)
      "int32 num_body_key_points_with_non_zero_prob
int32 num_face_key_points_with_non_zero_prob
int32 num_right_hand_key_points_with_non_zero_prob
int32 num_left_hand_key_points_with_non_zero_prob
BoundingBox body_bounding_box
BoundingBox face_bounding_box
PointWithProb[25] body_key_points_with_prob
PointWithProb[70] face_key_points_with_prob
PointWithProb[21] right_hand_key_points_with_prob
PointWithProb[21] left_hand_key_points_with_prob
================================================================================
MSG: openpose_ros_msgs/BoundingBox
float64 x
float64 y
float64 width
float64 height
================================================================================
MSG: openpose_ros_msgs/PointWithProb
float64 x
float64 y
float64 prob
")



(provide :openpose_ros_msgs/OpenPoseHuman "4e291ca1063e4a96ca852f33fa35d22b")


