
(cl:in-package :asdf)

(defsystem "openpose_ros_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BoundingBox" :depends-on ("_package_BoundingBox"))
    (:file "_package_BoundingBox" :depends-on ("_package"))
    (:file "OpenPoseHuman" :depends-on ("_package_OpenPoseHuman"))
    (:file "_package_OpenPoseHuman" :depends-on ("_package"))
    (:file "OpenPoseHumanList" :depends-on ("_package_OpenPoseHumanList"))
    (:file "_package_OpenPoseHumanList" :depends-on ("_package"))
    (:file "PointWithProb" :depends-on ("_package_PointWithProb"))
    (:file "_package_PointWithProb" :depends-on ("_package"))
  ))