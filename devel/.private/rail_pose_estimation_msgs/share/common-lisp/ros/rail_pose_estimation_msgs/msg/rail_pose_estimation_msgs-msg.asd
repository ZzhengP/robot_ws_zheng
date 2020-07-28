
(cl:in-package :asdf)

(defsystem "rail_pose_estimation_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Keypoints" :depends-on ("_package_Keypoints"))
    (:file "_package_Keypoints" :depends-on ("_package"))
    (:file "Poses" :depends-on ("_package_Poses"))
    (:file "_package_Poses" :depends-on ("_package"))
  ))