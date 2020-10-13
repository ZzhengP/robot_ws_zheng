
(cl:in-package :asdf)

(defsystem "kinect2_tracker-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "bounding_box" :depends-on ("_package_bounding_box"))
    (:file "_package_bounding_box" :depends-on ("_package"))
    (:file "user_IDs" :depends-on ("_package_user_IDs"))
    (:file "_package_user_IDs" :depends-on ("_package"))
    (:file "user_points" :depends-on ("_package_user_points"))
    (:file "_package_user_points" :depends-on ("_package"))
  ))