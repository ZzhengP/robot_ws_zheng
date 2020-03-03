
(cl:in-package :asdf)

(defsystem "openpose_ros_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "BodyPartDetection" :depends-on ("_package_BodyPartDetection"))
    (:file "_package_BodyPartDetection" :depends-on ("_package"))
    (:file "PersonDetection" :depends-on ("_package_PersonDetection"))
    (:file "_package_PersonDetection" :depends-on ("_package"))
    (:file "Persons" :depends-on ("_package_Persons"))
    (:file "_package_Persons" :depends-on ("_package"))
  ))