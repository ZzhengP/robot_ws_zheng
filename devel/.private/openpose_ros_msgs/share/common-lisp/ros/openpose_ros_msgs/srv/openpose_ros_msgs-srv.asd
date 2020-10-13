
(cl:in-package :asdf)

(defsystem "openpose_ros_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :openpose_ros_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "GetPersons" :depends-on ("_package_GetPersons"))
    (:file "_package_GetPersons" :depends-on ("_package"))
  ))