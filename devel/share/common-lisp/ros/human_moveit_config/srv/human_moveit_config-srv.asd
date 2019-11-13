
(cl:in-package :asdf)

(defsystem "human_moveit_config-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "GetHumanIK" :depends-on ("_package_GetHumanIK"))
    (:file "_package_GetHumanIK" :depends-on ("_package"))
    (:file "GetJacobian" :depends-on ("_package_GetJacobian"))
    (:file "_package_GetJacobian" :depends-on ("_package"))
  ))