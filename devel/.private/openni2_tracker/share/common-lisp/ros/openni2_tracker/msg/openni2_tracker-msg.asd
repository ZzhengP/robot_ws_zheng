
(cl:in-package :asdf)

(defsystem "openni2_tracker-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "TrackerUser" :depends-on ("_package_TrackerUser"))
    (:file "_package_TrackerUser" :depends-on ("_package"))
    (:file "TrackerUserArray" :depends-on ("_package_TrackerUserArray"))
    (:file "_package_TrackerUserArray" :depends-on ("_package"))
  ))