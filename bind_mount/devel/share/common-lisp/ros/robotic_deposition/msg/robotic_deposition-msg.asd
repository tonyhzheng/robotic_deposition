
(cl:in-package :asdf)

(defsystem "robotic_deposition-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ur5e_control" :depends-on ("_package_ur5e_control"))
    (:file "_package_ur5e_control" :depends-on ("_package"))
    (:file "ur5e_data" :depends-on ("_package_ur5e_data"))
    (:file "_package_ur5e_data" :depends-on ("_package"))
  ))