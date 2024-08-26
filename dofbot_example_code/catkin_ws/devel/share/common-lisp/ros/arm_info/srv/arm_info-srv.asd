
(cl:in-package :asdf)

(defsystem "arm_info-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "kinemarics" :depends-on ("_package_kinemarics"))
    (:file "_package_kinemarics" :depends-on ("_package"))
  ))