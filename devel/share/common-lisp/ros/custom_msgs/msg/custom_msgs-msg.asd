
(cl:in-package :asdf)

(defsystem "custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "IntList" :depends-on ("_package_IntList"))
    (:file "_package_IntList" :depends-on ("_package"))
  ))