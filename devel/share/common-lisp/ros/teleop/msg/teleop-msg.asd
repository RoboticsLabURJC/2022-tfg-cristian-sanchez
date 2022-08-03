
(cl:in-package :asdf)

(defsystem "teleop-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Px4Cmd" :depends-on ("_package_Px4Cmd"))
    (:file "_package_Px4Cmd" :depends-on ("_package"))
  ))