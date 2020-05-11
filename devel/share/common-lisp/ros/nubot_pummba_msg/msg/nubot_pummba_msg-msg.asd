
(cl:in-package :asdf)

(defsystem "nubot_pummba_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FlipCmd" :depends-on ("_package_FlipCmd"))
    (:file "_package_FlipCmd" :depends-on ("_package"))
  ))