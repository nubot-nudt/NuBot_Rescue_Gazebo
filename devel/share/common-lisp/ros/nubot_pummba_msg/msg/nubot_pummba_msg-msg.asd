
(cl:in-package :asdf)

(defsystem "nubot_pummba_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PummbaCmd" :depends-on ("_package_PummbaCmd"))
    (:file "_package_PummbaCmd" :depends-on ("_package"))
  ))