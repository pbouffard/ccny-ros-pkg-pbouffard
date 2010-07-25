
(in-package :asdf)

(defsystem "asctec_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "IMUCalcData" :depends-on ("_package"))
    (:file "_package_IMUCalcData" :depends-on ("_package"))
    (:file "Height" :depends-on ("_package"))
    (:file "_package_Height" :depends-on ("_package"))
    (:file "LLStatus" :depends-on ("_package"))
    (:file "_package_LLStatus" :depends-on ("_package"))
    ))
