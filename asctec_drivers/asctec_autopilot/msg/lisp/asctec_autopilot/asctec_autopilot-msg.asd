
(in-package :asdf)

(defsystem "asctec_autopilot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "IMUCalcData" :depends-on ("_package"))
    (:file "_package_IMUCalcData" :depends-on ("_package"))
    (:file "LLStatus" :depends-on ("_package"))
    (:file "_package_LLStatus" :depends-on ("_package"))
    ))
