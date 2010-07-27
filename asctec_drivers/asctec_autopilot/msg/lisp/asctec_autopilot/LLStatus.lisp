; Auto-generated. Do not edit!


(in-package asctec_autopilot-msg)


;//! \htmlinclude LLStatus.msg.html

(defclass <LLStatus> (ros-message)
  ((battery_voltage_1
    :reader battery_voltage_1-val
    :initarg :battery_voltage_1
    :type fixnum
    :initform 0)
   (battery_voltage_2
    :reader battery_voltage_2-val
    :initarg :battery_voltage_2
    :type fixnum
    :initform 0)
   (status
    :reader status-val
    :initarg :status
    :type fixnum
    :initform 0)
   (cpu_load
    :reader cpu_load-val
    :initarg :cpu_load
    :type fixnum
    :initform 0)
   (compass_enabled
    :reader compass_enabled-val
    :initarg :compass_enabled
    :type fixnum
    :initform 0)
   (chksum_error
    :reader chksum_error-val
    :initarg :chksum_error
    :type fixnum
    :initform 0)
   (flying
    :reader flying-val
    :initarg :flying
    :type fixnum
    :initform 0)
   (motors_on
    :reader motors_on-val
    :initarg :motors_on
    :type fixnum
    :initform 0)
   (flightMode
    :reader flightMode-val
    :initarg :flightMode
    :type fixnum
    :initform 0)
   (up_time
    :reader up_time-val
    :initarg :up_time
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <LLStatus>) ostream)
  "Serializes a message object of type '<LLStatus>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'battery_voltage_1)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'battery_voltage_1)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'battery_voltage_2)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'battery_voltage_2)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'status)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'status)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'cpu_load)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'cpu_load)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'compass_enabled)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'chksum_error)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'flying)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'motors_on)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'flightMode)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'flightMode)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'up_time)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'up_time)) ostream)
)
(defmethod deserialize ((msg <LLStatus>) istream)
  "Deserializes a message object of type '<LLStatus>"
  (setf (ldb (byte 8 0) (slot-value msg 'battery_voltage_1)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'battery_voltage_1)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'battery_voltage_2)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'battery_voltage_2)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'status)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'status)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'cpu_load)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'cpu_load)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'compass_enabled)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'chksum_error)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'flying)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'motors_on)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'flightMode)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'flightMode)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'up_time)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'up_time)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<LLStatus>)))
  "Returns string type for a message object of type '<LLStatus>"
  "asctec_autopilot/LLStatus")
(defmethod md5sum ((type (eql '<LLStatus>)))
  "Returns md5sum for a message object of type '<LLStatus>"
  "2b44d15e58f4c5af0047794df9b66a84")
(defmethod message-definition ((type (eql '<LLStatus>)))
  "Returns full string definition for message of type '<LLStatus>"
  (format nil "# battery voltages in mV~%int16 battery_voltage_1~%int16 battery_voltage_2~%# dont care~%int16 status~%# Controller cycles per second (should be about 1000)~%int16 cpu_load~%# dont care~%int8 compass_enabled~%int8 chksum_error~%int8 flying~%int8 motors_on~%int16 flightMode~%# Time motors are turning~%int16 up_time~%~%~%~%"))
(defmethod serialization-length ((msg <LLStatus>))
  (+ 0
     2
     2
     2
     2
     1
     1
     1
     1
     2
     2
))
(defmethod ros-message-to-list ((msg <LLStatus>))
  "Converts a ROS message object to a list"
  (list '<LLStatus>
    (cons ':battery_voltage_1 (battery_voltage_1-val msg))
    (cons ':battery_voltage_2 (battery_voltage_2-val msg))
    (cons ':status (status-val msg))
    (cons ':cpu_load (cpu_load-val msg))
    (cons ':compass_enabled (compass_enabled-val msg))
    (cons ':chksum_error (chksum_error-val msg))
    (cons ':flying (flying-val msg))
    (cons ':motors_on (motors_on-val msg))
    (cons ':flightMode (flightMode-val msg))
    (cons ':up_time (up_time-val msg))
))
