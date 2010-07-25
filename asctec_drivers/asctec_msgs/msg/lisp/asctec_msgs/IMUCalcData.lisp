; Auto-generated. Do not edit!


(in-package asctec_msgs-msg)


;//! \htmlinclude IMUCalcData.msg.html

(defclass <IMUCalcData> (ros-message)
  ((angle_nick
    :reader angle_nick-val
    :initarg :angle_nick
    :type integer
    :initform 0)
   (angle_roll
    :reader angle_roll-val
    :initarg :angle_roll
    :type integer
    :initform 0)
   (angle_yaw
    :reader angle_yaw-val
    :initarg :angle_yaw
    :type integer
    :initform 0)
   (angvel_nick
    :reader angvel_nick-val
    :initarg :angvel_nick
    :type integer
    :initform 0)
   (angvel_roll
    :reader angvel_roll-val
    :initarg :angvel_roll
    :type integer
    :initform 0)
   (angvel_yaw
    :reader angvel_yaw-val
    :initarg :angvel_yaw
    :type integer
    :initform 0)
   (acc_x_calib
    :reader acc_x_calib-val
    :initarg :acc_x_calib
    :type fixnum
    :initform 0)
   (acc_y_calib
    :reader acc_y_calib-val
    :initarg :acc_y_calib
    :type fixnum
    :initform 0)
   (acc_z_calib
    :reader acc_z_calib-val
    :initarg :acc_z_calib
    :type fixnum
    :initform 0)
   (acc_x
    :reader acc_x-val
    :initarg :acc_x
    :type fixnum
    :initform 0)
   (acc_y
    :reader acc_y-val
    :initarg :acc_y
    :type fixnum
    :initform 0)
   (acc_z
    :reader acc_z-val
    :initarg :acc_z
    :type fixnum
    :initform 0)
   (acc_angle_nick
    :reader acc_angle_nick-val
    :initarg :acc_angle_nick
    :type integer
    :initform 0)
   (acc_angle_roll
    :reader acc_angle_roll-val
    :initarg :acc_angle_roll
    :type integer
    :initform 0)
   (acc_absolute_value
    :reader acc_absolute_value-val
    :initarg :acc_absolute_value
    :type integer
    :initform 0)
   (Hx
    :reader Hx-val
    :initarg :Hx
    :type integer
    :initform 0)
   (Hy
    :reader Hy-val
    :initarg :Hy
    :type integer
    :initform 0)
   (Hz
    :reader Hz-val
    :initarg :Hz
    :type integer
    :initform 0)
   (mag_heading
    :reader mag_heading-val
    :initarg :mag_heading
    :type integer
    :initform 0)
   (speed_x
    :reader speed_x-val
    :initarg :speed_x
    :type integer
    :initform 0)
   (speed_y
    :reader speed_y-val
    :initarg :speed_y
    :type integer
    :initform 0)
   (speed_z
    :reader speed_z-val
    :initarg :speed_z
    :type integer
    :initform 0)
   (height
    :reader height-val
    :initarg :height
    :type integer
    :initform 0)
   (dheight
    :reader dheight-val
    :initarg :dheight
    :type integer
    :initform 0)
   (dheight_reference
    :reader dheight_reference-val
    :initarg :dheight_reference
    :type integer
    :initform 0)
   (height_reference
    :reader height_reference-val
    :initarg :height_reference
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <IMUCalcData>) ostream)
  "Serializes a message object of type '<IMUCalcData>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'angle_nick)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'angle_nick)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'angle_nick)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'angle_nick)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'angle_roll)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'angle_roll)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'angle_roll)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'angle_roll)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'angle_yaw)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'angle_yaw)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'angle_yaw)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'angle_yaw)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'angvel_nick)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'angvel_nick)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'angvel_nick)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'angvel_nick)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'angvel_roll)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'angvel_roll)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'angvel_roll)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'angvel_roll)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'angvel_yaw)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'angvel_yaw)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'angvel_yaw)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'angvel_yaw)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'acc_x_calib)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'acc_x_calib)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'acc_y_calib)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'acc_y_calib)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'acc_z_calib)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'acc_z_calib)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'acc_x)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'acc_x)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'acc_y)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'acc_y)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'acc_z)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'acc_z)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'acc_angle_nick)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'acc_angle_nick)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'acc_angle_nick)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'acc_angle_nick)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'acc_angle_roll)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'acc_angle_roll)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'acc_angle_roll)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'acc_angle_roll)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'acc_absolute_value)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'acc_absolute_value)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'acc_absolute_value)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'acc_absolute_value)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'Hx)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'Hx)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'Hx)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'Hx)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'Hy)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'Hy)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'Hy)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'Hy)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'Hz)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'Hz)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'Hz)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'Hz)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'mag_heading)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'mag_heading)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'mag_heading)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'mag_heading)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'speed_x)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'speed_x)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'speed_x)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'speed_x)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'speed_y)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'speed_y)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'speed_y)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'speed_y)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'speed_z)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'speed_z)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'speed_z)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'speed_z)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'height)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'height)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'height)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'height)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'dheight)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'dheight)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'dheight)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'dheight)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'dheight_reference)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'dheight_reference)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'dheight_reference)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'dheight_reference)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'height_reference)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'height_reference)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'height_reference)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'height_reference)) ostream)
)
(defmethod deserialize ((msg <IMUCalcData>) istream)
  "Deserializes a message object of type '<IMUCalcData>"
  (setf (ldb (byte 8 0) (slot-value msg 'angle_nick)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'angle_nick)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'angle_nick)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'angle_nick)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'angle_roll)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'angle_roll)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'angle_roll)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'angle_roll)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'angle_yaw)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'angle_yaw)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'angle_yaw)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'angle_yaw)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'angvel_nick)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'angvel_nick)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'angvel_nick)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'angvel_nick)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'angvel_roll)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'angvel_roll)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'angvel_roll)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'angvel_roll)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'angvel_yaw)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'angvel_yaw)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'angvel_yaw)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'angvel_yaw)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'acc_x_calib)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'acc_x_calib)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'acc_y_calib)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'acc_y_calib)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'acc_z_calib)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'acc_z_calib)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'acc_x)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'acc_x)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'acc_y)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'acc_y)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'acc_z)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'acc_z)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'acc_angle_nick)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'acc_angle_nick)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'acc_angle_nick)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'acc_angle_nick)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'acc_angle_roll)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'acc_angle_roll)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'acc_angle_roll)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'acc_angle_roll)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'acc_absolute_value)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'acc_absolute_value)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'acc_absolute_value)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'acc_absolute_value)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'Hx)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'Hx)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'Hx)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'Hx)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'Hy)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'Hy)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'Hy)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'Hy)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'Hz)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'Hz)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'Hz)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'Hz)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'mag_heading)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'mag_heading)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'mag_heading)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'mag_heading)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'speed_x)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'speed_x)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'speed_x)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'speed_x)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'speed_y)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'speed_y)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'speed_y)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'speed_y)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'speed_z)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'speed_z)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'speed_z)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'speed_z)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'height)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'height)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'height)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'height)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'dheight)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'dheight)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'dheight)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'dheight)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'dheight_reference)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'dheight_reference)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'dheight_reference)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'dheight_reference)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'height_reference)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'height_reference)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'height_reference)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'height_reference)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<IMUCalcData>)))
  "Returns string type for a message object of type '<IMUCalcData>"
  "asctec_msgs/IMUCalcData")
(defmethod md5sum ((type (eql '<IMUCalcData>)))
  "Returns md5sum for a message object of type '<IMUCalcData>"
  "d34111f01cd460b1a9967f6d9469d0f6")
(defmethod message-definition ((type (eql '<IMUCalcData>)))
  "Returns full string definition for message of type '<IMUCalcData>"
  (format nil "# angles derived by integration of gyro_outputs, drift compensated by data fusion;~%#-90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree~%~%int32 angle_nick~%int32 angle_roll~%int32 angle_yaw~%~%# angular velocities, raw values [16 bit], bias free, in 0.0154 deg/s (=> 64.8 = 1 deg/s)~%~%int32 angvel_nick~%int32 angvel_roll~%int32 angvel_yaw~%~%# acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g~%~%int16 acc_x_calib~%int16 acc_y_calib~%int16 acc_z_calib~%~%# horizontal / vertical accelerations: -10000..+10000 = -1g..+1g~%~%int16 acc_x~%int16 acc_y~%int16 acc_z~%~%# reference angles derived by accelerations only: -90000..+90000; 1000 = 1 degree~%~%int32 acc_angle_nick~%int32 acc_angle_roll~%~%# total acceleration measured (10000 = 1g)~%~%int32 acc_absolute_value~%~%# magnetic field sensors output, offset free and scaled; units not determined, ~%# as only the direction of the field vector is taken into account~%~%int32 Hx~%int32 Hy~%int32 Hz~%~%# compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree~%~%int32 mag_heading~%~%# pseudo speed measurements: integrated accelerations, pulled towards zero; units unknown;~%# used for short-term position stabilization~%~%int32 speed_x~%int32 speed_y~%int32 speed_z~%~%# height in mm (after data fusion)~%~%int32 height~%~%# diff. height in mm/s (after data fusion)~%~%int32 dheight~%~%# diff. height measured by the pressure sensor [mm/s]~%~%int32 dheight_reference~%~%# height measured by the pressure sensor [mm]~%~%int32 height_reference~%~%~%~%"))
(defmethod serialization-length ((msg <IMUCalcData>))
  (+ 0
     4
     4
     4
     4
     4
     4
     2
     2
     2
     2
     2
     2
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <IMUCalcData>))
  "Converts a ROS message object to a list"
  (list '<IMUCalcData>
    (cons ':angle_nick (angle_nick-val msg))
    (cons ':angle_roll (angle_roll-val msg))
    (cons ':angle_yaw (angle_yaw-val msg))
    (cons ':angvel_nick (angvel_nick-val msg))
    (cons ':angvel_roll (angvel_roll-val msg))
    (cons ':angvel_yaw (angvel_yaw-val msg))
    (cons ':acc_x_calib (acc_x_calib-val msg))
    (cons ':acc_y_calib (acc_y_calib-val msg))
    (cons ':acc_z_calib (acc_z_calib-val msg))
    (cons ':acc_x (acc_x-val msg))
    (cons ':acc_y (acc_y-val msg))
    (cons ':acc_z (acc_z-val msg))
    (cons ':acc_angle_nick (acc_angle_nick-val msg))
    (cons ':acc_angle_roll (acc_angle_roll-val msg))
    (cons ':acc_absolute_value (acc_absolute_value-val msg))
    (cons ':Hx (Hx-val msg))
    (cons ':Hy (Hy-val msg))
    (cons ':Hz (Hz-val msg))
    (cons ':mag_heading (mag_heading-val msg))
    (cons ':speed_x (speed_x-val msg))
    (cons ':speed_y (speed_y-val msg))
    (cons ':speed_z (speed_z-val msg))
    (cons ':height (height-val msg))
    (cons ':dheight (dheight-val msg))
    (cons ':dheight_reference (dheight_reference-val msg))
    (cons ':height_reference (height_reference-val msg))
))
