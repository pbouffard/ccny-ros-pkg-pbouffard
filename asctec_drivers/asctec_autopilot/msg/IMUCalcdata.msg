# angles derived by integration of gyro_outputs, drift compensated by data fusion; -90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree

int64 angle_roll
int64 angle_pitch
int64 angle_yaw

# acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g

int64 acc_x_calib
int64 acc_y_calib
int64 acc_z_calib

# horizontal / vertical accelerations: -10000..+10000 = -1g..+1g

int64 acc_x
int64 acc_y
int64 acc_z

# height in mm (after data fusion)

int64 height

# height from sensor in mm

int64 height_reference

