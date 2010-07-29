Header header
# angles derived by integration of gyro_outputs, drift compensated by data fusion;
#-90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree

int32 angle_nick
int32 angle_roll
int32 angle_yaw

# angular velocities, raw values [16 bit], bias free, in 0.0154 deg/s (=> 64.8 = 1 deg/s)

int32 angvel_nick
int32 angvel_roll
int32 angvel_yaw

# acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g

int16 acc_x_calib
int16 acc_y_calib
int16 acc_z_calib

# horizontal / vertical accelerations: -10000..+10000 = -1g..+1g

int16 acc_x
int16 acc_y
int16 acc_z

# reference angles derived by accelerations only: -90000..+90000; 1000 = 1 degree

int32 acc_angle_nick
int32 acc_angle_roll

# total acceleration measured (10000 = 1g)

int32 acc_absolute_value

# magnetic field sensors output, offset free and scaled; units not determined, 
# as only the direction of the field vector is taken into account

int32 Hx
int32 Hy
int32 Hz

# compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree

int32 mag_heading

# pseudo speed measurements: integrated accelerations, pulled towards zero; units unknown;
# used for short-term position stabilization

int32 speed_x
int32 speed_y
int32 speed_z

# height in mm (after data fusion)

int32 height

# diff. height in mm/s (after data fusion)

int32 dheight

# diff. height measured by the pressure sensor [mm/s]

int32 dheight_reference

# height measured by the pressure sensor [mm]

int32 height_reference

