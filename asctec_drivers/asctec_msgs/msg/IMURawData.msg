Header header
#pressure sensor 24-bit value, not scaled but bias free
int32 pressure

#16-bit gyro readings; 32768 = 2.5V
int16 gyro_x
int16 gyro_y
int16 gyro_z

#10-bit magnetic field sensor readings
int16 mag_x
int16 mag_y
int16 mag_z

#16-bit accelerometer readings
int16 acc_x
int16 acc_y
int16 acc_z

#16-bit temperature measurement using yaw-gyro internal sensor
int16 temp_gyro

#16-bit temperature measurement using ADC internal sensor
int32 temp_ADC
