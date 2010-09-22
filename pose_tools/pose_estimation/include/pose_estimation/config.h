#ifndef CONFIG_H
#define CONFIG_H

// **** Sizes

const int STATE_SIZE = 6;			//state: [x, y, z, roll, pitch, yaw]
const int INPUT_SIZE = 6;			//input: [dx, dy, dz, droll, dpitch, dyaw]
const int MEAS_SIZE  = 6; 	  //measurment: [x, y, z, roll, pitch, yaw]

// **** System noise median

const double MU_SYSTEM_NOISE_X     = 0.0;
const double MU_SYSTEM_NOISE_Y     = 0.0;
const double MU_SYSTEM_NOISE_Z     = 0.0;
const double MU_SYSTEM_NOISE_ROLL  = 0.0;
const double MU_SYSTEM_NOISE_PITCH = 0.0;
const double MU_SYSTEM_NOISE_YAW	 = 0.0;

// **** System noise covariance

const double SIGMA_SYSTEM_NOISE_X     = 0.00001;
const double SIGMA_SYSTEM_NOISE_Y     = 0.00001;
const double SIGMA_SYSTEM_NOISE_Z     = 0.00001;
const double SIGMA_SYSTEM_NOISE_ROLL  = 0.00001;
const double SIGMA_SYSTEM_NOISE_PITCH = 0.00001;
const double SIGMA_SYSTEM_NOISE_YAW	  = 0.00001;

// **** Measurement noise median

const double MU_MEAS_NOISE_X     = 0.0;
const double MU_MEAS_NOISE_Y     = 0.0;
const double MU_MEAS_NOISE_Z     = 0.0;
const double MU_MEAS_NOISE_ROLL  = 0.0;
const double MU_MEAS_NOISE_PITCH = 0.0;
const double MU_MEAS_NOISE_YAW	 = 0.0;

// **** Measurement noise covariance

const double SIGMA_MEAS_NOISE_X     = 1.0;
const double SIGMA_MEAS_NOISE_Y     = 1.0;
const double SIGMA_MEAS_NOISE_Z     = 1.0;
const double SIGMA_MEAS_NOISE_ROLL  = 1.0;
const double SIGMA_MEAS_NOISE_PITCH = 1.0;
const double SIGMA_MEAS_NOISE_YAW	  = 1.0;

// **** Prior median

const double MU_PRIOR_X     = 0.0;
const double MU_PRIOR_Y     = 0.0;
const double MU_PRIOR_Z     = 0.0;
const double MU_PRIOR_ROLL  = 0.0;
const double MU_PRIOR_PITCH = 0.0;
const double MU_PRIOR_YAW   = 0.0;

// **** Prior covariance

const double SIGMA_PRIOR_X     = 1.0;
const double SIGMA_PRIOR_Y     = 1.0;
const double SIGMA_PRIOR_Z     = 1.0;
const double SIGMA_PRIOR_ROLL  = 1.0;
const double SIGMA_PRIOR_PITCH = 1.0;
const double SIGMA_PRIOR_YAW   = 1.0;

#endif

