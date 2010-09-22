#include "pose_estimation/pose_estimation_filter.h"

PoseEstimationFilter::PoseEstimationFilter():
	  staticSysNoiseMu(STATE_SIZE),
		staticSysNoiseCov(STATE_SIZE),
		H(MEAS_SIZE,STATE_SIZE),
		staticMeasNoiseMu(MEAS_SIZE),
		staticMeasNoiseCov(MEAS_SIZE),
  	priorMu(STATE_SIZE),
  	priorCov(STATE_SIZE)
{

  // **** NonLinear system model

	// initialize mean and covariance
	initStaticSysModel();

  // create the static motion model
	staticSysUncertainty = new BFL::Gaussian(staticSysNoiseMu, staticSysNoiseCov);
 	staticSysPdf   = new BFL::NonLinearAnalyticConditionalGaussian6D(*staticSysUncertainty);
  staticSysModel = new BFL::AnalyticSystemModelGaussianUncertainty(staticSysPdf);

  // ****  Initialise measurement model

  // create matrix H for linear measurement model
	initH();

  // initialize the measurement noise 
	initStaticMeasModel();

  // create the static measurement model
	staticMeasUncertainty = new BFL::Gaussian(staticMeasNoiseMu, staticMeasNoiseCov);
  staticMeasPdf = new BFL::LinearAnalyticConditionalGaussian(H, *staticMeasUncertainty);
  staticMeasModel = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(staticMeasPdf);

  // **** Prior density 

  // Continuous Gaussian prior (for Kalman filters)
	initPriorModel();

  priorCont = new BFL::Gaussian(priorMu, priorCov);

  // ****  Construction of the filter

  ekfFilter = new BFL::ExtendedKalmanFilter(priorCont);
};

PoseEstimationFilter::~PoseEstimationFilter()
{

};

void PoseEstimationFilter::updateFromMeasurement(const BFL::ColumnVector& measurement,
											                           const BFL::ColumnVector& measNoiseMu,
                                                 const BFL::SymmetricMatrix& measNoiseCov)
{
  // create the measurement model
	BFL::Gaussian                                          *	measUncertainty;
	BFL::LinearAnalyticConditionalGaussian                 * measPdf;
	BFL::LinearAnalyticMeasurementModelGaussianUncertainty * measModel;

	measUncertainty = new BFL::Gaussian(measNoiseMu, measNoiseCov);
  measPdf = new BFL::LinearAnalyticConditionalGaussian(H, *measUncertainty);
  measModel = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(measPdf);

  // update the filter from measurement
	ekfFilter->Update(measModel, measurement);
}

void PoseEstimationFilter::updateFromMeasurement(const BFL::ColumnVector& measurement)
{
  // update filter with static measurement model
	ekfFilter->Update(staticMeasModel, measurement);
}

void PoseEstimationFilter::updateFromMotion(const BFL::ColumnVector& input,
											                      const BFL::ColumnVector& inputNoiseMu,
                                            const BFL::SymmetricMatrix& inputNoiseCov)
{
  // create the motion model
	BFL::Gaussian 														  * sysUncertainty;
	BFL::NonLinearAnalyticConditionalGaussian6D * sysPdf;
	BFL::AnalyticSystemModelGaussianUncertainty * sysModel;

	sysUncertainty = new BFL::Gaussian(inputNoiseMu, inputNoiseCov);
  sysPdf   = new BFL::NonLinearAnalyticConditionalGaussian6D(*sysUncertainty);
  sysModel = new BFL::AnalyticSystemModelGaussianUncertainty(sysPdf);

  // update the filter from motion
	ekfFilter->Update(sysModel, input);
}

void PoseEstimationFilter::updateFromMotion(const BFL::ColumnVector& input)
{
  // update filter with static system model
	ekfFilter->Update(staticSysModel, input);
}

void PoseEstimationFilter::getPosteriorMean(BFL::ColumnVector& mean)
{
	BFL::Pdf<BFL::ColumnVector> * posterior = ekfFilter->PostGet();
	mean = posterior->ExpectedValueGet();
}

void PoseEstimationFilter::getPosteriorCovariance(BFL::SymmetricMatrix& covariance)
{
	BFL::Pdf<BFL::ColumnVector> * posterior = ekfFilter->PostGet();
	covariance = posterior->CovarianceGet();
}

void PoseEstimationFilter::initStaticSysModel()
{
 	staticSysNoiseMu(1) = MU_SYSTEM_NOISE_X;
  staticSysNoiseMu(2) = MU_SYSTEM_NOISE_Y;
	staticSysNoiseMu(3) = MU_SYSTEM_NOISE_Z;
  staticSysNoiseMu(4) = MU_SYSTEM_NOISE_ROLL;
  staticSysNoiseMu(5) = MU_SYSTEM_NOISE_PITCH;
  staticSysNoiseMu(6) = MU_SYSTEM_NOISE_YAW;

  staticSysNoiseCov = 0.0;
  staticSysNoiseCov(1,1) = SIGMA_SYSTEM_NOISE_X;
  staticSysNoiseCov(1,2) = 0.0;
  staticSysNoiseCov(1,3) = 0.0;
  staticSysNoiseCov(1,4) = 0.0;
  staticSysNoiseCov(1,5) = 0.0;
  staticSysNoiseCov(1,6) = 0.0;

  staticSysNoiseCov(2,1) = 0.0;
  staticSysNoiseCov(2,2) = SIGMA_SYSTEM_NOISE_Y;
  staticSysNoiseCov(2,3) = 0.0;
  staticSysNoiseCov(2,4) = 0.0;
  staticSysNoiseCov(2,5) = 0.0;
  staticSysNoiseCov(2,6) = 0.0;

  staticSysNoiseCov(3,1) = 0.0;
  staticSysNoiseCov(3,2) = 0.0;
  staticSysNoiseCov(3,3) = SIGMA_SYSTEM_NOISE_Z;
  staticSysNoiseCov(3,4) = 0.0;
  staticSysNoiseCov(3,5) = 0.0;
  staticSysNoiseCov(3,6) = 0.0;

  staticSysNoiseCov(4,1) = 0.0;
  staticSysNoiseCov(4,2) = 0.0;
  staticSysNoiseCov(4,3) = 0.0;
  staticSysNoiseCov(4,4) = SIGMA_SYSTEM_NOISE_ROLL;
  staticSysNoiseCov(4,5) = 0.0;
  staticSysNoiseCov(4,6) = 0.0;

  staticSysNoiseCov(5,1) = 0.0;
  staticSysNoiseCov(5,2) = 0.0;
  staticSysNoiseCov(5,3) = 0.0;
  staticSysNoiseCov(5,4) = 0.0;
  staticSysNoiseCov(5,5) = SIGMA_SYSTEM_NOISE_PITCH;
  staticSysNoiseCov(5,6) = 0.0;

	staticSysNoiseCov(5,1) = 0.0;
  staticSysNoiseCov(5,2) = 0.0;
  staticSysNoiseCov(5,3) = 0.0;
  staticSysNoiseCov(5,4) = 0.0;
  staticSysNoiseCov(5,5) = 0.0;
  staticSysNoiseCov(5,6) = SIGMA_SYSTEM_NOISE_YAW;
}

void PoseEstimationFilter::initH()
{
	H = 0.0;

  H(1,1) = 1.0;
  H(1,2) = 0.0;
  H(1,3) = 0.0;
  H(1,4) = 0.0;
  H(1,5) = 0.0;
  H(1,6) = 0.0;

  H(2,1) = 0.0;
  H(2,2) = 1.0;
  H(2,3) = 0.0;
  H(2,4) = 0.0;
  H(2,5) = 0.0;
  H(2,6) = 0.0;

  H(3,1) = 0.0;
  H(3,2) = 0.0;
  H(3,3) = 1.0;
  H(3,4) = 0.0;
  H(3,5) = 0.0;
  H(3,6) = 0.0;

  H(4,1) = 0.0;
  H(4,2) = 0.0;
  H(4,3) = 0.0;
  H(4,4) = 1.0;
  H(4,5) = 0.0;
  H(4,6) = 0.0;

  H(5,1) = 0.0;
  H(5,2) = 0.0;
  H(5,3) = 0.0;
  H(5,4) = 0.0;
  H(5,5) = 1.0;
  H(5,6) = 0.0;

  H(6,1) = 0.0;
  H(6,2) = 0.0;
  H(6,3) = 0.0;
  H(6,4) = 0.0;
  H(6,5) = 0.0;
  H(6,6) = 1.0;
}

void PoseEstimationFilter::initStaticMeasModel()
{
  staticMeasNoiseMu(1) = MU_MEAS_NOISE_X;
	staticMeasNoiseMu(2) = MU_MEAS_NOISE_Y;
	staticMeasNoiseMu(3) = MU_MEAS_NOISE_Z;
	staticMeasNoiseMu(4) = MU_MEAS_NOISE_ROLL;
	staticMeasNoiseMu(5) = MU_MEAS_NOISE_PITCH;
	staticMeasNoiseMu(6) = MU_MEAS_NOISE_YAW;

  staticMeasNoiseCov(1,1) = SIGMA_MEAS_NOISE_X;
  staticMeasNoiseCov(1,2) = 0.0;
  staticMeasNoiseCov(1,3) = 0.0;
  staticMeasNoiseCov(1,4) = 0.0;
  staticMeasNoiseCov(1,5) = 0.0;
  staticMeasNoiseCov(1,6) = 0.0;

  staticMeasNoiseCov(2,1) = 0.0;
  staticMeasNoiseCov(2,2) = SIGMA_MEAS_NOISE_Y;
  staticMeasNoiseCov(2,3) = 0.0;
  staticMeasNoiseCov(2,4) = 0.0;
  staticMeasNoiseCov(2,5) = 0.0;
  staticMeasNoiseCov(2,6) = 0.0;

  staticMeasNoiseCov(3,1) = 0.0;
  staticMeasNoiseCov(3,2) = 0.0;
  staticMeasNoiseCov(3,3) = SIGMA_MEAS_NOISE_Z;
  staticMeasNoiseCov(3,4) = 0.0;
  staticMeasNoiseCov(3,5) = 0.0;
  staticMeasNoiseCov(3,6) = 0.0;

  staticMeasNoiseCov(4,1) = 0.0;
  staticMeasNoiseCov(4,2) = 0.0;
  staticMeasNoiseCov(4,3) = 0.0;
  staticMeasNoiseCov(4,4) = SIGMA_MEAS_NOISE_ROLL;
  staticMeasNoiseCov(4,5) = 0.0;
  staticMeasNoiseCov(4,6) = 0.0;

  staticMeasNoiseCov(5,1) = 0.0;
  staticMeasNoiseCov(5,2) = 0.0;
  staticMeasNoiseCov(5,3) = 0.0;
  staticMeasNoiseCov(5,4) = 0.0;
  staticMeasNoiseCov(5,5) = SIGMA_MEAS_NOISE_PITCH;
  staticMeasNoiseCov(5,6) = 0.0;

  staticMeasNoiseCov(6,1) = 0.0;
  staticMeasNoiseCov(6,2) = 0.0;
  staticMeasNoiseCov(6,3) = 0.0;
  staticMeasNoiseCov(6,4) = 0.0;
  staticMeasNoiseCov(6,5) = 0.0;
  staticMeasNoiseCov(6,6) = SIGMA_MEAS_NOISE_YAW;
}

void PoseEstimationFilter::initPriorModel()
{
  priorMu(1) = MU_PRIOR_X;
  priorMu(2) = MU_PRIOR_Y;
  priorMu(3) = MU_PRIOR_Z;
  priorMu(4) = MU_PRIOR_ROLL;
  priorMu(5) = MU_PRIOR_PITCH;
  priorMu(6) = MU_PRIOR_YAW;

  priorCov(1,1) = SIGMA_PRIOR_X;
  priorCov(1,2) = 0.0;
  priorCov(1,3) = 0.0;
  priorCov(1,4) = 0.0;
  priorCov(1,5) = 0.0;
  priorCov(1,6) = 0.0;

  priorCov(2,1) = 0.0;
  priorCov(2,2) = SIGMA_PRIOR_Y;
  priorCov(2,3) = 0.0;
  priorCov(2,4) = 0.0;
  priorCov(2,5) = 0.0;
  priorCov(2,6) = 0.0;

  priorCov(3,1) = 0.0;
  priorCov(3,2) = 0.0;
  priorCov(3,3) = SIGMA_PRIOR_Z;
  priorCov(3,4) = 0.0;
  priorCov(3,5) = 0.0;
  priorCov(3,6) = 0.0;

  priorCov(4,1) = 0.0;
  priorCov(4,2) = 0.0;
  priorCov(4,3) = 0.0;
  priorCov(4,4) = SIGMA_PRIOR_ROLL;
  priorCov(4,5) = 0.0;
  priorCov(4,6) = 0.0;

  priorCov(5,1) = 0.0;
  priorCov(5,2) = 0.0;
  priorCov(5,3) = 0.0;
  priorCov(5,4) = 0.0;
  priorCov(5,5) = SIGMA_PRIOR_PITCH;
  priorCov(5,6) = 0.0;

  priorCov(6,1) = 0.0;
  priorCov(6,2) = 0.0;
  priorCov(6,3) = 0.0;
  priorCov(6,4) = 0.0;
  priorCov(6,5) = 0.0;
  priorCov(6,6) = SIGMA_PRIOR_YAW;
}
