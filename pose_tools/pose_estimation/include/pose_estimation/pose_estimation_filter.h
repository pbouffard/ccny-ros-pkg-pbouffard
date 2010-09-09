#ifndef POSE_ESTIMATION_POSE_ESTIMATION_FILTER_H
#define POSE_ESTIMATION_POSE_ESTIMATION_FILTER_H

#include <stdio.h>

// **** BFL includes

#include <filter/extendedkalmanfilter.h>

#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>

// **** Other includes

#include "pose_estimation/config.h"
#include "pose_estimation/nonlinear_analytic_conditional_gaussian_6D.h"

//using namespace MatrixWrapper::;
//using namespace BFL;
//using namespace std;

class PoseEstimationFilter
{
	private:

		// **** system model

		BFL::ColumnVector 	staticSysNoiseMu;
		BFL::SymmetricMatrix staticSysNoiseCov;

 		BFL::Gaussian 															* staticSysUncertainty;
 		BFL::NonLinearAnalyticConditionalGaussian6D * staticSysPdf;
		BFL::AnalyticSystemModelGaussianUncertainty * staticSysModel;

		// **** measurement model

		BFL::Matrix H;

		BFL::ColumnVector    staticMeasNoiseMu;
  	BFL::SymmetricMatrix staticMeasNoiseCov;

  	BFL::Gaussian                                          *	staticMeasUncertainty;
		BFL::LinearAnalyticConditionalGaussian                 * staticMeasPdf;
		BFL::LinearAnalyticMeasurementModelGaussianUncertainty * staticMeasModel;

		// **** prior density

		BFL::ColumnVector 	  priorMu;
  	BFL::SymmetricMatrix priorCov;

		BFL::Gaussian * priorCont;

		// **** filter

		BFL::ExtendedKalmanFilter * ekfFilter;

		void initStaticSysModel();
		void initH();
		void initStaticMeasModel();
		void initPriorModel();

	public:

		/// constructor
		PoseEstimationFilter();

		/// destructor
		virtual ~PoseEstimationFilter();

		void updateFromMeasurement(const BFL::ColumnVector& measurement);
    void updateFromMeasurement(const BFL::ColumnVector& measurement,
											         const BFL::ColumnVector& measNoiseMu,
                               const BFL::SymmetricMatrix& measNoiseCov);

		void updateFromMotion(const BFL::ColumnVector& input);
    void updateFromMotion(const BFL::ColumnVector& input,
											    const BFL::ColumnVector& inputNoiseMu,
                          const BFL::SymmetricMatrix& inputNoiseCov);

		void getPosteriorMean(BFL::ColumnVector& mean);
		void getPosteriorCovariance(BFL::SymmetricMatrix& covariance);
};

#endif //POSE_ESTIMATION_POSE_ESTIMATION_FILTER_H
