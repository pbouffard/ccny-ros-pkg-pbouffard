#ifndef POSE_ESTIMATION_NONLINEAR_ANALYTIC_CONDITIONAL_GAUSSIAN_6D_H
#define POSE_ESTIMATION_NONLINEAR_ANALYTIC_CONDITIONAL_GAUSSIAN_6D_H

#include <pdf/analyticconditionalgaussian_additivenoise.h>
#include <wrappers/rng/rng.h> // Wrapper around several rng libraries

#define NUMCONDARGUMENTS 2

namespace BFL
{
  /// Non Linear Conditional Gaussian
  /**
     - \f$ \mu = Matrix[1] . ConditionalArguments[0] +
     Matrix[2]. ConditionalArguments[1]  + ... + Noise.\mu \f$
     - Covariance is independent of the ConditionalArguments, and is
     the covariance of the Noise pdf
  */

  class NonLinearAnalyticConditionalGaussian6D : 
	public AnalyticConditionalGaussianAdditiveNoise
  {
    public:
      /// Constructor
      /** @pre:  Every Matrix should have the same amount of rows!
	  This is currently not checked.  The same goes for the number
	  of columns, which should be equal to the number of rows of
	  the corresponding conditional argument!
	  @param ratio: vector containing the different matrices of
	  the linear relationship between the conditional arguments
	  and \f$\mu\f$
	  @param additiveNoise Pdf representing the additive Gaussian uncertainty
      */
      NonLinearAnalyticConditionalGaussian6D(const Gaussian& additiveNoise);

      /// Destructor
      virtual ~NonLinearAnalyticConditionalGaussian6D();

      // redefine virtual functions
      virtual MatrixWrapper::ColumnVector    ExpectedValueGet() const;
      virtual MatrixWrapper::Matrix          dfGet(unsigned int i) const;
    };

} // End namespace BFL

#endif

