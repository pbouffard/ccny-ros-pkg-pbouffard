#include "pose_estimation/nonlinear_analytic_conditional_gaussian_6D.h"

namespace BFL
{
  using namespace MatrixWrapper;

  NonLinearAnalyticConditionalGaussian6D::NonLinearAnalyticConditionalGaussian6D(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise, NUMCONDARGUMENTS)
  {

  }

  NonLinearAnalyticConditionalGaussian6D::~NonLinearAnalyticConditionalGaussian6D()
	{

	}

  ColumnVector NonLinearAnalyticConditionalGaussian6D::ExpectedValueGet() const
  {
		// System model - update the pose

/*
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector vel  = ConditionalArgumentGet(1);
    state(1) += cos(state(3)) * vel(1);
    state(2) += sin(state(3)) * vel(1);
    state(3) += vel(2);
*/
    ColumnVector state  = ConditionalArgumentGet(0);	// [x, y, z, r, p, y]
    ColumnVector dstate = ConditionalArgumentGet(1);  // [dx, dy, dz, dr, dp, dy]
	
		state(1) += dstate(1);
		state(2) += dstate(2);
		state(3) += dstate(3);
		state(4) += dstate(4);
		state(5) += dstate(5);
		state(6) += dstate(6);	

    return state + AdditiveNoiseMuGet();
  }

  Matrix NonLinearAnalyticConditionalGaussian6D::dfGet(unsigned int i) const
  {
		// system model - update the derivative

    if (i==0)//derivative to the first conditional argument (x)
    {
			Matrix df(6,6);

			// The Jacobian is the identity matrix

			for (int i = 1; i <= 6; i++)
			for (int j = 1; j <= 6; j++)
				if (i == j) df(i, j) = 1;
				else df(i, j) = 0;

			return df;
    }
    else
    {
			if (i >= NumConditionalArgumentsGet())
			{
			  cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
			  exit(-BFL_ERRMISUSE);
			}
			else
			{
				cerr << "The df is not implemented for the" <<i << "th conditional argument\n";
				exit(-BFL_ERRMISUSE);
			}
    }
  }

}//namespace BFL

