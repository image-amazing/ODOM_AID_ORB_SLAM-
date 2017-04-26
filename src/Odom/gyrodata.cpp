#include "Odom/gyrodata.h"


namespace ORB_SLAM2
{
double GyroData::_gyrBiasRw2 = 2.0e-5*2.0e-5/**10*/;  //2e-12*1e3

Matrix3d GyroData::_gyrMeasCov = Matrix3d::Identity()*1.7e-4*1.7e-4/0.005/**100*/;       // sigma_g * sigma_g / dt, ~6e-6*10

// covariance of bias random walk
Matrix3d GyroData::_gyrBiasRWCov = Matrix3d::Identity()*_gyrBiasRw2;     // sigma_gw * sigma_gw * dt, ~2e-12

GyroData::GyroData(const double& gx, const double& gy, const double& gz,
		   const double& t) :
    _g(gx,gy,gz), _t(t)
{
}


}
