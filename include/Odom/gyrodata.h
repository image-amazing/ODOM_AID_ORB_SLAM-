#ifndef GYRODATA_H
#define GYRODATA_H

#include <Eigen/Dense>

namespace ORB_SLAM2
{

using namespace Eigen;

class GyroData
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // covariance of measurement
    static Matrix3d _gyrMeasCov;

    static Matrix3d getGyrMeasCov(void) {return _gyrMeasCov;}

    // covariance of bias random walk
    static Matrix3d _gyrBiasRWCov;
    static Matrix3d getGyrBiasRWCov(void) {return _gyrBiasRWCov;}

    static double _gyrBiasRw2;
    static double getGyrBiasRW2(void) {return _gyrBiasRw2;}


    GyroData(const double& gx, const double& gy, const double& gz,
             const double& t);

    // Raw data of gyro's
    Vector3d _g;    //gyr data
    double _t;      //timestamp
};

}

#endif