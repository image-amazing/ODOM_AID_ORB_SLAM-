#ifndef ODOMDATA_H
#define ODOMDATA_H

#include <Eigen/Dense>

namespace ORB_SLAM2
{

using namespace Eigen;

class OdomData
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // covariance of measurement
    static Matrix3d _omeMeasCov;
    static Matrix3d _velMeasCov;
    static Matrix3d getOmeMeasCov(void) {return _omeMeasCov;}
    static Matrix3d getVelMeasCov(void) {return _velMeasCov;}

    OdomData(const double& vx, const double& vy, const double& vz,
	     const double& wx, const double& wy, const double& wz,
             const double& t);

    // Raw data of odom's
    Vector3d _v;
    Vector3d _w;
    double _t;      //timestamp
};

}

#endif // ODOMDATA_H
