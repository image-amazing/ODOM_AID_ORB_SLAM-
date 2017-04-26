#include "Odom/odomdata.h"

namespace ORB_SLAM2
{

Matrix3d OdomData::_velMeasCov = Matrix3d::Identity()*4e-06/0.0105;

Matrix3d OdomData::_omeMeasCov = Matrix3d::Identity()*2.89e-04/0.0105;

OdomData::OdomData(const double& vx, const double& vy, const double& vz,
		   const double& wx, const double& wy, const double& wz,
		   const double& t) :
		   _v(vx, vy, vz), _w(wx, wy, wz), _t(t)
{
  _velMeasCov(2,2) = 1000/0.0105;
  _omeMeasCov(0,0) = 1000/0.0105;
  _omeMeasCov(1,1) = 1000/0.0105;
}

}