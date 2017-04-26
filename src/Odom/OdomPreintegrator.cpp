#include "Odom/OdomPreintegrator.h"

namespace ORB_SLAM2
{

OdomPreintegrator::OdomPreintegrator(const OdomPreintegrator& pre):
    _delta_P(pre._delta_P), 
    _delta_R(pre._delta_R),
    
    _cov_P_Phi(pre._cov_P_Phi),
    _delta_time(pre._delta_time)
{

}

OdomPreintegrator::OdomPreintegrator()
{
    // delta measurements, position/velocity/rotation(matrix)
    _delta_P.setZero();    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    _delta_R.setIdentity();    // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

    // noise covariance propagation of delta measurements
    _cov_P_Phi.setZero();

    _delta_time = 0;
}

void OdomPreintegrator::reset()
{
    // delta measurements, position/velocity/rotation(matrix)
    _delta_P.setZero();    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    _delta_R.setIdentity();    // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

    // noise covariance propagation of delta measurements
    _cov_P_Phi.setZero();

    _delta_time = 0;

}

// incrementally update 1)delta measurements, 2)jacobians, 3)covariance matrix
void OdomPreintegrator::update(const Vector3d& v, const Vector3d& w, const double& dt)
{
    Matrix3d dR = Expmap(w*dt);

    // noise covariance propagation of delta measurements
    // err_k+1 = A*err_k + B*err_gyro + C*err_acc
    Matrix3d I3x3 = Matrix3d::Identity();
    Matrix6d A = Matrix6d::Identity();
    A.block<3,3>(3,3) = dR.transpose();
    A.block<3,3>(0,3) = -_delta_R*skew(v)*dt;
    A.block<3,3>(0,0) = I3x3;
    Matrix<double,6,3> B = Matrix<double,6,3>::Zero();
    B.block<3,3>(3,0) = I3x3*dt; 
    Matrix<double,6,3> C = Matrix<double,6,3>::Zero();
    C.block<3,3>(0,0) = _delta_R*dt;
    _cov_P_Phi = A*_cov_P_Phi*A.transpose() +
		 B*OdomData::getOmeMeasCov()*B.transpose() +
		 C*OdomData::getVelMeasCov()*C.transpose();
		 
    // delta measurements, position/velocity/rotation(matrix)
    // update P first, then V, then R. because P's update need V&R's previous state
    _delta_P += _delta_R*v*dt;    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    _delta_R = normalizeRotationM(_delta_R*dR);  // normalize rotation, in case of numerical error accumulation

    // delta time
    _delta_time += dt;

}

}
