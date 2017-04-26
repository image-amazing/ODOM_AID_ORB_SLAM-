#ifndef NAVSTATE_H
#define NAVSTATE_H

#include "Eigen/Geometry"
#include "Odom/SophusType.h"

namespace ORB_SLAM2
{
//navigation state w.r.t. odom frame
  
using namespace Eigen;
//using namespace g2o;
typedef Matrix<double, 15, 1> Vector15d;
typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 6, 1> Vector6d;

typedef Matrix<double, 12, 1> Vector12d;

class NavState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    NavState();
    NavState(const NavState& _ns);

    //Quaterniond Get_qR(){return _qR;}     // rotation
    SO3Type Get_R() const{return _R;}
    //Matrix3d Get_RotMatrix(){return _qR.toRotationMatrix();}   // rotation matrix
    Matrix3d Get_RotMatrix() const{return _R.matrix();}
    Vector3d Get_P() const{return _P;}         // position
//     Vector3d Get_V() const{return _V;}         // velocity
    void Set_Pos(const Vector3d &pos){_P = pos;}
//     void Set_Vel(const Vector3d &vel){_V = vel;}
    void Set_Rot(const Matrix3d &rot){_R = SO3Type(rot);}
    void Set_Rot(const SO3Type &rot){_R = rot;}

    // incremental addition, delta = [dP, dPhi]
    void IncSmall(Vector6d delta);

    // normalize rotation quaternion. !!! important!!!
    //void normalizeRotation(void){_qR = normalizeRotationQ(_qR);}
    //void normalizeRotation(void){_R.normalize();}

    //    inline Quaterniond normalizeRotationQ(const Quaterniond& r)
    //    {
    //        Quaterniond _r(r);
    //        if (_r.w()<0)
    //        {
    //            _r.coeffs() *= -1;
    //        }
    //        return _r.normalized();
    //    }

private:
    /*
     * Note:
     * don't add pointer as member variable.
     * operator = is used in g2o
    */

    Vector3d _P;         // position
    //Quaterniond _qR;     // rotation
    SO3Type _R;
};

}

#endif // NAVSTATE_H
