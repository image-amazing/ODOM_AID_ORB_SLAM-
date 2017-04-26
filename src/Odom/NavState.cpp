#include "Odom/NavState.h"

namespace ORB_SLAM2
{

NavState::NavState()
{
    //_qR.setIdentity();     // rotation
    _P.setZero();         // position
}

// if there's some other constructor, normalizeRotation() is needed
NavState::NavState(const NavState &_ns):
    _P(_ns._P), _R(_ns._R)
{
    //normalizeRotation();
}

void NavState::IncSmall(Vector6d update)
{
    // 1.
    // order in 'update_'
    // dP, dV, dPhi, dBiasGyr, dBiasAcc

    // 2.
    // the same as Forster 15'RSS
    // pi = pi + Ri*dpi,    pj = pj + Rj*dpj
    // vi = vi + dvi,       vj = vj + dvj
    // Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)
    //      Note: the optimized bias term is the 'delta bias'
    // delta_biasg_i = delta_biasg_i + dbgi,    delta_biasg_j = delta_biasg_j + dbgj
    // delta_biasa_i = delta_biasa_i + dbai,    delta_biasa_j = delta_biasa_j + dbaj

    Vector3d upd_P = update.segment<3>(0);
    Vector3d upd_Phi = update.segment<3>(3);

    // rotation matrix before update
    //Matrix3d R = Get_qR().toRotationMatrix();
    Matrix3d R = Get_R().matrix();

    // position
    _P += R * upd_P;
    // rotation
    SO3Type dR = SO3Type::exp(upd_Phi);
    _R = Get_R()*dR;

}

}
