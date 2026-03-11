#include "forward_dynamics.h"


// Initialization of static-const data

const typename hyq::rcg2::ForwardDynamics::ExtForces
hyq::rcg2::ForwardDynamics::zeroExtForces(Force::Zero());


hyq::rcg2::ForwardDynamics::ForwardDynamics(const InertiaProperties& inertia, Transforms& transforms) :
   ip(inertia), xt(transforms)
{
    v_LF_hipassembly.setZero();
    c_LF_hipassembly.setZero();
    v_LF_upperleg.setZero();
    c_LF_upperleg.setZero();
    v_LF_lowerleg.setZero();
    c_LF_lowerleg.setZero();
    v_RF_hipassembly.setZero();
    c_RF_hipassembly.setZero();
    v_RF_upperleg.setZero();
    c_RF_upperleg.setZero();
    v_RF_lowerleg.setZero();
    c_RF_lowerleg.setZero();
    v_LH_hipassembly.setZero();
    c_LH_hipassembly.setZero();
    v_LH_upperleg.setZero();
    c_LH_upperleg.setZero();
    v_LH_lowerleg.setZero();
    c_LH_lowerleg.setZero();
    v_RH_hipassembly.setZero();
    c_RH_hipassembly.setZero();
    v_RH_upperleg.setZero();
    c_RH_upperleg.setZero();
    v_RH_lowerleg.setZero();
    c_RH_lowerleg.setZero();
    vcross.setZero();
    IaB.setZero(); //not really necessary, but avoids warning
}



void hyq::rcg2::ForwardDynamics::ForwardDynamics::fd(
    JState_t& qdd,
    Acceleration& a_trunk,
    const Velocity& v_trunk, const Acceleration& gravity,
    const JState_t& qd, const JState_t& tau, const ExtForces& fext)
{
    using namespace iit::rbd;

    IA_trunk = ip.getTensor_trunk();
    p_trunk = - fext[trunk];
    IA_LF_hipassembly = ip.getTensor_LF_hipassembly();
    p_LF_hipassembly = - fext[LF_hipassembly];
    IA_LF_upperleg = ip.getTensor_LF_upperleg();
    p_LF_upperleg = - fext[LF_upperleg];
    IA_LF_lowerleg = ip.getTensor_LF_lowerleg();
    p_LF_lowerleg = - fext[LF_lowerleg];
    IA_RF_hipassembly = ip.getTensor_RF_hipassembly();
    p_RF_hipassembly = - fext[RF_hipassembly];
    IA_RF_upperleg = ip.getTensor_RF_upperleg();
    p_RF_upperleg = - fext[RF_upperleg];
    IA_RF_lowerleg = ip.getTensor_RF_lowerleg();
    p_RF_lowerleg = - fext[RF_lowerleg];
    IA_LH_hipassembly = ip.getTensor_LH_hipassembly();
    p_LH_hipassembly = - fext[LH_hipassembly];
    IA_LH_upperleg = ip.getTensor_LH_upperleg();
    p_LH_upperleg = - fext[LH_upperleg];
    IA_LH_lowerleg = ip.getTensor_LH_lowerleg();
    p_LH_lowerleg = - fext[LH_lowerleg];
    IA_RH_hipassembly = ip.getTensor_RH_hipassembly();
    p_RH_hipassembly = - fext[RH_hipassembly];
    IA_RH_upperleg = ip.getTensor_RH_upperleg();
    p_RH_upperleg = - fext[RH_upperleg];
    IA_RH_lowerleg = ip.getTensor_RH_lowerleg();
    p_RH_lowerleg = - fext[RH_lowerleg];

// ---------------------- FIRST PASS ---------------------- //
// Note that, during the first pass, the articulated inertias are really
//  just the spatial inertia of the links (see assignments above).
//  Afterwards things change, and articulated inertias shall not be used
//  in functions which work specifically with spatial inertias.


    // + Link LF_hipassembly
    //  - The spatial velocity:
    v_LF_hipassembly = xt.m_LF_hipassembly_X_trunk.LF_hipassembly_XM_trunk() * v_trunk;
    v_LF_hipassembly(AZ) += qd(LF_HAA);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_LF_hipassembly, vcross);
    c_LF_hipassembly = vcross.col(AZ) * qd(LF_HAA);

    //  - The bias force term:
    p_LF_hipassembly += vxIv(v_LF_hipassembly, IA_LF_hipassembly);

    // + Link LF_upperleg
    //  - The spatial velocity:
    v_LF_upperleg = xt.m_LF_upperleg_X_LF_hipassembly.LF_upperleg_XM_LF_hipassembly() * v_LF_hipassembly;
    v_LF_upperleg(AZ) += qd(LF_HFE);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_LF_upperleg, vcross);
    c_LF_upperleg = vcross.col(AZ) * qd(LF_HFE);

    //  - The bias force term:
    p_LF_upperleg += vxIv(v_LF_upperleg, IA_LF_upperleg);

    // + Link LF_lowerleg
    //  - The spatial velocity:
    v_LF_lowerleg = xt.m_LF_lowerleg_X_LF_upperleg.LF_lowerleg_XM_LF_upperleg() * v_LF_upperleg;
    v_LF_lowerleg(AZ) += qd(LF_KFE);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_LF_lowerleg, vcross);
    c_LF_lowerleg = vcross.col(AZ) * qd(LF_KFE);

    //  - The bias force term:
    p_LF_lowerleg += vxIv(v_LF_lowerleg, IA_LF_lowerleg);

    // + Link RF_hipassembly
    //  - The spatial velocity:
    v_RF_hipassembly = xt.m_RF_hipassembly_X_trunk.RF_hipassembly_XM_trunk() * v_trunk;
    v_RF_hipassembly(AZ) += qd(RF_HAA);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_RF_hipassembly, vcross);
    c_RF_hipassembly = vcross.col(AZ) * qd(RF_HAA);

    //  - The bias force term:
    p_RF_hipassembly += vxIv(v_RF_hipassembly, IA_RF_hipassembly);

    // + Link RF_upperleg
    //  - The spatial velocity:
    v_RF_upperleg = xt.m_RF_upperleg_X_RF_hipassembly.RF_upperleg_XM_RF_hipassembly() * v_RF_hipassembly;
    v_RF_upperleg(AZ) += qd(RF_HFE);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_RF_upperleg, vcross);
    c_RF_upperleg = vcross.col(AZ) * qd(RF_HFE);

    //  - The bias force term:
    p_RF_upperleg += vxIv(v_RF_upperleg, IA_RF_upperleg);

    // + Link RF_lowerleg
    //  - The spatial velocity:
    v_RF_lowerleg = xt.m_RF_lowerleg_X_RF_upperleg.RF_lowerleg_XM_RF_upperleg() * v_RF_upperleg;
    v_RF_lowerleg(AZ) += qd(RF_KFE);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_RF_lowerleg, vcross);
    c_RF_lowerleg = vcross.col(AZ) * qd(RF_KFE);

    //  - The bias force term:
    p_RF_lowerleg += vxIv(v_RF_lowerleg, IA_RF_lowerleg);

    // + Link LH_hipassembly
    //  - The spatial velocity:
    v_LH_hipassembly = xt.m_LH_hipassembly_X_trunk.LH_hipassembly_XM_trunk() * v_trunk;
    v_LH_hipassembly(AZ) += qd(LH_HAA);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_LH_hipassembly, vcross);
    c_LH_hipassembly = vcross.col(AZ) * qd(LH_HAA);

    //  - The bias force term:
    p_LH_hipassembly += vxIv(v_LH_hipassembly, IA_LH_hipassembly);

    // + Link LH_upperleg
    //  - The spatial velocity:
    v_LH_upperleg = xt.m_LH_upperleg_X_LH_hipassembly.LH_upperleg_XM_LH_hipassembly() * v_LH_hipassembly;
    v_LH_upperleg(AZ) += qd(LH_HFE);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_LH_upperleg, vcross);
    c_LH_upperleg = vcross.col(AZ) * qd(LH_HFE);

    //  - The bias force term:
    p_LH_upperleg += vxIv(v_LH_upperleg, IA_LH_upperleg);

    // + Link LH_lowerleg
    //  - The spatial velocity:
    v_LH_lowerleg = xt.m_LH_lowerleg_X_LH_upperleg.LH_lowerleg_XM_LH_upperleg() * v_LH_upperleg;
    v_LH_lowerleg(AZ) += qd(LH_KFE);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_LH_lowerleg, vcross);
    c_LH_lowerleg = vcross.col(AZ) * qd(LH_KFE);

    //  - The bias force term:
    p_LH_lowerleg += vxIv(v_LH_lowerleg, IA_LH_lowerleg);

    // + Link RH_hipassembly
    //  - The spatial velocity:
    v_RH_hipassembly = xt.m_RH_hipassembly_X_trunk.RH_hipassembly_XM_trunk() * v_trunk;
    v_RH_hipassembly(AZ) += qd(RH_HAA);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_RH_hipassembly, vcross);
    c_RH_hipassembly = vcross.col(AZ) * qd(RH_HAA);

    //  - The bias force term:
    p_RH_hipassembly += vxIv(v_RH_hipassembly, IA_RH_hipassembly);

    // + Link RH_upperleg
    //  - The spatial velocity:
    v_RH_upperleg = xt.m_RH_upperleg_X_RH_hipassembly.RH_upperleg_XM_RH_hipassembly() * v_RH_hipassembly;
    v_RH_upperleg(AZ) += qd(RH_HFE);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_RH_upperleg, vcross);
    c_RH_upperleg = vcross.col(AZ) * qd(RH_HFE);

    //  - The bias force term:
    p_RH_upperleg += vxIv(v_RH_upperleg, IA_RH_upperleg);

    // + Link RH_lowerleg
    //  - The spatial velocity:
    v_RH_lowerleg = xt.m_RH_lowerleg_X_RH_upperleg.RH_lowerleg_XM_RH_upperleg() * v_RH_upperleg;
    v_RH_lowerleg(AZ) += qd(RH_KFE);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_RH_lowerleg, vcross);
    c_RH_lowerleg = vcross.col(AZ) * qd(RH_KFE);

    //  - The bias force term:
    p_RH_lowerleg += vxIv(v_RH_lowerleg, IA_RH_lowerleg);

    // Bias force on the floating base
    p_trunk += vxIv(v_trunk, IA_trunk);

// ---------------------- SECOND PASS ---------------------- //
    Force pa;

    // + Link RH_lowerleg
    RH_lowerleg_u = tau(RH_KFE) - p_RH_lowerleg(AZ);
    RH_lowerleg_U = IA_RH_lowerleg.col(AZ);
    RH_lowerleg_D = RH_lowerleg_U(AZ);

    compute_Ia_revolute(IA_RH_lowerleg, RH_lowerleg_U, RH_lowerleg_D, Ia_r);  // same as: Ia_r = IA_RH_lowerleg - RH_lowerleg_U/RH_lowerleg_D * RH_lowerleg_U.transpose();
    pa = p_RH_lowerleg + Ia_r * c_RH_lowerleg + RH_lowerleg_U * RH_lowerleg_u/RH_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, xt.m_RH_lowerleg_X_RH_upperleg.ct, IaB);
    IA_RH_upperleg += IaB;
    p_RH_upperleg += xt.m_RH_lowerleg_X_RH_upperleg.RH_upperleg_XF_RH_lowerleg() * pa;

    // + Link RH_upperleg
    RH_upperleg_u = tau(RH_HFE) - p_RH_upperleg(AZ);
    RH_upperleg_U = IA_RH_upperleg.col(AZ);
    RH_upperleg_D = RH_upperleg_U(AZ);

    compute_Ia_revolute(IA_RH_upperleg, RH_upperleg_U, RH_upperleg_D, Ia_r);  // same as: Ia_r = IA_RH_upperleg - RH_upperleg_U/RH_upperleg_D * RH_upperleg_U.transpose();
    pa = p_RH_upperleg + Ia_r * c_RH_upperleg + RH_upperleg_U * RH_upperleg_u/RH_upperleg_D;
    ctransform_Ia_revolute(Ia_r, xt.m_RH_upperleg_X_RH_hipassembly.ct, IaB);
    IA_RH_hipassembly += IaB;
    p_RH_hipassembly += xt.m_RH_upperleg_X_RH_hipassembly.RH_hipassembly_XF_RH_upperleg() * pa;

    // + Link RH_hipassembly
    RH_hipassembly_u = tau(RH_HAA) - p_RH_hipassembly(AZ);
    RH_hipassembly_U = IA_RH_hipassembly.col(AZ);
    RH_hipassembly_D = RH_hipassembly_U(AZ);

    compute_Ia_revolute(IA_RH_hipassembly, RH_hipassembly_U, RH_hipassembly_D, Ia_r);  // same as: Ia_r = IA_RH_hipassembly - RH_hipassembly_U/RH_hipassembly_D * RH_hipassembly_U.transpose();
    pa = p_RH_hipassembly + Ia_r * c_RH_hipassembly + RH_hipassembly_U * RH_hipassembly_u/RH_hipassembly_D;
    ctransform_Ia_revolute(Ia_r, xt.m_RH_hipassembly_X_trunk.ct, IaB);
    IA_trunk += IaB;
    p_trunk += xt.m_RH_hipassembly_X_trunk.trunk_XF_RH_hipassembly() * pa;

    // + Link LH_lowerleg
    LH_lowerleg_u = tau(LH_KFE) - p_LH_lowerleg(AZ);
    LH_lowerleg_U = IA_LH_lowerleg.col(AZ);
    LH_lowerleg_D = LH_lowerleg_U(AZ);

    compute_Ia_revolute(IA_LH_lowerleg, LH_lowerleg_U, LH_lowerleg_D, Ia_r);  // same as: Ia_r = IA_LH_lowerleg - LH_lowerleg_U/LH_lowerleg_D * LH_lowerleg_U.transpose();
    pa = p_LH_lowerleg + Ia_r * c_LH_lowerleg + LH_lowerleg_U * LH_lowerleg_u/LH_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, xt.m_LH_lowerleg_X_LH_upperleg.ct, IaB);
    IA_LH_upperleg += IaB;
    p_LH_upperleg += xt.m_LH_lowerleg_X_LH_upperleg.LH_upperleg_XF_LH_lowerleg() * pa;

    // + Link LH_upperleg
    LH_upperleg_u = tau(LH_HFE) - p_LH_upperleg(AZ);
    LH_upperleg_U = IA_LH_upperleg.col(AZ);
    LH_upperleg_D = LH_upperleg_U(AZ);

    compute_Ia_revolute(IA_LH_upperleg, LH_upperleg_U, LH_upperleg_D, Ia_r);  // same as: Ia_r = IA_LH_upperleg - LH_upperleg_U/LH_upperleg_D * LH_upperleg_U.transpose();
    pa = p_LH_upperleg + Ia_r * c_LH_upperleg + LH_upperleg_U * LH_upperleg_u/LH_upperleg_D;
    ctransform_Ia_revolute(Ia_r, xt.m_LH_upperleg_X_LH_hipassembly.ct, IaB);
    IA_LH_hipassembly += IaB;
    p_LH_hipassembly += xt.m_LH_upperleg_X_LH_hipassembly.LH_hipassembly_XF_LH_upperleg() * pa;

    // + Link LH_hipassembly
    LH_hipassembly_u = tau(LH_HAA) - p_LH_hipassembly(AZ);
    LH_hipassembly_U = IA_LH_hipassembly.col(AZ);
    LH_hipassembly_D = LH_hipassembly_U(AZ);

    compute_Ia_revolute(IA_LH_hipassembly, LH_hipassembly_U, LH_hipassembly_D, Ia_r);  // same as: Ia_r = IA_LH_hipassembly - LH_hipassembly_U/LH_hipassembly_D * LH_hipassembly_U.transpose();
    pa = p_LH_hipassembly + Ia_r * c_LH_hipassembly + LH_hipassembly_U * LH_hipassembly_u/LH_hipassembly_D;
    ctransform_Ia_revolute(Ia_r, xt.m_LH_hipassembly_X_trunk.ct, IaB);
    IA_trunk += IaB;
    p_trunk += xt.m_LH_hipassembly_X_trunk.trunk_XF_LH_hipassembly() * pa;

    // + Link RF_lowerleg
    RF_lowerleg_u = tau(RF_KFE) - p_RF_lowerleg(AZ);
    RF_lowerleg_U = IA_RF_lowerleg.col(AZ);
    RF_lowerleg_D = RF_lowerleg_U(AZ);

    compute_Ia_revolute(IA_RF_lowerleg, RF_lowerleg_U, RF_lowerleg_D, Ia_r);  // same as: Ia_r = IA_RF_lowerleg - RF_lowerleg_U/RF_lowerleg_D * RF_lowerleg_U.transpose();
    pa = p_RF_lowerleg + Ia_r * c_RF_lowerleg + RF_lowerleg_U * RF_lowerleg_u/RF_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, xt.m_RF_lowerleg_X_RF_upperleg.ct, IaB);
    IA_RF_upperleg += IaB;
    p_RF_upperleg += xt.m_RF_lowerleg_X_RF_upperleg.RF_upperleg_XF_RF_lowerleg() * pa;

    // + Link RF_upperleg
    RF_upperleg_u = tau(RF_HFE) - p_RF_upperleg(AZ);
    RF_upperleg_U = IA_RF_upperleg.col(AZ);
    RF_upperleg_D = RF_upperleg_U(AZ);

    compute_Ia_revolute(IA_RF_upperleg, RF_upperleg_U, RF_upperleg_D, Ia_r);  // same as: Ia_r = IA_RF_upperleg - RF_upperleg_U/RF_upperleg_D * RF_upperleg_U.transpose();
    pa = p_RF_upperleg + Ia_r * c_RF_upperleg + RF_upperleg_U * RF_upperleg_u/RF_upperleg_D;
    ctransform_Ia_revolute(Ia_r, xt.m_RF_upperleg_X_RF_hipassembly.ct, IaB);
    IA_RF_hipassembly += IaB;
    p_RF_hipassembly += xt.m_RF_upperleg_X_RF_hipassembly.RF_hipassembly_XF_RF_upperleg() * pa;

    // + Link RF_hipassembly
    RF_hipassembly_u = tau(RF_HAA) - p_RF_hipassembly(AZ);
    RF_hipassembly_U = IA_RF_hipassembly.col(AZ);
    RF_hipassembly_D = RF_hipassembly_U(AZ);

    compute_Ia_revolute(IA_RF_hipassembly, RF_hipassembly_U, RF_hipassembly_D, Ia_r);  // same as: Ia_r = IA_RF_hipassembly - RF_hipassembly_U/RF_hipassembly_D * RF_hipassembly_U.transpose();
    pa = p_RF_hipassembly + Ia_r * c_RF_hipassembly + RF_hipassembly_U * RF_hipassembly_u/RF_hipassembly_D;
    ctransform_Ia_revolute(Ia_r, xt.m_RF_hipassembly_X_trunk.ct, IaB);
    IA_trunk += IaB;
    p_trunk += xt.m_RF_hipassembly_X_trunk.trunk_XF_RF_hipassembly() * pa;

    // + Link LF_lowerleg
    LF_lowerleg_u = tau(LF_KFE) - p_LF_lowerleg(AZ);
    LF_lowerleg_U = IA_LF_lowerleg.col(AZ);
    LF_lowerleg_D = LF_lowerleg_U(AZ);

    compute_Ia_revolute(IA_LF_lowerleg, LF_lowerleg_U, LF_lowerleg_D, Ia_r);  // same as: Ia_r = IA_LF_lowerleg - LF_lowerleg_U/LF_lowerleg_D * LF_lowerleg_U.transpose();
    pa = p_LF_lowerleg + Ia_r * c_LF_lowerleg + LF_lowerleg_U * LF_lowerleg_u/LF_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, xt.m_LF_lowerleg_X_LF_upperleg.ct, IaB);
    IA_LF_upperleg += IaB;
    p_LF_upperleg += xt.m_LF_lowerleg_X_LF_upperleg.LF_upperleg_XF_LF_lowerleg() * pa;

    // + Link LF_upperleg
    LF_upperleg_u = tau(LF_HFE) - p_LF_upperleg(AZ);
    LF_upperleg_U = IA_LF_upperleg.col(AZ);
    LF_upperleg_D = LF_upperleg_U(AZ);

    compute_Ia_revolute(IA_LF_upperleg, LF_upperleg_U, LF_upperleg_D, Ia_r);  // same as: Ia_r = IA_LF_upperleg - LF_upperleg_U/LF_upperleg_D * LF_upperleg_U.transpose();
    pa = p_LF_upperleg + Ia_r * c_LF_upperleg + LF_upperleg_U * LF_upperleg_u/LF_upperleg_D;
    ctransform_Ia_revolute(Ia_r, xt.m_LF_upperleg_X_LF_hipassembly.ct, IaB);
    IA_LF_hipassembly += IaB;
    p_LF_hipassembly += xt.m_LF_upperleg_X_LF_hipassembly.LF_hipassembly_XF_LF_upperleg() * pa;

    // + Link LF_hipassembly
    LF_hipassembly_u = tau(LF_HAA) - p_LF_hipassembly(AZ);
    LF_hipassembly_U = IA_LF_hipassembly.col(AZ);
    LF_hipassembly_D = LF_hipassembly_U(AZ);

    compute_Ia_revolute(IA_LF_hipassembly, LF_hipassembly_U, LF_hipassembly_D, Ia_r);  // same as: Ia_r = IA_LF_hipassembly - LF_hipassembly_U/LF_hipassembly_D * LF_hipassembly_U.transpose();
    pa = p_LF_hipassembly + Ia_r * c_LF_hipassembly + LF_hipassembly_U * LF_hipassembly_u/LF_hipassembly_D;
    ctransform_Ia_revolute(Ia_r, xt.m_LF_hipassembly_X_trunk.ct, IaB);
    IA_trunk += IaB;
    p_trunk += xt.m_LF_hipassembly_X_trunk.trunk_XF_LF_hipassembly() * pa;

    // + The acceleration of the floating base trunk, without gravity
    a_trunk = - IA_trunk.llt().solve(p_trunk);  // a_trunk = - IA^-1 * p_trunk

// ---------------------- THIRD PASS ---------------------- //
    a_LF_hipassembly = xt.m_LF_hipassembly_X_trunk.LF_hipassembly_XM_trunk() * a_trunk + c_LF_hipassembly;
    qdd(LF_HAA) = (LF_hipassembly_u - LF_hipassembly_U.dot(a_LF_hipassembly)) / LF_hipassembly_D;
    a_LF_hipassembly(AZ) += qdd(LF_HAA);

    a_LF_upperleg = xt.m_LF_upperleg_X_LF_hipassembly.LF_upperleg_XM_LF_hipassembly() * a_LF_hipassembly + c_LF_upperleg;
    qdd(LF_HFE) = (LF_upperleg_u - LF_upperleg_U.dot(a_LF_upperleg)) / LF_upperleg_D;
    a_LF_upperleg(AZ) += qdd(LF_HFE);

    a_LF_lowerleg = xt.m_LF_lowerleg_X_LF_upperleg.LF_lowerleg_XM_LF_upperleg() * a_LF_upperleg + c_LF_lowerleg;
    qdd(LF_KFE) = (LF_lowerleg_u - LF_lowerleg_U.dot(a_LF_lowerleg)) / LF_lowerleg_D;
    a_LF_lowerleg(AZ) += qdd(LF_KFE);

    a_RF_hipassembly = xt.m_RF_hipassembly_X_trunk.RF_hipassembly_XM_trunk() * a_trunk + c_RF_hipassembly;
    qdd(RF_HAA) = (RF_hipassembly_u - RF_hipassembly_U.dot(a_RF_hipassembly)) / RF_hipassembly_D;
    a_RF_hipassembly(AZ) += qdd(RF_HAA);

    a_RF_upperleg = xt.m_RF_upperleg_X_RF_hipassembly.RF_upperleg_XM_RF_hipassembly() * a_RF_hipassembly + c_RF_upperleg;
    qdd(RF_HFE) = (RF_upperleg_u - RF_upperleg_U.dot(a_RF_upperleg)) / RF_upperleg_D;
    a_RF_upperleg(AZ) += qdd(RF_HFE);

    a_RF_lowerleg = xt.m_RF_lowerleg_X_RF_upperleg.RF_lowerleg_XM_RF_upperleg() * a_RF_upperleg + c_RF_lowerleg;
    qdd(RF_KFE) = (RF_lowerleg_u - RF_lowerleg_U.dot(a_RF_lowerleg)) / RF_lowerleg_D;
    a_RF_lowerleg(AZ) += qdd(RF_KFE);

    a_LH_hipassembly = xt.m_LH_hipassembly_X_trunk.LH_hipassembly_XM_trunk() * a_trunk + c_LH_hipassembly;
    qdd(LH_HAA) = (LH_hipassembly_u - LH_hipassembly_U.dot(a_LH_hipassembly)) / LH_hipassembly_D;
    a_LH_hipassembly(AZ) += qdd(LH_HAA);

    a_LH_upperleg = xt.m_LH_upperleg_X_LH_hipassembly.LH_upperleg_XM_LH_hipassembly() * a_LH_hipassembly + c_LH_upperleg;
    qdd(LH_HFE) = (LH_upperleg_u - LH_upperleg_U.dot(a_LH_upperleg)) / LH_upperleg_D;
    a_LH_upperleg(AZ) += qdd(LH_HFE);

    a_LH_lowerleg = xt.m_LH_lowerleg_X_LH_upperleg.LH_lowerleg_XM_LH_upperleg() * a_LH_upperleg + c_LH_lowerleg;
    qdd(LH_KFE) = (LH_lowerleg_u - LH_lowerleg_U.dot(a_LH_lowerleg)) / LH_lowerleg_D;
    a_LH_lowerleg(AZ) += qdd(LH_KFE);

    a_RH_hipassembly = xt.m_RH_hipassembly_X_trunk.RH_hipassembly_XM_trunk() * a_trunk + c_RH_hipassembly;
    qdd(RH_HAA) = (RH_hipassembly_u - RH_hipassembly_U.dot(a_RH_hipassembly)) / RH_hipassembly_D;
    a_RH_hipassembly(AZ) += qdd(RH_HAA);

    a_RH_upperleg = xt.m_RH_upperleg_X_RH_hipassembly.RH_upperleg_XM_RH_hipassembly() * a_RH_hipassembly + c_RH_upperleg;
    qdd(RH_HFE) = (RH_upperleg_u - RH_upperleg_U.dot(a_RH_upperleg)) / RH_upperleg_D;
    a_RH_upperleg(AZ) += qdd(RH_HFE);

    a_RH_lowerleg = xt.m_RH_lowerleg_X_RH_upperleg.RH_lowerleg_XM_RH_upperleg() * a_RH_upperleg + c_RH_lowerleg;
    qdd(RH_KFE) = (RH_lowerleg_u - RH_lowerleg_U.dot(a_RH_lowerleg)) / RH_lowerleg_D;
    a_RH_lowerleg(AZ) += qdd(RH_KFE);


    // + Add gravity to the acceleration of the floating base
    a_trunk += gravity;
}

