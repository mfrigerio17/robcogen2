#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"


// Initialization of static-const data

const typename hyq::rcg2::InverseDynamics::ExtForces
hyq::rcg2::InverseDynamics::zeroExtForces(Force::Zero());


hyq::rcg2::InverseDynamics::InverseDynamics(const InertiaProperties& inertia, Transforms& transforms) :
    // the local aliases for the inertia tensors:
    I_LF_hipassembly( inertia.getTensor_LF_hipassembly() ),
    I_LF_upperleg( inertia.getTensor_LF_upperleg() ),
    I_LF_lowerleg( inertia.getTensor_LF_lowerleg() ),
    I_RF_hipassembly( inertia.getTensor_RF_hipassembly() ),
    I_RF_upperleg( inertia.getTensor_RF_upperleg() ),
    I_RF_lowerleg( inertia.getTensor_RF_lowerleg() ),
    I_LH_hipassembly( inertia.getTensor_LH_hipassembly() ),
    I_LH_upperleg( inertia.getTensor_LH_upperleg() ),
    I_LH_lowerleg( inertia.getTensor_LH_lowerleg() ),
    I_RH_hipassembly( inertia.getTensor_RH_hipassembly() ),
    I_RH_upperleg( inertia.getTensor_RH_upperleg() ),
    I_RH_lowerleg( inertia.getTensor_RH_lowerleg() ),
    I_trunk( inertia.getTensor_trunk() ),
    // the composite inertia of leaf links IS the regular inertia
    Ic_LF_lowerleg(I_LF_lowerleg),
    Ic_RF_lowerleg(I_RF_lowerleg),
    Ic_LH_lowerleg(I_LH_lowerleg),
    Ic_RH_lowerleg(I_RH_lowerleg),
    xt(transforms)
{
    v_LF_hipassembly.setZero();
    v_LF_upperleg.setZero();
    v_LF_lowerleg.setZero();
    v_RF_hipassembly.setZero();
    v_RF_upperleg.setZero();
    v_RF_lowerleg.setZero();
    v_LH_hipassembly.setZero();
    v_LH_upperleg.setZero();
    v_LH_lowerleg.setZero();
    v_RH_hipassembly.setZero();
    v_RH_upperleg.setZero();
    v_RH_lowerleg.setZero();

    vcross.setZero();
}


void hyq::rcg2::InverseDynamics::InverseDynamics::G_terms_fully_actuated(Force& base_f, JState_t& tau,
        const Acceleration& gravity)
{
    using namespace iit::rbd;
    const Acceleration a_trunk{-gravity};
    f_trunk = I_trunk * a_trunk;

    // Link 'LF_hipassembly'
    a_LF_hipassembly = (xt.m_LF_hipassembly_X_trunk.LF_hipassembly_XM_trunk()) * a_trunk;
    f_LF_hipassembly = I_LF_hipassembly * a_LF_hipassembly;

    // Link 'LF_upperleg'
    a_LF_upperleg = (xt.m_LF_upperleg_X_LF_hipassembly.LF_upperleg_XM_LF_hipassembly()) * a_LF_hipassembly;
    f_LF_upperleg = I_LF_upperleg * a_LF_upperleg;

    // Link 'LF_lowerleg'
    a_LF_lowerleg = (xt.m_LF_lowerleg_X_LF_upperleg.LF_lowerleg_XM_LF_upperleg()) * a_LF_upperleg;
    f_LF_lowerleg = I_LF_lowerleg * a_LF_lowerleg;

    // Link 'RF_hipassembly'
    a_RF_hipassembly = (xt.m_RF_hipassembly_X_trunk.RF_hipassembly_XM_trunk()) * a_trunk;
    f_RF_hipassembly = I_RF_hipassembly * a_RF_hipassembly;

    // Link 'RF_upperleg'
    a_RF_upperleg = (xt.m_RF_upperleg_X_RF_hipassembly.RF_upperleg_XM_RF_hipassembly()) * a_RF_hipassembly;
    f_RF_upperleg = I_RF_upperleg * a_RF_upperleg;

    // Link 'RF_lowerleg'
    a_RF_lowerleg = (xt.m_RF_lowerleg_X_RF_upperleg.RF_lowerleg_XM_RF_upperleg()) * a_RF_upperleg;
    f_RF_lowerleg = I_RF_lowerleg * a_RF_lowerleg;

    // Link 'LH_hipassembly'
    a_LH_hipassembly = (xt.m_LH_hipassembly_X_trunk.LH_hipassembly_XM_trunk()) * a_trunk;
    f_LH_hipassembly = I_LH_hipassembly * a_LH_hipassembly;

    // Link 'LH_upperleg'
    a_LH_upperleg = (xt.m_LH_upperleg_X_LH_hipassembly.LH_upperleg_XM_LH_hipassembly()) * a_LH_hipassembly;
    f_LH_upperleg = I_LH_upperleg * a_LH_upperleg;

    // Link 'LH_lowerleg'
    a_LH_lowerleg = (xt.m_LH_lowerleg_X_LH_upperleg.LH_lowerleg_XM_LH_upperleg()) * a_LH_upperleg;
    f_LH_lowerleg = I_LH_lowerleg * a_LH_lowerleg;

    // Link 'RH_hipassembly'
    a_RH_hipassembly = (xt.m_RH_hipassembly_X_trunk.RH_hipassembly_XM_trunk()) * a_trunk;
    f_RH_hipassembly = I_RH_hipassembly * a_RH_hipassembly;

    // Link 'RH_upperleg'
    a_RH_upperleg = (xt.m_RH_upperleg_X_RH_hipassembly.RH_upperleg_XM_RH_hipassembly()) * a_RH_hipassembly;
    f_RH_upperleg = I_RH_upperleg * a_RH_upperleg;

    // Link 'RH_lowerleg'
    a_RH_lowerleg = (xt.m_RH_lowerleg_X_RH_upperleg.RH_lowerleg_XM_RH_upperleg()) * a_RH_upperleg;
    f_RH_lowerleg = I_RH_lowerleg * a_RH_lowerleg;


    sweep_inwards_fully_actuated(tau);

    base_f = f_trunk;
}


void hyq::rcg2::InverseDynamics::InverseDynamics::C_terms_fully_actuated(Force& base_f, JState_t& tau, const Velocity& v_trunk, const JState_t& qd)
{
    using namespace iit::rbd;
    f_trunk = vxIv(v_trunk, I_trunk);

    // Link 'LF_hipassembly'
    v_LF_hipassembly = xt.m_LF_hipassembly_X_trunk.LF_hipassembly_XM_trunk() * v_trunk;
    v_LF_hipassembly(AZ) += qd(LF_HAA);
    motionCrossProductMx<Scalar>(v_LF_hipassembly, vcross);

    a_LF_hipassembly = vcross.col(AZ) * qd(LF_HAA);
    f_LF_hipassembly = I_LF_hipassembly * a_LF_hipassembly + vxIv(v_LF_hipassembly, I_LF_hipassembly);

    // Link 'LF_upperleg'
    v_LF_upperleg = xt.m_LF_upperleg_X_LF_hipassembly.LF_upperleg_XM_LF_hipassembly() * v_LF_hipassembly;
    v_LF_upperleg(AZ) += qd(LF_HFE);
    motionCrossProductMx<Scalar>(v_LF_upperleg, vcross);

    a_LF_upperleg = xt.m_LF_upperleg_X_LF_hipassembly.LF_upperleg_XM_LF_hipassembly() * a_LF_hipassembly + vcross.col(AZ) * qd(LF_HFE);
    f_LF_upperleg = I_LF_upperleg * a_LF_upperleg + vxIv(v_LF_upperleg, I_LF_upperleg);

    // Link 'LF_lowerleg'
    v_LF_lowerleg = xt.m_LF_lowerleg_X_LF_upperleg.LF_lowerleg_XM_LF_upperleg() * v_LF_upperleg;
    v_LF_lowerleg(AZ) += qd(LF_KFE);
    motionCrossProductMx<Scalar>(v_LF_lowerleg, vcross);

    a_LF_lowerleg = xt.m_LF_lowerleg_X_LF_upperleg.LF_lowerleg_XM_LF_upperleg() * a_LF_upperleg + vcross.col(AZ) * qd(LF_KFE);
    f_LF_lowerleg = I_LF_lowerleg * a_LF_lowerleg + vxIv(v_LF_lowerleg, I_LF_lowerleg);

    // Link 'RF_hipassembly'
    v_RF_hipassembly = xt.m_RF_hipassembly_X_trunk.RF_hipassembly_XM_trunk() * v_trunk;
    v_RF_hipassembly(AZ) += qd(RF_HAA);
    motionCrossProductMx<Scalar>(v_RF_hipassembly, vcross);

    a_RF_hipassembly = vcross.col(AZ) * qd(RF_HAA);
    f_RF_hipassembly = I_RF_hipassembly * a_RF_hipassembly + vxIv(v_RF_hipassembly, I_RF_hipassembly);

    // Link 'RF_upperleg'
    v_RF_upperleg = xt.m_RF_upperleg_X_RF_hipassembly.RF_upperleg_XM_RF_hipassembly() * v_RF_hipassembly;
    v_RF_upperleg(AZ) += qd(RF_HFE);
    motionCrossProductMx<Scalar>(v_RF_upperleg, vcross);

    a_RF_upperleg = xt.m_RF_upperleg_X_RF_hipassembly.RF_upperleg_XM_RF_hipassembly() * a_RF_hipassembly + vcross.col(AZ) * qd(RF_HFE);
    f_RF_upperleg = I_RF_upperleg * a_RF_upperleg + vxIv(v_RF_upperleg, I_RF_upperleg);

    // Link 'RF_lowerleg'
    v_RF_lowerleg = xt.m_RF_lowerleg_X_RF_upperleg.RF_lowerleg_XM_RF_upperleg() * v_RF_upperleg;
    v_RF_lowerleg(AZ) += qd(RF_KFE);
    motionCrossProductMx<Scalar>(v_RF_lowerleg, vcross);

    a_RF_lowerleg = xt.m_RF_lowerleg_X_RF_upperleg.RF_lowerleg_XM_RF_upperleg() * a_RF_upperleg + vcross.col(AZ) * qd(RF_KFE);
    f_RF_lowerleg = I_RF_lowerleg * a_RF_lowerleg + vxIv(v_RF_lowerleg, I_RF_lowerleg);

    // Link 'LH_hipassembly'
    v_LH_hipassembly = xt.m_LH_hipassembly_X_trunk.LH_hipassembly_XM_trunk() * v_trunk;
    v_LH_hipassembly(AZ) += qd(LH_HAA);
    motionCrossProductMx<Scalar>(v_LH_hipassembly, vcross);

    a_LH_hipassembly = vcross.col(AZ) * qd(LH_HAA);
    f_LH_hipassembly = I_LH_hipassembly * a_LH_hipassembly + vxIv(v_LH_hipassembly, I_LH_hipassembly);

    // Link 'LH_upperleg'
    v_LH_upperleg = xt.m_LH_upperleg_X_LH_hipassembly.LH_upperleg_XM_LH_hipassembly() * v_LH_hipassembly;
    v_LH_upperleg(AZ) += qd(LH_HFE);
    motionCrossProductMx<Scalar>(v_LH_upperleg, vcross);

    a_LH_upperleg = xt.m_LH_upperleg_X_LH_hipassembly.LH_upperleg_XM_LH_hipassembly() * a_LH_hipassembly + vcross.col(AZ) * qd(LH_HFE);
    f_LH_upperleg = I_LH_upperleg * a_LH_upperleg + vxIv(v_LH_upperleg, I_LH_upperleg);

    // Link 'LH_lowerleg'
    v_LH_lowerleg = xt.m_LH_lowerleg_X_LH_upperleg.LH_lowerleg_XM_LH_upperleg() * v_LH_upperleg;
    v_LH_lowerleg(AZ) += qd(LH_KFE);
    motionCrossProductMx<Scalar>(v_LH_lowerleg, vcross);

    a_LH_lowerleg = xt.m_LH_lowerleg_X_LH_upperleg.LH_lowerleg_XM_LH_upperleg() * a_LH_upperleg + vcross.col(AZ) * qd(LH_KFE);
    f_LH_lowerleg = I_LH_lowerleg * a_LH_lowerleg + vxIv(v_LH_lowerleg, I_LH_lowerleg);

    // Link 'RH_hipassembly'
    v_RH_hipassembly = xt.m_RH_hipassembly_X_trunk.RH_hipassembly_XM_trunk() * v_trunk;
    v_RH_hipassembly(AZ) += qd(RH_HAA);
    motionCrossProductMx<Scalar>(v_RH_hipassembly, vcross);

    a_RH_hipassembly = vcross.col(AZ) * qd(RH_HAA);
    f_RH_hipassembly = I_RH_hipassembly * a_RH_hipassembly + vxIv(v_RH_hipassembly, I_RH_hipassembly);

    // Link 'RH_upperleg'
    v_RH_upperleg = xt.m_RH_upperleg_X_RH_hipassembly.RH_upperleg_XM_RH_hipassembly() * v_RH_hipassembly;
    v_RH_upperleg(AZ) += qd(RH_HFE);
    motionCrossProductMx<Scalar>(v_RH_upperleg, vcross);

    a_RH_upperleg = xt.m_RH_upperleg_X_RH_hipassembly.RH_upperleg_XM_RH_hipassembly() * a_RH_hipassembly + vcross.col(AZ) * qd(RH_HFE);
    f_RH_upperleg = I_RH_upperleg * a_RH_upperleg + vxIv(v_RH_upperleg, I_RH_upperleg);

    // Link 'RH_lowerleg'
    v_RH_lowerleg = xt.m_RH_lowerleg_X_RH_upperleg.RH_lowerleg_XM_RH_upperleg() * v_RH_upperleg;
    v_RH_lowerleg(AZ) += qd(RH_KFE);
    motionCrossProductMx<Scalar>(v_RH_lowerleg, vcross);

    a_RH_lowerleg = xt.m_RH_lowerleg_X_RH_upperleg.RH_lowerleg_XM_RH_upperleg() * a_RH_upperleg + vcross.col(AZ) * qd(RH_KFE);
    f_RH_lowerleg = I_RH_lowerleg * a_RH_lowerleg + vxIv(v_RH_lowerleg, I_RH_lowerleg);


    sweep_inwards_fully_actuated(tau);

    base_f = f_trunk;
}


void hyq::rcg2::InverseDynamics::InverseDynamics::id_fully_actuated(
    Force& base_f, JState_t& tau,
    const Acceleration& gravity, const Velocity& v_trunk, const Acceleration& base_a,
    const JState_t& qd, const JState_t& qdd,
    const ExtForces& fext /*= zeroExtForces*/)
{
    using namespace iit::rbd;
    Acceleration a_trunk = base_a - gravity;
    f_trunk = I_trunk * a_trunk + vxIv(v_trunk, I_trunk) - fext[trunk];

    // Link 'LF_hipassembly'
    v_LF_hipassembly = xt.m_LF_hipassembly_X_trunk.LF_hipassembly_XM_trunk() * v_trunk;
    v_LF_hipassembly(AZ) += qd(LF_HAA);

    motionCrossProductMx<Scalar>(v_LF_hipassembly, vcross);

    a_LF_hipassembly = xt.m_LF_hipassembly_X_trunk.LF_hipassembly_XM_trunk() * a_trunk + vcross.col(AZ) * qd(LF_HAA);
    a_LF_hipassembly(AZ) += qdd(LF_HAA);

    f_LF_hipassembly = I_LF_hipassembly * a_LF_hipassembly + vxIv(v_LF_hipassembly, I_LF_hipassembly) - fext[LF_hipassembly];

    // Link 'LF_upperleg'
    v_LF_upperleg = xt.m_LF_upperleg_X_LF_hipassembly.LF_upperleg_XM_LF_hipassembly() * v_LF_hipassembly;
    v_LF_upperleg(AZ) += qd(LF_HFE);

    motionCrossProductMx<Scalar>(v_LF_upperleg, vcross);

    a_LF_upperleg = xt.m_LF_upperleg_X_LF_hipassembly.LF_upperleg_XM_LF_hipassembly() * a_LF_hipassembly + vcross.col(AZ) * qd(LF_HFE);
    a_LF_upperleg(AZ) += qdd(LF_HFE);

    f_LF_upperleg = I_LF_upperleg * a_LF_upperleg + vxIv(v_LF_upperleg, I_LF_upperleg) - fext[LF_upperleg];

    // Link 'LF_lowerleg'
    v_LF_lowerleg = xt.m_LF_lowerleg_X_LF_upperleg.LF_lowerleg_XM_LF_upperleg() * v_LF_upperleg;
    v_LF_lowerleg(AZ) += qd(LF_KFE);

    motionCrossProductMx<Scalar>(v_LF_lowerleg, vcross);

    a_LF_lowerleg = xt.m_LF_lowerleg_X_LF_upperleg.LF_lowerleg_XM_LF_upperleg() * a_LF_upperleg + vcross.col(AZ) * qd(LF_KFE);
    a_LF_lowerleg(AZ) += qdd(LF_KFE);

    f_LF_lowerleg = I_LF_lowerleg * a_LF_lowerleg + vxIv(v_LF_lowerleg, I_LF_lowerleg) - fext[LF_lowerleg];

    // Link 'RF_hipassembly'
    v_RF_hipassembly = xt.m_RF_hipassembly_X_trunk.RF_hipassembly_XM_trunk() * v_trunk;
    v_RF_hipassembly(AZ) += qd(RF_HAA);

    motionCrossProductMx<Scalar>(v_RF_hipassembly, vcross);

    a_RF_hipassembly = xt.m_RF_hipassembly_X_trunk.RF_hipassembly_XM_trunk() * a_trunk + vcross.col(AZ) * qd(RF_HAA);
    a_RF_hipassembly(AZ) += qdd(RF_HAA);

    f_RF_hipassembly = I_RF_hipassembly * a_RF_hipassembly + vxIv(v_RF_hipassembly, I_RF_hipassembly) - fext[RF_hipassembly];

    // Link 'RF_upperleg'
    v_RF_upperleg = xt.m_RF_upperleg_X_RF_hipassembly.RF_upperleg_XM_RF_hipassembly() * v_RF_hipassembly;
    v_RF_upperleg(AZ) += qd(RF_HFE);

    motionCrossProductMx<Scalar>(v_RF_upperleg, vcross);

    a_RF_upperleg = xt.m_RF_upperleg_X_RF_hipassembly.RF_upperleg_XM_RF_hipassembly() * a_RF_hipassembly + vcross.col(AZ) * qd(RF_HFE);
    a_RF_upperleg(AZ) += qdd(RF_HFE);

    f_RF_upperleg = I_RF_upperleg * a_RF_upperleg + vxIv(v_RF_upperleg, I_RF_upperleg) - fext[RF_upperleg];

    // Link 'RF_lowerleg'
    v_RF_lowerleg = xt.m_RF_lowerleg_X_RF_upperleg.RF_lowerleg_XM_RF_upperleg() * v_RF_upperleg;
    v_RF_lowerleg(AZ) += qd(RF_KFE);

    motionCrossProductMx<Scalar>(v_RF_lowerleg, vcross);

    a_RF_lowerleg = xt.m_RF_lowerleg_X_RF_upperleg.RF_lowerleg_XM_RF_upperleg() * a_RF_upperleg + vcross.col(AZ) * qd(RF_KFE);
    a_RF_lowerleg(AZ) += qdd(RF_KFE);

    f_RF_lowerleg = I_RF_lowerleg * a_RF_lowerleg + vxIv(v_RF_lowerleg, I_RF_lowerleg) - fext[RF_lowerleg];

    // Link 'LH_hipassembly'
    v_LH_hipassembly = xt.m_LH_hipassembly_X_trunk.LH_hipassembly_XM_trunk() * v_trunk;
    v_LH_hipassembly(AZ) += qd(LH_HAA);

    motionCrossProductMx<Scalar>(v_LH_hipassembly, vcross);

    a_LH_hipassembly = xt.m_LH_hipassembly_X_trunk.LH_hipassembly_XM_trunk() * a_trunk + vcross.col(AZ) * qd(LH_HAA);
    a_LH_hipassembly(AZ) += qdd(LH_HAA);

    f_LH_hipassembly = I_LH_hipassembly * a_LH_hipassembly + vxIv(v_LH_hipassembly, I_LH_hipassembly) - fext[LH_hipassembly];

    // Link 'LH_upperleg'
    v_LH_upperleg = xt.m_LH_upperleg_X_LH_hipassembly.LH_upperleg_XM_LH_hipassembly() * v_LH_hipassembly;
    v_LH_upperleg(AZ) += qd(LH_HFE);

    motionCrossProductMx<Scalar>(v_LH_upperleg, vcross);

    a_LH_upperleg = xt.m_LH_upperleg_X_LH_hipassembly.LH_upperleg_XM_LH_hipassembly() * a_LH_hipassembly + vcross.col(AZ) * qd(LH_HFE);
    a_LH_upperleg(AZ) += qdd(LH_HFE);

    f_LH_upperleg = I_LH_upperleg * a_LH_upperleg + vxIv(v_LH_upperleg, I_LH_upperleg) - fext[LH_upperleg];

    // Link 'LH_lowerleg'
    v_LH_lowerleg = xt.m_LH_lowerleg_X_LH_upperleg.LH_lowerleg_XM_LH_upperleg() * v_LH_upperleg;
    v_LH_lowerleg(AZ) += qd(LH_KFE);

    motionCrossProductMx<Scalar>(v_LH_lowerleg, vcross);

    a_LH_lowerleg = xt.m_LH_lowerleg_X_LH_upperleg.LH_lowerleg_XM_LH_upperleg() * a_LH_upperleg + vcross.col(AZ) * qd(LH_KFE);
    a_LH_lowerleg(AZ) += qdd(LH_KFE);

    f_LH_lowerleg = I_LH_lowerleg * a_LH_lowerleg + vxIv(v_LH_lowerleg, I_LH_lowerleg) - fext[LH_lowerleg];

    // Link 'RH_hipassembly'
    v_RH_hipassembly = xt.m_RH_hipassembly_X_trunk.RH_hipassembly_XM_trunk() * v_trunk;
    v_RH_hipassembly(AZ) += qd(RH_HAA);

    motionCrossProductMx<Scalar>(v_RH_hipassembly, vcross);

    a_RH_hipassembly = xt.m_RH_hipassembly_X_trunk.RH_hipassembly_XM_trunk() * a_trunk + vcross.col(AZ) * qd(RH_HAA);
    a_RH_hipassembly(AZ) += qdd(RH_HAA);

    f_RH_hipassembly = I_RH_hipassembly * a_RH_hipassembly + vxIv(v_RH_hipassembly, I_RH_hipassembly) - fext[RH_hipassembly];

    // Link 'RH_upperleg'
    v_RH_upperleg = xt.m_RH_upperleg_X_RH_hipassembly.RH_upperleg_XM_RH_hipassembly() * v_RH_hipassembly;
    v_RH_upperleg(AZ) += qd(RH_HFE);

    motionCrossProductMx<Scalar>(v_RH_upperleg, vcross);

    a_RH_upperleg = xt.m_RH_upperleg_X_RH_hipassembly.RH_upperleg_XM_RH_hipassembly() * a_RH_hipassembly + vcross.col(AZ) * qd(RH_HFE);
    a_RH_upperleg(AZ) += qdd(RH_HFE);

    f_RH_upperleg = I_RH_upperleg * a_RH_upperleg + vxIv(v_RH_upperleg, I_RH_upperleg) - fext[RH_upperleg];

    // Link 'RH_lowerleg'
    v_RH_lowerleg = xt.m_RH_lowerleg_X_RH_upperleg.RH_lowerleg_XM_RH_upperleg() * v_RH_upperleg;
    v_RH_lowerleg(AZ) += qd(RH_KFE);

    motionCrossProductMx<Scalar>(v_RH_lowerleg, vcross);

    a_RH_lowerleg = xt.m_RH_lowerleg_X_RH_upperleg.RH_lowerleg_XM_RH_upperleg() * a_RH_upperleg + vcross.col(AZ) * qd(RH_KFE);
    a_RH_lowerleg(AZ) += qdd(RH_KFE);

    f_RH_lowerleg = I_RH_lowerleg * a_RH_lowerleg + vxIv(v_RH_lowerleg, I_RH_lowerleg) - fext[RH_lowerleg];



    sweep_inwards_fully_actuated(tau);

    base_f = f_trunk;
}


void hyq::rcg2::InverseDynamics::InverseDynamics::sweep_inwards_fully_actuated(JState_t& tau)
{
    using namespace iit::rbd;
    // Link 'RH_lowerleg'
    tau(RH_KFE) = f_RH_lowerleg(AZ);
    f_RH_upperleg += xt.m_RH_lowerleg_X_RH_upperleg.RH_upperleg_XF_RH_lowerleg() * f_RH_lowerleg;

    // Link 'RH_upperleg'
    tau(RH_HFE) = f_RH_upperleg(AZ);
    f_RH_hipassembly += xt.m_RH_upperleg_X_RH_hipassembly.RH_hipassembly_XF_RH_upperleg() * f_RH_upperleg;

    // Link 'RH_hipassembly'
    tau(RH_HAA) = f_RH_hipassembly(AZ);
    f_trunk += xt.m_RH_hipassembly_X_trunk.trunk_XF_RH_hipassembly() * f_RH_hipassembly;

    // Link 'LH_lowerleg'
    tau(LH_KFE) = f_LH_lowerleg(AZ);
    f_LH_upperleg += xt.m_LH_lowerleg_X_LH_upperleg.LH_upperleg_XF_LH_lowerleg() * f_LH_lowerleg;

    // Link 'LH_upperleg'
    tau(LH_HFE) = f_LH_upperleg(AZ);
    f_LH_hipassembly += xt.m_LH_upperleg_X_LH_hipassembly.LH_hipassembly_XF_LH_upperleg() * f_LH_upperleg;

    // Link 'LH_hipassembly'
    tau(LH_HAA) = f_LH_hipassembly(AZ);
    f_trunk += xt.m_LH_hipassembly_X_trunk.trunk_XF_LH_hipassembly() * f_LH_hipassembly;

    // Link 'RF_lowerleg'
    tau(RF_KFE) = f_RF_lowerleg(AZ);
    f_RF_upperleg += xt.m_RF_lowerleg_X_RF_upperleg.RF_upperleg_XF_RF_lowerleg() * f_RF_lowerleg;

    // Link 'RF_upperleg'
    tau(RF_HFE) = f_RF_upperleg(AZ);
    f_RF_hipassembly += xt.m_RF_upperleg_X_RF_hipassembly.RF_hipassembly_XF_RF_upperleg() * f_RF_upperleg;

    // Link 'RF_hipassembly'
    tau(RF_HAA) = f_RF_hipassembly(AZ);
    f_trunk += xt.m_RF_hipassembly_X_trunk.trunk_XF_RF_hipassembly() * f_RF_hipassembly;

    // Link 'LF_lowerleg'
    tau(LF_KFE) = f_LF_lowerleg(AZ);
    f_LF_upperleg += xt.m_LF_lowerleg_X_LF_upperleg.LF_upperleg_XF_LF_lowerleg() * f_LF_lowerleg;

    // Link 'LF_upperleg'
    tau(LF_HFE) = f_LF_upperleg(AZ);
    f_LF_hipassembly += xt.m_LF_upperleg_X_LF_hipassembly.LF_hipassembly_XF_LF_upperleg() * f_LF_upperleg;

    // Link 'LF_hipassembly'
    tau(LF_HAA) = f_LF_hipassembly(AZ);
    f_trunk += xt.m_LF_hipassembly_X_trunk.trunk_XF_LF_hipassembly() * f_LF_hipassembly;

}


void hyq::rcg2::InverseDynamics::InverseDynamics::id(
    JState_t& tau, Acceleration& a_trunk,
    const Acceleration& gravity, const Velocity& v_trunk,
    const JState_t& qd, const JState_t& qdd,
    const ExtForces& fext /*= zeroExtForces*/)
{
    using namespace iit::rbd;
    f_trunk = vxIv(v_trunk, I_trunk) - fext[trunk];

    // Link 'LF_hipassembly'
    v_LF_hipassembly = xt.m_LF_hipassembly_X_trunk.LF_hipassembly_XM_trunk() * v_trunk;
    v_LF_hipassembly(AZ) += qd(LF_HAA);

    motionCrossProductMx<Scalar>(v_LF_hipassembly, vcross);

    a_LF_hipassembly = vcross.col(AZ) * qd(LF_HAA);
    a_LF_hipassembly(AZ) += qdd(LF_HAA);

    f_LF_hipassembly = I_LF_hipassembly * a_LF_hipassembly + vxIv(v_LF_hipassembly, I_LF_hipassembly) - fext[LF_hipassembly];

    // Link 'LF_upperleg'
    v_LF_upperleg = xt.m_LF_upperleg_X_LF_hipassembly.LF_upperleg_XM_LF_hipassembly() * v_LF_hipassembly;
    v_LF_upperleg(AZ) += qd(LF_HFE);

    motionCrossProductMx<Scalar>(v_LF_upperleg, vcross);

    a_LF_upperleg = xt.m_LF_upperleg_X_LF_hipassembly.LF_upperleg_XM_LF_hipassembly() * a_LF_hipassembly + vcross.col(AZ) * qd(LF_HFE);
    a_LF_upperleg(AZ) += qdd(LF_HFE);

    f_LF_upperleg = I_LF_upperleg * a_LF_upperleg + vxIv(v_LF_upperleg, I_LF_upperleg) - fext[LF_upperleg];

    // Link 'LF_lowerleg'
    v_LF_lowerleg = xt.m_LF_lowerleg_X_LF_upperleg.LF_lowerleg_XM_LF_upperleg() * v_LF_upperleg;
    v_LF_lowerleg(AZ) += qd(LF_KFE);

    motionCrossProductMx<Scalar>(v_LF_lowerleg, vcross);

    a_LF_lowerleg = xt.m_LF_lowerleg_X_LF_upperleg.LF_lowerleg_XM_LF_upperleg() * a_LF_upperleg + vcross.col(AZ) * qd(LF_KFE);
    a_LF_lowerleg(AZ) += qdd(LF_KFE);

    f_LF_lowerleg = I_LF_lowerleg * a_LF_lowerleg + vxIv(v_LF_lowerleg, I_LF_lowerleg) - fext[LF_lowerleg];

    // Link 'RF_hipassembly'
    v_RF_hipassembly = xt.m_RF_hipassembly_X_trunk.RF_hipassembly_XM_trunk() * v_trunk;
    v_RF_hipassembly(AZ) += qd(RF_HAA);

    motionCrossProductMx<Scalar>(v_RF_hipassembly, vcross);

    a_RF_hipassembly = vcross.col(AZ) * qd(RF_HAA);
    a_RF_hipassembly(AZ) += qdd(RF_HAA);

    f_RF_hipassembly = I_RF_hipassembly * a_RF_hipassembly + vxIv(v_RF_hipassembly, I_RF_hipassembly) - fext[RF_hipassembly];

    // Link 'RF_upperleg'
    v_RF_upperleg = xt.m_RF_upperleg_X_RF_hipassembly.RF_upperleg_XM_RF_hipassembly() * v_RF_hipassembly;
    v_RF_upperleg(AZ) += qd(RF_HFE);

    motionCrossProductMx<Scalar>(v_RF_upperleg, vcross);

    a_RF_upperleg = xt.m_RF_upperleg_X_RF_hipassembly.RF_upperleg_XM_RF_hipassembly() * a_RF_hipassembly + vcross.col(AZ) * qd(RF_HFE);
    a_RF_upperleg(AZ) += qdd(RF_HFE);

    f_RF_upperleg = I_RF_upperleg * a_RF_upperleg + vxIv(v_RF_upperleg, I_RF_upperleg) - fext[RF_upperleg];

    // Link 'RF_lowerleg'
    v_RF_lowerleg = xt.m_RF_lowerleg_X_RF_upperleg.RF_lowerleg_XM_RF_upperleg() * v_RF_upperleg;
    v_RF_lowerleg(AZ) += qd(RF_KFE);

    motionCrossProductMx<Scalar>(v_RF_lowerleg, vcross);

    a_RF_lowerleg = xt.m_RF_lowerleg_X_RF_upperleg.RF_lowerleg_XM_RF_upperleg() * a_RF_upperleg + vcross.col(AZ) * qd(RF_KFE);
    a_RF_lowerleg(AZ) += qdd(RF_KFE);

    f_RF_lowerleg = I_RF_lowerleg * a_RF_lowerleg + vxIv(v_RF_lowerleg, I_RF_lowerleg) - fext[RF_lowerleg];

    // Link 'LH_hipassembly'
    v_LH_hipassembly = xt.m_LH_hipassembly_X_trunk.LH_hipassembly_XM_trunk() * v_trunk;
    v_LH_hipassembly(AZ) += qd(LH_HAA);

    motionCrossProductMx<Scalar>(v_LH_hipassembly, vcross);

    a_LH_hipassembly = vcross.col(AZ) * qd(LH_HAA);
    a_LH_hipassembly(AZ) += qdd(LH_HAA);

    f_LH_hipassembly = I_LH_hipassembly * a_LH_hipassembly + vxIv(v_LH_hipassembly, I_LH_hipassembly) - fext[LH_hipassembly];

    // Link 'LH_upperleg'
    v_LH_upperleg = xt.m_LH_upperleg_X_LH_hipassembly.LH_upperleg_XM_LH_hipassembly() * v_LH_hipassembly;
    v_LH_upperleg(AZ) += qd(LH_HFE);

    motionCrossProductMx<Scalar>(v_LH_upperleg, vcross);

    a_LH_upperleg = xt.m_LH_upperleg_X_LH_hipassembly.LH_upperleg_XM_LH_hipassembly() * a_LH_hipassembly + vcross.col(AZ) * qd(LH_HFE);
    a_LH_upperleg(AZ) += qdd(LH_HFE);

    f_LH_upperleg = I_LH_upperleg * a_LH_upperleg + vxIv(v_LH_upperleg, I_LH_upperleg) - fext[LH_upperleg];

    // Link 'LH_lowerleg'
    v_LH_lowerleg = xt.m_LH_lowerleg_X_LH_upperleg.LH_lowerleg_XM_LH_upperleg() * v_LH_upperleg;
    v_LH_lowerleg(AZ) += qd(LH_KFE);

    motionCrossProductMx<Scalar>(v_LH_lowerleg, vcross);

    a_LH_lowerleg = xt.m_LH_lowerleg_X_LH_upperleg.LH_lowerleg_XM_LH_upperleg() * a_LH_upperleg + vcross.col(AZ) * qd(LH_KFE);
    a_LH_lowerleg(AZ) += qdd(LH_KFE);

    f_LH_lowerleg = I_LH_lowerleg * a_LH_lowerleg + vxIv(v_LH_lowerleg, I_LH_lowerleg) - fext[LH_lowerleg];

    // Link 'RH_hipassembly'
    v_RH_hipassembly = xt.m_RH_hipassembly_X_trunk.RH_hipassembly_XM_trunk() * v_trunk;
    v_RH_hipassembly(AZ) += qd(RH_HAA);

    motionCrossProductMx<Scalar>(v_RH_hipassembly, vcross);

    a_RH_hipassembly = vcross.col(AZ) * qd(RH_HAA);
    a_RH_hipassembly(AZ) += qdd(RH_HAA);

    f_RH_hipassembly = I_RH_hipassembly * a_RH_hipassembly + vxIv(v_RH_hipassembly, I_RH_hipassembly) - fext[RH_hipassembly];

    // Link 'RH_upperleg'
    v_RH_upperleg = xt.m_RH_upperleg_X_RH_hipassembly.RH_upperleg_XM_RH_hipassembly() * v_RH_hipassembly;
    v_RH_upperleg(AZ) += qd(RH_HFE);

    motionCrossProductMx<Scalar>(v_RH_upperleg, vcross);

    a_RH_upperleg = xt.m_RH_upperleg_X_RH_hipassembly.RH_upperleg_XM_RH_hipassembly() * a_RH_hipassembly + vcross.col(AZ) * qd(RH_HFE);
    a_RH_upperleg(AZ) += qdd(RH_HFE);

    f_RH_upperleg = I_RH_upperleg * a_RH_upperleg + vxIv(v_RH_upperleg, I_RH_upperleg) - fext[RH_upperleg];

    // Link 'RH_lowerleg'
    v_RH_lowerleg = xt.m_RH_lowerleg_X_RH_upperleg.RH_lowerleg_XM_RH_upperleg() * v_RH_upperleg;
    v_RH_lowerleg(AZ) += qd(RH_KFE);

    motionCrossProductMx<Scalar>(v_RH_lowerleg, vcross);

    a_RH_lowerleg = xt.m_RH_lowerleg_X_RH_upperleg.RH_lowerleg_XM_RH_upperleg() * a_RH_upperleg + vcross.col(AZ) * qd(RH_KFE);
    a_RH_lowerleg(AZ) += qdd(RH_KFE);

    f_RH_lowerleg = I_RH_lowerleg * a_RH_lowerleg + vxIv(v_RH_lowerleg, I_RH_lowerleg) - fext[RH_lowerleg];



    // Second pass //
    // ----------- //
    // propagate inwards wrenches and composite inertia

    InertiaMatrix Ic_aux;
    // initialize the composite inertias
    Ic_trunk = I_trunk;
    Ic_LF_hipassembly = I_LF_hipassembly;
    Ic_LF_upperleg = I_LF_upperleg;
    Ic_RF_hipassembly = I_RF_hipassembly;
    Ic_RF_upperleg = I_RF_upperleg;
    Ic_LH_hipassembly = I_LH_hipassembly;
    Ic_LH_upperleg = I_LH_upperleg;
    Ic_RH_hipassembly = I_RH_hipassembly;
    Ic_RH_upperleg = I_RH_upperleg;

    // Link 'RH_lowerleg'
    iit::rbd::transformInertia<Scalar>(Ic_RH_lowerleg, xt.m_RH_lowerleg_X_RH_upperleg.ct, Ic_aux);
    Ic_RH_upperleg += Ic_aux;
    f_RH_upperleg += xt.m_RH_lowerleg_X_RH_upperleg.RH_upperleg_XF_RH_lowerleg() * f_RH_lowerleg;

    // Link 'RH_upperleg'
    iit::rbd::transformInertia<Scalar>(Ic_RH_upperleg, xt.m_RH_upperleg_X_RH_hipassembly.ct, Ic_aux);
    Ic_RH_hipassembly += Ic_aux;
    f_RH_hipassembly += xt.m_RH_upperleg_X_RH_hipassembly.RH_hipassembly_XF_RH_upperleg() * f_RH_upperleg;

    // Link 'RH_hipassembly'
    iit::rbd::transformInertia<Scalar>(Ic_RH_hipassembly, xt.m_RH_hipassembly_X_trunk.ct, Ic_aux);
    Ic_trunk += Ic_aux;
    f_trunk += xt.m_RH_hipassembly_X_trunk.trunk_XF_RH_hipassembly() * f_RH_hipassembly;

    // Link 'LH_lowerleg'
    iit::rbd::transformInertia<Scalar>(Ic_LH_lowerleg, xt.m_LH_lowerleg_X_LH_upperleg.ct, Ic_aux);
    Ic_LH_upperleg += Ic_aux;
    f_LH_upperleg += xt.m_LH_lowerleg_X_LH_upperleg.LH_upperleg_XF_LH_lowerleg() * f_LH_lowerleg;

    // Link 'LH_upperleg'
    iit::rbd::transformInertia<Scalar>(Ic_LH_upperleg, xt.m_LH_upperleg_X_LH_hipassembly.ct, Ic_aux);
    Ic_LH_hipassembly += Ic_aux;
    f_LH_hipassembly += xt.m_LH_upperleg_X_LH_hipassembly.LH_hipassembly_XF_LH_upperleg() * f_LH_upperleg;

    // Link 'LH_hipassembly'
    iit::rbd::transformInertia<Scalar>(Ic_LH_hipassembly, xt.m_LH_hipassembly_X_trunk.ct, Ic_aux);
    Ic_trunk += Ic_aux;
    f_trunk += xt.m_LH_hipassembly_X_trunk.trunk_XF_LH_hipassembly() * f_LH_hipassembly;

    // Link 'RF_lowerleg'
    iit::rbd::transformInertia<Scalar>(Ic_RF_lowerleg, xt.m_RF_lowerleg_X_RF_upperleg.ct, Ic_aux);
    Ic_RF_upperleg += Ic_aux;
    f_RF_upperleg += xt.m_RF_lowerleg_X_RF_upperleg.RF_upperleg_XF_RF_lowerleg() * f_RF_lowerleg;

    // Link 'RF_upperleg'
    iit::rbd::transformInertia<Scalar>(Ic_RF_upperleg, xt.m_RF_upperleg_X_RF_hipassembly.ct, Ic_aux);
    Ic_RF_hipassembly += Ic_aux;
    f_RF_hipassembly += xt.m_RF_upperleg_X_RF_hipassembly.RF_hipassembly_XF_RF_upperleg() * f_RF_upperleg;

    // Link 'RF_hipassembly'
    iit::rbd::transformInertia<Scalar>(Ic_RF_hipassembly, xt.m_RF_hipassembly_X_trunk.ct, Ic_aux);
    Ic_trunk += Ic_aux;
    f_trunk += xt.m_RF_hipassembly_X_trunk.trunk_XF_RF_hipassembly() * f_RF_hipassembly;

    // Link 'LF_lowerleg'
    iit::rbd::transformInertia<Scalar>(Ic_LF_lowerleg, xt.m_LF_lowerleg_X_LF_upperleg.ct, Ic_aux);
    Ic_LF_upperleg += Ic_aux;
    f_LF_upperleg += xt.m_LF_lowerleg_X_LF_upperleg.LF_upperleg_XF_LF_lowerleg() * f_LF_lowerleg;

    // Link 'LF_upperleg'
    iit::rbd::transformInertia<Scalar>(Ic_LF_upperleg, xt.m_LF_upperleg_X_LF_hipassembly.ct, Ic_aux);
    Ic_LF_hipassembly += Ic_aux;
    f_LF_hipassembly += xt.m_LF_upperleg_X_LF_hipassembly.LF_hipassembly_XF_LF_upperleg() * f_LF_upperleg;

    // Link 'LF_hipassembly'
    iit::rbd::transformInertia<Scalar>(Ic_LF_hipassembly, xt.m_LF_hipassembly_X_trunk.ct, Ic_aux);
    Ic_trunk += Ic_aux;
    f_trunk += xt.m_LF_hipassembly_X_trunk.trunk_XF_LF_hipassembly() * f_LF_hipassembly;


    // The base acceleration due to the force due to the motion of the links
    a_trunk = - Ic_trunk.inverse() * f_trunk;

    // Third pass //
    // ---------- //
    // propagate outwards the base acceleration and get the joint torques

    a_LF_hipassembly = xt.m_LF_hipassembly_X_trunk.LF_hipassembly_XM_trunk() * a_trunk;
    tau(LF_HAA) = (Ic_LF_hipassembly.row(AZ) * a_LF_hipassembly + f_LF_hipassembly(AZ));

    a_LF_upperleg = xt.m_LF_upperleg_X_LF_hipassembly.LF_upperleg_XM_LF_hipassembly() * a_LF_hipassembly;
    tau(LF_HFE) = (Ic_LF_upperleg.row(AZ) * a_LF_upperleg + f_LF_upperleg(AZ));

    a_LF_lowerleg = xt.m_LF_lowerleg_X_LF_upperleg.LF_lowerleg_XM_LF_upperleg() * a_LF_upperleg;
    tau(LF_KFE) = (Ic_LF_lowerleg.row(AZ) * a_LF_lowerleg + f_LF_lowerleg(AZ));

    a_RF_hipassembly = xt.m_RF_hipassembly_X_trunk.RF_hipassembly_XM_trunk() * a_trunk;
    tau(RF_HAA) = (Ic_RF_hipassembly.row(AZ) * a_RF_hipassembly + f_RF_hipassembly(AZ));

    a_RF_upperleg = xt.m_RF_upperleg_X_RF_hipassembly.RF_upperleg_XM_RF_hipassembly() * a_RF_hipassembly;
    tau(RF_HFE) = (Ic_RF_upperleg.row(AZ) * a_RF_upperleg + f_RF_upperleg(AZ));

    a_RF_lowerleg = xt.m_RF_lowerleg_X_RF_upperleg.RF_lowerleg_XM_RF_upperleg() * a_RF_upperleg;
    tau(RF_KFE) = (Ic_RF_lowerleg.row(AZ) * a_RF_lowerleg + f_RF_lowerleg(AZ));

    a_LH_hipassembly = xt.m_LH_hipassembly_X_trunk.LH_hipassembly_XM_trunk() * a_trunk;
    tau(LH_HAA) = (Ic_LH_hipassembly.row(AZ) * a_LH_hipassembly + f_LH_hipassembly(AZ));

    a_LH_upperleg = xt.m_LH_upperleg_X_LH_hipassembly.LH_upperleg_XM_LH_hipassembly() * a_LH_hipassembly;
    tau(LH_HFE) = (Ic_LH_upperleg.row(AZ) * a_LH_upperleg + f_LH_upperleg(AZ));

    a_LH_lowerleg = xt.m_LH_lowerleg_X_LH_upperleg.LH_lowerleg_XM_LH_upperleg() * a_LH_upperleg;
    tau(LH_KFE) = (Ic_LH_lowerleg.row(AZ) * a_LH_lowerleg + f_LH_lowerleg(AZ));

    a_RH_hipassembly = xt.m_RH_hipassembly_X_trunk.RH_hipassembly_XM_trunk() * a_trunk;
    tau(RH_HAA) = (Ic_RH_hipassembly.row(AZ) * a_RH_hipassembly + f_RH_hipassembly(AZ));

    a_RH_upperleg = xt.m_RH_upperleg_X_RH_hipassembly.RH_upperleg_XM_RH_hipassembly() * a_RH_hipassembly;
    tau(RH_HFE) = (Ic_RH_upperleg.row(AZ) * a_RH_upperleg + f_RH_upperleg(AZ));

    a_RH_lowerleg = xt.m_RH_lowerleg_X_RH_upperleg.RH_lowerleg_XM_RH_upperleg() * a_RH_upperleg;
    tau(RH_KFE) = (Ic_RH_lowerleg.row(AZ) * a_RH_lowerleg + f_RH_lowerleg(AZ));


    a_trunk += gravity;
}

