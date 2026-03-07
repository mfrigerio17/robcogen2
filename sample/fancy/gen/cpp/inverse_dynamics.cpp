#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"


// Initialization of static-const data

const typename fancy::rcg2::InverseDynamics::ExtForces
fancy::rcg2::InverseDynamics::zeroExtForces(Force::Zero());


fancy::rcg2::InverseDynamics::InverseDynamics(const InertiaProperties& inertia, Transforms& transforms) :
    // the local aliases for the inertia tensors:
    I_link1( inertia.getTensor_link1() ),
    I_link2( inertia.getTensor_link2() ),
    I_link3( inertia.getTensor_link3() ),
    I_link4( inertia.getTensor_link4() ),
    I_link5( inertia.getTensor_link5() ),
    xt(transforms)
{
    v_link1.setZero();
    v_link2.setZero();
    v_link3.setZero();
    v_link4.setZero();
    v_link5.setZero();

    vcross.setZero();
}


void fancy::rcg2::InverseDynamics::InverseDynamics::G_terms(JState_t& tau)
{
    using namespace iit::rbd;
    // Link 'link1'
    a_link1 = (xt.m_link1_X_base0.link1_XM_base0()).matrix().col(iit::rbd::LZ) * iit::rbd::g;
    f_link1 = I_link1 * a_link1;

    // Link 'link2'
    a_link2 = (xt.m_link2_X_link1.link2_XM_link1()) * a_link1;
    f_link2 = I_link2 * a_link2;

    // Link 'link3'
    a_link3 = (xt.m_link3_X_link2.link3_XM_link2()) * a_link2;
    f_link3 = I_link3 * a_link3;

    // Link 'link4'
    a_link4 = (xt.m_link4_X_link3.link4_XM_link3()) * a_link3;
    f_link4 = I_link4 * a_link4;

    // Link 'link5'
    a_link5 = (xt.m_link5_X_link4.link5_XM_link4()) * a_link4;
    f_link5 = I_link5 * a_link5;


    secondPass(tau);
}

void fancy::rcg2::InverseDynamics::InverseDynamics::C_terms(JState_t& tau, const JState_t& qd)
{
    using namespace iit::rbd;
    // Link 'link1'
    v_link1(AZ) = qd(jA);   // v_link1 = vJ, for the first link of a fixed base robot
    f_link1 = vxIv(qd(jA), I_link1);

    // Link 'link2'
    v_link2 = xt.m_link2_X_link1.link2_XM_link1() * v_link1;
    v_link2(LZ) += qd(jB);
    motionCrossProductMx<Scalar>(v_link2, vcross);

    a_link2 = vcross.col(LZ) * qd(jB);
    f_link2 = I_link2 * a_link2 + vxIv(v_link2, I_link2);

    // Link 'link3'
    v_link3 = xt.m_link3_X_link2.link3_XM_link2() * v_link2;
    v_link3(AZ) += qd(jC);
    motionCrossProductMx<Scalar>(v_link3, vcross);

    a_link3 = xt.m_link3_X_link2.link3_XM_link2() * a_link2 + vcross.col(AZ) * qd(jC);
    f_link3 = I_link3 * a_link3 + vxIv(v_link3, I_link3);

    // Link 'link4'
    v_link4 = xt.m_link4_X_link3.link4_XM_link3() * v_link3;
    v_link4(LZ) += qd(jD);
    motionCrossProductMx<Scalar>(v_link4, vcross);

    a_link4 = xt.m_link4_X_link3.link4_XM_link3() * a_link3 + vcross.col(LZ) * qd(jD);
    f_link4 = I_link4 * a_link4 + vxIv(v_link4, I_link4);

    // Link 'link5'
    v_link5 = xt.m_link5_X_link4.link5_XM_link4() * v_link4;
    v_link5(AZ) += qd(jE);
    motionCrossProductMx<Scalar>(v_link5, vcross);

    a_link5 = xt.m_link5_X_link4.link5_XM_link4() * a_link4 + vcross.col(AZ) * qd(jE);
    f_link5 = I_link5 * a_link5 + vxIv(v_link5, I_link5);


    secondPass(tau);
}


void fancy::rcg2::InverseDynamics::InverseDynamics::firstPass(const JState_t& qd, const JState_t& qdd, const ExtForces& fext)
{
    using namespace iit::rbd;
    // Link 'link1'
    v_link1(AZ) = qd(jA);   // v_link1 = vJ, for the first link of a fixed base robot
    a_link1 = xt.m_link1_X_base0.link1_XM_base0().matrix().col(LZ) * iit::rbd::g;
    a_link1(AZ) += qdd(jA);
    f_link1 = I_link1 * a_link1 + vxIv(qd(jA), I_link1) - fext[link1];

    // Link 'link2'
    v_link2 = xt.m_link2_X_link1.link2_XM_link1() * v_link1;
    v_link2(LZ) += qd(jB);

    motionCrossProductMx<Scalar>(v_link2, vcross);

    a_link2 = xt.m_link2_X_link1.link2_XM_link1() * a_link1 + vcross.col(LZ) * qd(jB);
    a_link2(LZ) += qdd(jB);

    f_link2 = I_link2 * a_link2 + vxIv(v_link2, I_link2) - fext[link2];

    // Link 'link3'
    v_link3 = xt.m_link3_X_link2.link3_XM_link2() * v_link2;
    v_link3(AZ) += qd(jC);

    motionCrossProductMx<Scalar>(v_link3, vcross);

    a_link3 = xt.m_link3_X_link2.link3_XM_link2() * a_link2 + vcross.col(AZ) * qd(jC);
    a_link3(AZ) += qdd(jC);

    f_link3 = I_link3 * a_link3 + vxIv(v_link3, I_link3) - fext[link3];

    // Link 'link4'
    v_link4 = xt.m_link4_X_link3.link4_XM_link3() * v_link3;
    v_link4(LZ) += qd(jD);

    motionCrossProductMx<Scalar>(v_link4, vcross);

    a_link4 = xt.m_link4_X_link3.link4_XM_link3() * a_link3 + vcross.col(LZ) * qd(jD);
    a_link4(LZ) += qdd(jD);

    f_link4 = I_link4 * a_link4 + vxIv(v_link4, I_link4) - fext[link4];

    // Link 'link5'
    v_link5 = xt.m_link5_X_link4.link5_XM_link4() * v_link4;
    v_link5(AZ) += qd(jE);

    motionCrossProductMx<Scalar>(v_link5, vcross);

    a_link5 = xt.m_link5_X_link4.link5_XM_link4() * a_link4 + vcross.col(AZ) * qd(jE);
    a_link5(AZ) += qdd(jE);

    f_link5 = I_link5 * a_link5 + vxIv(v_link5, I_link5) - fext[link5];


}


void fancy::rcg2::InverseDynamics::InverseDynamics::secondPass(JState_t& tau)
{
    using namespace iit::rbd;
    // Link 'link5'
    tau(jE) = f_link5(AZ);
    f_link4 += xt.m_link5_X_link4.link4_XF_link5() * f_link5;

    // Link 'link4'
    tau(jD) = f_link4(LZ);
    f_link3 += xt.m_link4_X_link3.link3_XF_link4() * f_link4;

    // Link 'link3'
    tau(jC) = f_link3(AZ);
    f_link2 += xt.m_link3_X_link2.link2_XF_link3() * f_link3;

    // Link 'link2'
    tau(jB) = f_link2(LZ);
    f_link1 += xt.m_link2_X_link1.link1_XF_link2() * f_link2;

    // Link 'link1'
    tau(jA) = f_link1(AZ);

}

