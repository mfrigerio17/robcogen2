#include "forward_dynamics.h"


// Initialization of static-const data

const typename fancy::rcg2::ForwardDynamics::ExtForces
fancy::rcg2::ForwardDynamics::zeroExtForces(Force::Zero());


fancy::rcg2::ForwardDynamics::ForwardDynamics(const InertiaProperties& inertia, Transforms& transforms) :
   ip(inertia), xt(transforms)
{
    v_link1.setZero();
    c_link1.setZero();
    v_link2.setZero();
    c_link2.setZero();
    v_link3.setZero();
    c_link3.setZero();
    v_link4.setZero();
    c_link4.setZero();
    v_link5.setZero();
    c_link5.setZero();
    vcross.setZero();
    IaB.setZero(); //not really necessary, but avoids warning
}



void fancy::rcg2::ForwardDynamics::ForwardDynamics::fd(
    JState_t& qdd,
    const JState_t& qd, const JState_t& tau, const ExtForces& fext)
{
    using namespace iit::rbd;

    IA_link1 = ip.getTensor_link1();
    p_link1 = - fext[link1];
    IA_link2 = ip.getTensor_link2();
    p_link2 = - fext[link2];
    IA_link3 = ip.getTensor_link3();
    p_link3 = - fext[link3];
    IA_link4 = ip.getTensor_link4();
    p_link4 = - fext[link4];
    IA_link5 = ip.getTensor_link5();
    p_link5 = - fext[link5];

// ---------------------- FIRST PASS ---------------------- //
// Note that, during the first pass, the articulated inertias are really
//  just the spatial inertia of the links (see assignments above).
//  Afterwards things change, and articulated inertias shall not be used
//  in functions which work specifically with spatial inertias.


    // + Link link1
    //  - The spatial velocity:
    v_link1(AZ) = qd(jA);

    //  - The bias force term:
    p_link1 += vxIv(qd(jA), IA_link1);

    // + Link link2
    //  - The spatial velocity:
    v_link2 = xt.m_link2_X_link1.link2_XM_link1() * v_link1;
    v_link2(LZ) += qd(jB);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_link2, vcross);
    c_link2 = vcross.col(LZ) * qd(jB);

    //  - The bias force term:
    p_link2 += vxIv(v_link2, IA_link2);

    // + Link link3
    //  - The spatial velocity:
    v_link3 = xt.m_link3_X_link2.link3_XM_link2() * v_link2;
    v_link3(AZ) += qd(jC);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_link3, vcross);
    c_link3 = vcross.col(AZ) * qd(jC);

    //  - The bias force term:
    p_link3 += vxIv(v_link3, IA_link3);

    // + Link link4
    //  - The spatial velocity:
    v_link4 = xt.m_link4_X_link3.link4_XM_link3() * v_link3;
    v_link4(LZ) += qd(jD);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_link4, vcross);
    c_link4 = vcross.col(LZ) * qd(jD);

    //  - The bias force term:
    p_link4 += vxIv(v_link4, IA_link4);

    // + Link link5
    //  - The spatial velocity:
    v_link5 = xt.m_link5_X_link4.link5_XM_link4() * v_link4;
    v_link5(AZ) += qd(jE);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(v_link5, vcross);
    c_link5 = vcross.col(AZ) * qd(jE);

    //  - The bias force term:
    p_link5 += vxIv(v_link5, IA_link5);


// ---------------------- SECOND PASS ---------------------- //
    Force pa;

    // + Link link5
    link5_u = tau(jE) - p_link5(AZ);
    link5_U = IA_link5.col(AZ);
    link5_D = link5_U(AZ);

    compute_Ia_revolute(IA_link5, link5_U, link5_D, Ia_r);  // same as: Ia_r = IA_link5 - link5_U/link5_D * link5_U.transpose();
    pa = p_link5 + Ia_r * c_link5 + link5_U * link5_u/link5_D;
    ctransform_Ia_revolute(Ia_r, xt.m_link5_X_link4.ct, IaB);
    IA_link4 += IaB;
    p_link4 += xt.m_link5_X_link4.link4_XF_link5() * pa;

    // + Link link4
    link4_u = tau(jD) - p_link4(LZ);
    link4_U = IA_link4.col(LZ);
    link4_D = link4_U(LZ);

    compute_Ia_prismatic(IA_link4, link4_U, link4_D, Ia_p);  // same as: Ia_p = IA_link4 - link4_U/link4_D * link4_U.transpose();
    pa = p_link4 + Ia_p * c_link4 + link4_U * link4_u/link4_D;
    ctransform_Ia_prismatic(Ia_p, xt.m_link4_X_link3.ct, IaB);
    IA_link3 += IaB;
    p_link3 += xt.m_link4_X_link3.link3_XF_link4() * pa;

    // + Link link3
    link3_u = tau(jC) - p_link3(AZ);
    link3_U = IA_link3.col(AZ);
    link3_D = link3_U(AZ);

    compute_Ia_revolute(IA_link3, link3_U, link3_D, Ia_r);  // same as: Ia_r = IA_link3 - link3_U/link3_D * link3_U.transpose();
    pa = p_link3 + Ia_r * c_link3 + link3_U * link3_u/link3_D;
    ctransform_Ia_revolute(Ia_r, xt.m_link3_X_link2.ct, IaB);
    IA_link2 += IaB;
    p_link2 += xt.m_link3_X_link2.link2_XF_link3() * pa;

    // + Link link2
    link2_u = tau(jB) - p_link2(LZ);
    link2_U = IA_link2.col(LZ);
    link2_D = link2_U(LZ);

    compute_Ia_prismatic(IA_link2, link2_U, link2_D, Ia_p);  // same as: Ia_p = IA_link2 - link2_U/link2_D * link2_U.transpose();
    pa = p_link2 + Ia_p * c_link2 + link2_U * link2_u/link2_D;
    ctransform_Ia_prismatic(Ia_p, xt.m_link2_X_link1.ct, IaB);
    IA_link1 += IaB;
    p_link1 += xt.m_link2_X_link1.link1_XF_link2() * pa;

    // + Link link1
    link1_u = tau(jA) - p_link1(AZ);
    link1_U = IA_link1.col(AZ);
    link1_D = link1_U(AZ);



// ---------------------- THIRD PASS ---------------------- //
    a_link1 = xt.m_link1_X_base0.link1_XM_base0().matrix().col(LZ) * iit::rbd::g;
    qdd(jA) = (link1_u - link1_U.dot(a_link1)) / link1_D;
    a_link1(AZ) += qdd(jA);

    a_link2 = xt.m_link2_X_link1.link2_XM_link1() * a_link1 + c_link2;
    qdd(jB) = (link2_u - link2_U.dot(a_link2)) / link2_D;
    a_link2(LZ) += qdd(jB);

    a_link3 = xt.m_link3_X_link2.link3_XM_link2() * a_link2 + c_link3;
    qdd(jC) = (link3_u - link3_U.dot(a_link3)) / link3_D;
    a_link3(AZ) += qdd(jC);

    a_link4 = xt.m_link4_X_link3.link4_XM_link3() * a_link3 + c_link4;
    qdd(jD) = (link4_u - link4_U.dot(a_link4)) / link4_D;
    a_link4(LZ) += qdd(jD);

    a_link5 = xt.m_link5_X_link4.link5_XM_link4() * a_link4 + c_link5;
    qdd(jE) = (link5_u - link5_U.dot(a_link5)) / link5_D;
    a_link5(AZ) += qdd(jE);


}

