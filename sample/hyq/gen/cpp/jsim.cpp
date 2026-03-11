#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>


//Implementation of default constructor

hyq::rcg2::JSIM::JSIM(const InertiaProperties& ip, Transforms& xt) :
    ip(ip),
    xt( xt ),
    Ic_LF_lowerleg(ip.getTensor_LF_lowerleg()),
    Ic_RF_lowerleg(ip.getTensor_RF_lowerleg()),
    Ic_LH_lowerleg(ip.getTensor_LH_lowerleg()),
    Ic_RH_lowerleg(ip.getTensor_RH_lowerleg())
{
    //Initialize the matrix itself
    this->setZero();
    this->L.setZero();
    this->Linv.setZero();
}

#define DATA(r,c) this->operator()(r+6,c+6)
#define FCOL(c) this->template block<6,1>(0,c+6)


const hyq::rcg2::JSIM& hyq::rcg2::JSIM::update(const JState_t& state)
{
    using namespace iit::rbd;
    Force F;

    // Precomputes only once the coordinate transforms:
    xt.m_LF_hipassembly_X_trunk(state);
    xt.m_LF_upperleg_X_LF_hipassembly(state);
    xt.m_LF_lowerleg_X_LF_upperleg(state);
    xt.m_RF_hipassembly_X_trunk(state);
    xt.m_RF_upperleg_X_RF_hipassembly(state);
    xt.m_RF_lowerleg_X_RF_upperleg(state);
    xt.m_LH_hipassembly_X_trunk(state);
    xt.m_LH_upperleg_X_LH_hipassembly(state);
    xt.m_LH_lowerleg_X_LH_upperleg(state);
    xt.m_RH_hipassembly_X_trunk(state);
    xt.m_RH_upperleg_X_RH_hipassembly(state);
    xt.m_RH_lowerleg_X_RH_upperleg(state);

    // Initializes the composite inertia tensors
    Ic_trunk = ip.getTensor_trunk();
    Ic_LF_hipassembly = ip.getTensor_LF_hipassembly();
    Ic_LF_upperleg = ip.getTensor_LF_upperleg();
    Ic_RF_hipassembly = ip.getTensor_RF_hipassembly();
    Ic_RF_upperleg = ip.getTensor_RF_upperleg();
    Ic_LH_hipassembly = ip.getTensor_LH_hipassembly();
    Ic_LH_upperleg = ip.getTensor_LH_upperleg();
    Ic_RH_hipassembly = ip.getTensor_RH_hipassembly();
    Ic_RH_upperleg = ip.getTensor_RH_upperleg();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration
    // Link RH_lowerleg:
    transformInertia<Scalar>(Ic_RH_lowerleg, xt.m_RH_lowerleg_X_RH_upperleg.ct, Ic_spare);
    Ic_RH_upperleg += Ic_spare;

    F = Ic_RH_lowerleg.col(AZ);
    DATA(RH_KFE, RH_KFE) = F(AZ);

    F = xt.m_RH_lowerleg_X_RH_upperleg.RH_upperleg_XF_RH_lowerleg() * F;
    DATA(RH_KFE, RH_HFE) = DATA(RH_HFE, RH_KFE) = F(AZ);
    F = xt.m_RH_upperleg_X_RH_hipassembly.RH_hipassembly_XF_RH_upperleg() * F;
    DATA(RH_KFE, RH_HAA) = DATA(RH_HAA, RH_KFE) = F(AZ);
    FCOL(RH_KFE) = xt.m_RH_hipassembly_X_trunk.trunk_XF_RH_hipassembly() * F;

    // Link RH_upperleg:
    transformInertia<Scalar>(Ic_RH_upperleg, xt.m_RH_upperleg_X_RH_hipassembly.ct, Ic_spare);
    Ic_RH_hipassembly += Ic_spare;

    F = Ic_RH_upperleg.col(AZ);
    DATA(RH_HFE, RH_HFE) = F(AZ);

    F = xt.m_RH_upperleg_X_RH_hipassembly.RH_hipassembly_XF_RH_upperleg() * F;
    DATA(RH_HFE, RH_HAA) = DATA(RH_HAA, RH_HFE) = F(AZ);
    FCOL(RH_HFE) = xt.m_RH_hipassembly_X_trunk.trunk_XF_RH_hipassembly() * F;

    // Link RH_hipassembly:
    transformInertia<Scalar>(Ic_RH_hipassembly, xt.m_RH_hipassembly_X_trunk.ct, Ic_spare);
    Ic_trunk += Ic_spare;

    F = Ic_RH_hipassembly.col(AZ);
    DATA(RH_HAA, RH_HAA) = F(AZ);

    FCOL(RH_HAA) = xt.m_RH_hipassembly_X_trunk.trunk_XF_RH_hipassembly() * F;

    // Link LH_lowerleg:
    transformInertia<Scalar>(Ic_LH_lowerleg, xt.m_LH_lowerleg_X_LH_upperleg.ct, Ic_spare);
    Ic_LH_upperleg += Ic_spare;

    F = Ic_LH_lowerleg.col(AZ);
    DATA(LH_KFE, LH_KFE) = F(AZ);

    F = xt.m_LH_lowerleg_X_LH_upperleg.LH_upperleg_XF_LH_lowerleg() * F;
    DATA(LH_KFE, LH_HFE) = DATA(LH_HFE, LH_KFE) = F(AZ);
    F = xt.m_LH_upperleg_X_LH_hipassembly.LH_hipassembly_XF_LH_upperleg() * F;
    DATA(LH_KFE, LH_HAA) = DATA(LH_HAA, LH_KFE) = F(AZ);
    FCOL(LH_KFE) = xt.m_LH_hipassembly_X_trunk.trunk_XF_LH_hipassembly() * F;

    // Link LH_upperleg:
    transformInertia<Scalar>(Ic_LH_upperleg, xt.m_LH_upperleg_X_LH_hipassembly.ct, Ic_spare);
    Ic_LH_hipassembly += Ic_spare;

    F = Ic_LH_upperleg.col(AZ);
    DATA(LH_HFE, LH_HFE) = F(AZ);

    F = xt.m_LH_upperleg_X_LH_hipassembly.LH_hipassembly_XF_LH_upperleg() * F;
    DATA(LH_HFE, LH_HAA) = DATA(LH_HAA, LH_HFE) = F(AZ);
    FCOL(LH_HFE) = xt.m_LH_hipassembly_X_trunk.trunk_XF_LH_hipassembly() * F;

    // Link LH_hipassembly:
    transformInertia<Scalar>(Ic_LH_hipassembly, xt.m_LH_hipassembly_X_trunk.ct, Ic_spare);
    Ic_trunk += Ic_spare;

    F = Ic_LH_hipassembly.col(AZ);
    DATA(LH_HAA, LH_HAA) = F(AZ);

    FCOL(LH_HAA) = xt.m_LH_hipassembly_X_trunk.trunk_XF_LH_hipassembly() * F;

    // Link RF_lowerleg:
    transformInertia<Scalar>(Ic_RF_lowerleg, xt.m_RF_lowerleg_X_RF_upperleg.ct, Ic_spare);
    Ic_RF_upperleg += Ic_spare;

    F = Ic_RF_lowerleg.col(AZ);
    DATA(RF_KFE, RF_KFE) = F(AZ);

    F = xt.m_RF_lowerleg_X_RF_upperleg.RF_upperleg_XF_RF_lowerleg() * F;
    DATA(RF_KFE, RF_HFE) = DATA(RF_HFE, RF_KFE) = F(AZ);
    F = xt.m_RF_upperleg_X_RF_hipassembly.RF_hipassembly_XF_RF_upperleg() * F;
    DATA(RF_KFE, RF_HAA) = DATA(RF_HAA, RF_KFE) = F(AZ);
    FCOL(RF_KFE) = xt.m_RF_hipassembly_X_trunk.trunk_XF_RF_hipassembly() * F;

    // Link RF_upperleg:
    transformInertia<Scalar>(Ic_RF_upperleg, xt.m_RF_upperleg_X_RF_hipassembly.ct, Ic_spare);
    Ic_RF_hipassembly += Ic_spare;

    F = Ic_RF_upperleg.col(AZ);
    DATA(RF_HFE, RF_HFE) = F(AZ);

    F = xt.m_RF_upperleg_X_RF_hipassembly.RF_hipassembly_XF_RF_upperleg() * F;
    DATA(RF_HFE, RF_HAA) = DATA(RF_HAA, RF_HFE) = F(AZ);
    FCOL(RF_HFE) = xt.m_RF_hipassembly_X_trunk.trunk_XF_RF_hipassembly() * F;

    // Link RF_hipassembly:
    transformInertia<Scalar>(Ic_RF_hipassembly, xt.m_RF_hipassembly_X_trunk.ct, Ic_spare);
    Ic_trunk += Ic_spare;

    F = Ic_RF_hipassembly.col(AZ);
    DATA(RF_HAA, RF_HAA) = F(AZ);

    FCOL(RF_HAA) = xt.m_RF_hipassembly_X_trunk.trunk_XF_RF_hipassembly() * F;

    // Link LF_lowerleg:
    transformInertia<Scalar>(Ic_LF_lowerleg, xt.m_LF_lowerleg_X_LF_upperleg.ct, Ic_spare);
    Ic_LF_upperleg += Ic_spare;

    F = Ic_LF_lowerleg.col(AZ);
    DATA(LF_KFE, LF_KFE) = F(AZ);

    F = xt.m_LF_lowerleg_X_LF_upperleg.LF_upperleg_XF_LF_lowerleg() * F;
    DATA(LF_KFE, LF_HFE) = DATA(LF_HFE, LF_KFE) = F(AZ);
    F = xt.m_LF_upperleg_X_LF_hipassembly.LF_hipassembly_XF_LF_upperleg() * F;
    DATA(LF_KFE, LF_HAA) = DATA(LF_HAA, LF_KFE) = F(AZ);
    FCOL(LF_KFE) = xt.m_LF_hipassembly_X_trunk.trunk_XF_LF_hipassembly() * F;

    // Link LF_upperleg:
    transformInertia<Scalar>(Ic_LF_upperleg, xt.m_LF_upperleg_X_LF_hipassembly.ct, Ic_spare);
    Ic_LF_hipassembly += Ic_spare;

    F = Ic_LF_upperleg.col(AZ);
    DATA(LF_HFE, LF_HFE) = F(AZ);

    F = xt.m_LF_upperleg_X_LF_hipassembly.LF_hipassembly_XF_LF_upperleg() * F;
    DATA(LF_HFE, LF_HAA) = DATA(LF_HAA, LF_HFE) = F(AZ);
    FCOL(LF_HFE) = xt.m_LF_hipassembly_X_trunk.trunk_XF_LF_hipassembly() * F;

    // Link LF_hipassembly:
    transformInertia<Scalar>(Ic_LF_hipassembly, xt.m_LF_hipassembly_X_trunk.ct, Ic_spare);
    Ic_trunk += Ic_spare;

    F = Ic_LF_hipassembly.col(AZ);
    DATA(LF_HAA, LF_HAA) = F(AZ);

    FCOL(LF_HAA) = xt.m_LF_hipassembly_X_trunk.trunk_XF_LF_hipassembly() * F;


    // Copies the upper-right block into the lower-left block, after transposing
    this->template block<12, 6>(6,0) = (this->template block<6, 12>(0,6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    this->template block<6,6>(0,0) = Ic_trunk;
    return *this;
}

#undef FCOL


void hyq::rcg2::JSIM::computeL()
{
    // Joint RH_KFE, index RH_KFE
    L(RH_KFE, RH_KFE) = ScalarTraits::sqrt( DATA(RH_KFE, RH_KFE) );
    L(RH_KFE, RH_HFE) = DATA(RH_KFE, RH_HFE) / L(RH_KFE, RH_KFE);
    L(RH_KFE, RH_HAA) = DATA(RH_KFE, RH_HAA) / L(RH_KFE, RH_KFE);
    L(RH_HFE, RH_HFE) = DATA(RH_HFE, RH_HFE) - L(RH_KFE, RH_HFE) * L(RH_KFE, RH_HFE);
    L(RH_HFE, RH_HAA) = DATA(RH_HFE, RH_HAA) - L(RH_KFE, RH_HFE) * L(RH_KFE, RH_HAA);
    L(RH_HAA, RH_HAA) = DATA(RH_HAA, RH_HAA) - L(RH_KFE, RH_HAA) * L(RH_KFE, RH_HAA);

    // Joint RH_HFE, index RH_HFE
    L(RH_HFE, RH_HFE) = ScalarTraits::sqrt( L(RH_HFE, RH_HFE) );
    L(RH_HFE, RH_HAA) /= L(RH_HFE, RH_HFE);
    L(RH_HAA, RH_HAA) -= L(RH_HFE, RH_HAA) * L(RH_HFE, RH_HAA);

    // Joint RH_HAA, index RH_HAA
    L(RH_HAA, RH_HAA) = ScalarTraits::sqrt( L(RH_HAA, RH_HAA) );

    // Joint LH_KFE, index LH_KFE
    L(LH_KFE, LH_KFE) = ScalarTraits::sqrt( DATA(LH_KFE, LH_KFE) );
    L(LH_KFE, LH_HFE) = DATA(LH_KFE, LH_HFE) / L(LH_KFE, LH_KFE);
    L(LH_KFE, LH_HAA) = DATA(LH_KFE, LH_HAA) / L(LH_KFE, LH_KFE);
    L(LH_HFE, LH_HFE) = DATA(LH_HFE, LH_HFE) - L(LH_KFE, LH_HFE) * L(LH_KFE, LH_HFE);
    L(LH_HFE, LH_HAA) = DATA(LH_HFE, LH_HAA) - L(LH_KFE, LH_HFE) * L(LH_KFE, LH_HAA);
    L(LH_HAA, LH_HAA) = DATA(LH_HAA, LH_HAA) - L(LH_KFE, LH_HAA) * L(LH_KFE, LH_HAA);

    // Joint LH_HFE, index LH_HFE
    L(LH_HFE, LH_HFE) = ScalarTraits::sqrt( L(LH_HFE, LH_HFE) );
    L(LH_HFE, LH_HAA) /= L(LH_HFE, LH_HFE);
    L(LH_HAA, LH_HAA) -= L(LH_HFE, LH_HAA) * L(LH_HFE, LH_HAA);

    // Joint LH_HAA, index LH_HAA
    L(LH_HAA, LH_HAA) = ScalarTraits::sqrt( L(LH_HAA, LH_HAA) );

    // Joint RF_KFE, index RF_KFE
    L(RF_KFE, RF_KFE) = ScalarTraits::sqrt( DATA(RF_KFE, RF_KFE) );
    L(RF_KFE, RF_HFE) = DATA(RF_KFE, RF_HFE) / L(RF_KFE, RF_KFE);
    L(RF_KFE, RF_HAA) = DATA(RF_KFE, RF_HAA) / L(RF_KFE, RF_KFE);
    L(RF_HFE, RF_HFE) = DATA(RF_HFE, RF_HFE) - L(RF_KFE, RF_HFE) * L(RF_KFE, RF_HFE);
    L(RF_HFE, RF_HAA) = DATA(RF_HFE, RF_HAA) - L(RF_KFE, RF_HFE) * L(RF_KFE, RF_HAA);
    L(RF_HAA, RF_HAA) = DATA(RF_HAA, RF_HAA) - L(RF_KFE, RF_HAA) * L(RF_KFE, RF_HAA);

    // Joint RF_HFE, index RF_HFE
    L(RF_HFE, RF_HFE) = ScalarTraits::sqrt( L(RF_HFE, RF_HFE) );
    L(RF_HFE, RF_HAA) /= L(RF_HFE, RF_HFE);
    L(RF_HAA, RF_HAA) -= L(RF_HFE, RF_HAA) * L(RF_HFE, RF_HAA);

    // Joint RF_HAA, index RF_HAA
    L(RF_HAA, RF_HAA) = ScalarTraits::sqrt( L(RF_HAA, RF_HAA) );

    // Joint LF_KFE, index LF_KFE
    L(LF_KFE, LF_KFE) = ScalarTraits::sqrt( DATA(LF_KFE, LF_KFE) );
    L(LF_KFE, LF_HFE) = DATA(LF_KFE, LF_HFE) / L(LF_KFE, LF_KFE);
    L(LF_KFE, LF_HAA) = DATA(LF_KFE, LF_HAA) / L(LF_KFE, LF_KFE);
    L(LF_HFE, LF_HFE) = DATA(LF_HFE, LF_HFE) - L(LF_KFE, LF_HFE) * L(LF_KFE, LF_HFE);
    L(LF_HFE, LF_HAA) = DATA(LF_HFE, LF_HAA) - L(LF_KFE, LF_HFE) * L(LF_KFE, LF_HAA);
    L(LF_HAA, LF_HAA) = DATA(LF_HAA, LF_HAA) - L(LF_KFE, LF_HAA) * L(LF_KFE, LF_HAA);

    // Joint LF_HFE, index LF_HFE
    L(LF_HFE, LF_HFE) = ScalarTraits::sqrt( L(LF_HFE, LF_HFE) );
    L(LF_HFE, LF_HAA) /= L(LF_HFE, LF_HFE);
    L(LF_HAA, LF_HAA) -= L(LF_HFE, LF_HAA) * L(LF_HFE, LF_HAA);

    // Joint LF_HAA, index LF_HAA
    L(LF_HAA, LF_HAA) = ScalarTraits::sqrt( L(LF_HAA, LF_HAA) );

}

#undef DATA


void hyq::rcg2::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(LF_HAA, LF_HAA) = 1 / L(LF_HAA, LF_HAA);
    Linv(LF_HFE, LF_HFE) = 1 / L(LF_HFE, LF_HFE);
    Linv(LF_KFE, LF_KFE) = 1 / L(LF_KFE, LF_KFE);
    Linv(RF_HAA, RF_HAA) = 1 / L(RF_HAA, RF_HAA);
    Linv(RF_HFE, RF_HFE) = 1 / L(RF_HFE, RF_HFE);
    Linv(RF_KFE, RF_KFE) = 1 / L(RF_KFE, RF_KFE);
    Linv(LH_HAA, LH_HAA) = 1 / L(LH_HAA, LH_HAA);
    Linv(LH_HFE, LH_HFE) = 1 / L(LH_HFE, LH_HFE);
    Linv(LH_KFE, LH_KFE) = 1 / L(LH_KFE, LH_KFE);
    Linv(RH_HAA, RH_HAA) = 1 / L(RH_HAA, RH_HAA);
    Linv(RH_HFE, RH_HFE) = 1 / L(RH_HFE, RH_HFE);
    Linv(RH_KFE, RH_KFE) = 1 / L(RH_KFE, RH_KFE);
    Linv(LF_HFE, LF_HAA) = - Linv(LF_HAA, LF_HAA) * (
        ( Linv(LF_HFE, LF_HFE) * L(LF_HFE, LF_HAA) ) +
            0);
    Linv(LF_KFE, LF_HFE) = - Linv(LF_HFE, LF_HFE) * (
        ( Linv(LF_KFE, LF_KFE) * L(LF_KFE, LF_HFE) ) +
            0);
    Linv(LF_KFE, LF_HAA) = - Linv(LF_HAA, LF_HAA) * (
        ( Linv(LF_KFE, LF_KFE) * L(LF_KFE, LF_HAA) ) +
        ( Linv(LF_KFE, LF_HFE) * L(LF_HFE, LF_HAA) ) +
            0);
    Linv(RF_HFE, RF_HAA) = - Linv(RF_HAA, RF_HAA) * (
        ( Linv(RF_HFE, RF_HFE) * L(RF_HFE, RF_HAA) ) +
            0);
    Linv(RF_KFE, RF_HFE) = - Linv(RF_HFE, RF_HFE) * (
        ( Linv(RF_KFE, RF_KFE) * L(RF_KFE, RF_HFE) ) +
            0);
    Linv(RF_KFE, RF_HAA) = - Linv(RF_HAA, RF_HAA) * (
        ( Linv(RF_KFE, RF_KFE) * L(RF_KFE, RF_HAA) ) +
        ( Linv(RF_KFE, RF_HFE) * L(RF_HFE, RF_HAA) ) +
            0);
    Linv(LH_HFE, LH_HAA) = - Linv(LH_HAA, LH_HAA) * (
        ( Linv(LH_HFE, LH_HFE) * L(LH_HFE, LH_HAA) ) +
            0);
    Linv(LH_KFE, LH_HFE) = - Linv(LH_HFE, LH_HFE) * (
        ( Linv(LH_KFE, LH_KFE) * L(LH_KFE, LH_HFE) ) +
            0);
    Linv(LH_KFE, LH_HAA) = - Linv(LH_HAA, LH_HAA) * (
        ( Linv(LH_KFE, LH_KFE) * L(LH_KFE, LH_HAA) ) +
        ( Linv(LH_KFE, LH_HFE) * L(LH_HFE, LH_HAA) ) +
            0);
    Linv(RH_HFE, RH_HAA) = - Linv(RH_HAA, RH_HAA) * (
        ( Linv(RH_HFE, RH_HFE) * L(RH_HFE, RH_HAA) ) +
            0);
    Linv(RH_KFE, RH_HFE) = - Linv(RH_HFE, RH_HFE) * (
        ( Linv(RH_KFE, RH_KFE) * L(RH_KFE, RH_HFE) ) +
            0);
    Linv(RH_KFE, RH_HAA) = - Linv(RH_HAA, RH_HAA) * (
        ( Linv(RH_KFE, RH_KFE) * L(RH_KFE, RH_HAA) ) +
        ( Linv(RH_KFE, RH_HFE) * L(RH_HFE, RH_HAA) ) +
            0);
}


void hyq::rcg2::JSIM::computeInverse()
{
    computeLInverse();

    inverse(0, 0) =
        + ( Linv(0,LF_HAA) * Linv(0,LF_HAA) )
        ;
    inverse(1, 1) =
        + ( Linv(1,LF_HFE) * Linv(1,LF_HFE) )
        + ( Linv(1,LF_HAA) * Linv(1,LF_HAA) )
        ;
    inverse(1, 0) =
        + ( Linv(1,LF_HAA) * Linv(0,LF_HAA) )
        ;
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =
        + ( Linv(2,LF_KFE) * Linv(2,LF_KFE) )
        + ( Linv(2,LF_HFE) * Linv(2,LF_HFE) )
        + ( Linv(2,LF_HAA) * Linv(2,LF_HAA) )
        ;
    inverse(2, 1) =
        + ( Linv(2,LF_HFE) * Linv(1,LF_HFE) )
        + ( Linv(2,LF_HAA) * Linv(1,LF_HAA) )
        ;
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) =
        + ( Linv(2,LF_HAA) * Linv(0,LF_HAA) )
        ;
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) =
        + ( Linv(3,RF_HAA) * Linv(3,RF_HAA) )
        ;
    inverse(3, 2) =
        + ( Linv(3,LF_KFE) * Linv(2,LF_KFE) )
        + ( Linv(3,LF_HFE) * Linv(2,LF_HFE) )
        + ( Linv(3,LF_HAA) * Linv(2,LF_HAA) )
        ;
    inverse(2, 3) = inverse(3, 2);
    inverse(3, 1) =
        + ( Linv(3,LF_HFE) * Linv(1,LF_HFE) )
        + ( Linv(3,LF_HAA) * Linv(1,LF_HAA) )
        ;
    inverse(1, 3) = inverse(3, 1);
    inverse(3, 0) =
        + ( Linv(3,LF_HAA) * Linv(0,LF_HAA) )
        ;
    inverse(0, 3) = inverse(3, 0);
    inverse(4, 4) =
        + ( Linv(4,RF_HFE) * Linv(4,RF_HFE) )
        + ( Linv(4,RF_HAA) * Linv(4,RF_HAA) )
        ;
    inverse(4, 3) =
        + ( Linv(4,RF_HAA) * Linv(3,RF_HAA) )
        ;
    inverse(3, 4) = inverse(4, 3);
    inverse(4, 2) =
        + ( Linv(4,LF_KFE) * Linv(2,LF_KFE) )
        + ( Linv(4,LF_HFE) * Linv(2,LF_HFE) )
        + ( Linv(4,LF_HAA) * Linv(2,LF_HAA) )
        ;
    inverse(2, 4) = inverse(4, 2);
    inverse(4, 1) =
        + ( Linv(4,LF_HFE) * Linv(1,LF_HFE) )
        + ( Linv(4,LF_HAA) * Linv(1,LF_HAA) )
        ;
    inverse(1, 4) = inverse(4, 1);
    inverse(4, 0) =
        + ( Linv(4,LF_HAA) * Linv(0,LF_HAA) )
        ;
    inverse(0, 4) = inverse(4, 0);
    inverse(5, 5) =
        + ( Linv(5,RF_KFE) * Linv(5,RF_KFE) )
        + ( Linv(5,RF_HFE) * Linv(5,RF_HFE) )
        + ( Linv(5,RF_HAA) * Linv(5,RF_HAA) )
        ;
    inverse(5, 4) =
        + ( Linv(5,RF_HFE) * Linv(4,RF_HFE) )
        + ( Linv(5,RF_HAA) * Linv(4,RF_HAA) )
        ;
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =
        + ( Linv(5,RF_HAA) * Linv(3,RF_HAA) )
        ;
    inverse(3, 5) = inverse(5, 3);
    inverse(5, 2) =
        + ( Linv(5,LF_KFE) * Linv(2,LF_KFE) )
        + ( Linv(5,LF_HFE) * Linv(2,LF_HFE) )
        + ( Linv(5,LF_HAA) * Linv(2,LF_HAA) )
        ;
    inverse(2, 5) = inverse(5, 2);
    inverse(5, 1) =
        + ( Linv(5,LF_HFE) * Linv(1,LF_HFE) )
        + ( Linv(5,LF_HAA) * Linv(1,LF_HAA) )
        ;
    inverse(1, 5) = inverse(5, 1);
    inverse(5, 0) =
        + ( Linv(5,LF_HAA) * Linv(0,LF_HAA) )
        ;
    inverse(0, 5) = inverse(5, 0);
    inverse(6, 6) =
        + ( Linv(6,LH_HAA) * Linv(6,LH_HAA) )
        ;
    inverse(6, 5) =
        + ( Linv(6,RF_KFE) * Linv(5,RF_KFE) )
        + ( Linv(6,RF_HFE) * Linv(5,RF_HFE) )
        + ( Linv(6,RF_HAA) * Linv(5,RF_HAA) )
        ;
    inverse(5, 6) = inverse(6, 5);
    inverse(6, 4) =
        + ( Linv(6,RF_HFE) * Linv(4,RF_HFE) )
        + ( Linv(6,RF_HAA) * Linv(4,RF_HAA) )
        ;
    inverse(4, 6) = inverse(6, 4);
    inverse(6, 3) =
        + ( Linv(6,RF_HAA) * Linv(3,RF_HAA) )
        ;
    inverse(3, 6) = inverse(6, 3);
    inverse(6, 2) =
        + ( Linv(6,LF_KFE) * Linv(2,LF_KFE) )
        + ( Linv(6,LF_HFE) * Linv(2,LF_HFE) )
        + ( Linv(6,LF_HAA) * Linv(2,LF_HAA) )
        ;
    inverse(2, 6) = inverse(6, 2);
    inverse(6, 1) =
        + ( Linv(6,LF_HFE) * Linv(1,LF_HFE) )
        + ( Linv(6,LF_HAA) * Linv(1,LF_HAA) )
        ;
    inverse(1, 6) = inverse(6, 1);
    inverse(6, 0) =
        + ( Linv(6,LF_HAA) * Linv(0,LF_HAA) )
        ;
    inverse(0, 6) = inverse(6, 0);
    inverse(7, 7) =
        + ( Linv(7,LH_HFE) * Linv(7,LH_HFE) )
        + ( Linv(7,LH_HAA) * Linv(7,LH_HAA) )
        ;
    inverse(7, 6) =
        + ( Linv(7,LH_HAA) * Linv(6,LH_HAA) )
        ;
    inverse(6, 7) = inverse(7, 6);
    inverse(7, 5) =
        + ( Linv(7,RF_KFE) * Linv(5,RF_KFE) )
        + ( Linv(7,RF_HFE) * Linv(5,RF_HFE) )
        + ( Linv(7,RF_HAA) * Linv(5,RF_HAA) )
        ;
    inverse(5, 7) = inverse(7, 5);
    inverse(7, 4) =
        + ( Linv(7,RF_HFE) * Linv(4,RF_HFE) )
        + ( Linv(7,RF_HAA) * Linv(4,RF_HAA) )
        ;
    inverse(4, 7) = inverse(7, 4);
    inverse(7, 3) =
        + ( Linv(7,RF_HAA) * Linv(3,RF_HAA) )
        ;
    inverse(3, 7) = inverse(7, 3);
    inverse(7, 2) =
        + ( Linv(7,LF_KFE) * Linv(2,LF_KFE) )
        + ( Linv(7,LF_HFE) * Linv(2,LF_HFE) )
        + ( Linv(7,LF_HAA) * Linv(2,LF_HAA) )
        ;
    inverse(2, 7) = inverse(7, 2);
    inverse(7, 1) =
        + ( Linv(7,LF_HFE) * Linv(1,LF_HFE) )
        + ( Linv(7,LF_HAA) * Linv(1,LF_HAA) )
        ;
    inverse(1, 7) = inverse(7, 1);
    inverse(7, 0) =
        + ( Linv(7,LF_HAA) * Linv(0,LF_HAA) )
        ;
    inverse(0, 7) = inverse(7, 0);
    inverse(8, 8) =
        + ( Linv(8,LH_KFE) * Linv(8,LH_KFE) )
        + ( Linv(8,LH_HFE) * Linv(8,LH_HFE) )
        + ( Linv(8,LH_HAA) * Linv(8,LH_HAA) )
        ;
    inverse(8, 7) =
        + ( Linv(8,LH_HFE) * Linv(7,LH_HFE) )
        + ( Linv(8,LH_HAA) * Linv(7,LH_HAA) )
        ;
    inverse(7, 8) = inverse(8, 7);
    inverse(8, 6) =
        + ( Linv(8,LH_HAA) * Linv(6,LH_HAA) )
        ;
    inverse(6, 8) = inverse(8, 6);
    inverse(8, 5) =
        + ( Linv(8,RF_KFE) * Linv(5,RF_KFE) )
        + ( Linv(8,RF_HFE) * Linv(5,RF_HFE) )
        + ( Linv(8,RF_HAA) * Linv(5,RF_HAA) )
        ;
    inverse(5, 8) = inverse(8, 5);
    inverse(8, 4) =
        + ( Linv(8,RF_HFE) * Linv(4,RF_HFE) )
        + ( Linv(8,RF_HAA) * Linv(4,RF_HAA) )
        ;
    inverse(4, 8) = inverse(8, 4);
    inverse(8, 3) =
        + ( Linv(8,RF_HAA) * Linv(3,RF_HAA) )
        ;
    inverse(3, 8) = inverse(8, 3);
    inverse(8, 2) =
        + ( Linv(8,LF_KFE) * Linv(2,LF_KFE) )
        + ( Linv(8,LF_HFE) * Linv(2,LF_HFE) )
        + ( Linv(8,LF_HAA) * Linv(2,LF_HAA) )
        ;
    inverse(2, 8) = inverse(8, 2);
    inverse(8, 1) =
        + ( Linv(8,LF_HFE) * Linv(1,LF_HFE) )
        + ( Linv(8,LF_HAA) * Linv(1,LF_HAA) )
        ;
    inverse(1, 8) = inverse(8, 1);
    inverse(8, 0) =
        + ( Linv(8,LF_HAA) * Linv(0,LF_HAA) )
        ;
    inverse(0, 8) = inverse(8, 0);
    inverse(9, 9) =
        + ( Linv(9,RH_HAA) * Linv(9,RH_HAA) )
        ;
    inverse(9, 8) =
        + ( Linv(9,LH_KFE) * Linv(8,LH_KFE) )
        + ( Linv(9,LH_HFE) * Linv(8,LH_HFE) )
        + ( Linv(9,LH_HAA) * Linv(8,LH_HAA) )
        ;
    inverse(8, 9) = inverse(9, 8);
    inverse(9, 7) =
        + ( Linv(9,LH_HFE) * Linv(7,LH_HFE) )
        + ( Linv(9,LH_HAA) * Linv(7,LH_HAA) )
        ;
    inverse(7, 9) = inverse(9, 7);
    inverse(9, 6) =
        + ( Linv(9,LH_HAA) * Linv(6,LH_HAA) )
        ;
    inverse(6, 9) = inverse(9, 6);
    inverse(9, 5) =
        + ( Linv(9,RF_KFE) * Linv(5,RF_KFE) )
        + ( Linv(9,RF_HFE) * Linv(5,RF_HFE) )
        + ( Linv(9,RF_HAA) * Linv(5,RF_HAA) )
        ;
    inverse(5, 9) = inverse(9, 5);
    inverse(9, 4) =
        + ( Linv(9,RF_HFE) * Linv(4,RF_HFE) )
        + ( Linv(9,RF_HAA) * Linv(4,RF_HAA) )
        ;
    inverse(4, 9) = inverse(9, 4);
    inverse(9, 3) =
        + ( Linv(9,RF_HAA) * Linv(3,RF_HAA) )
        ;
    inverse(3, 9) = inverse(9, 3);
    inverse(9, 2) =
        + ( Linv(9,LF_KFE) * Linv(2,LF_KFE) )
        + ( Linv(9,LF_HFE) * Linv(2,LF_HFE) )
        + ( Linv(9,LF_HAA) * Linv(2,LF_HAA) )
        ;
    inverse(2, 9) = inverse(9, 2);
    inverse(9, 1) =
        + ( Linv(9,LF_HFE) * Linv(1,LF_HFE) )
        + ( Linv(9,LF_HAA) * Linv(1,LF_HAA) )
        ;
    inverse(1, 9) = inverse(9, 1);
    inverse(9, 0) =
        + ( Linv(9,LF_HAA) * Linv(0,LF_HAA) )
        ;
    inverse(0, 9) = inverse(9, 0);
    inverse(10, 10) =
        + ( Linv(10,RH_HFE) * Linv(10,RH_HFE) )
        + ( Linv(10,RH_HAA) * Linv(10,RH_HAA) )
        ;
    inverse(10, 9) =
        + ( Linv(10,RH_HAA) * Linv(9,RH_HAA) )
        ;
    inverse(9, 10) = inverse(10, 9);
    inverse(10, 8) =
        + ( Linv(10,LH_KFE) * Linv(8,LH_KFE) )
        + ( Linv(10,LH_HFE) * Linv(8,LH_HFE) )
        + ( Linv(10,LH_HAA) * Linv(8,LH_HAA) )
        ;
    inverse(8, 10) = inverse(10, 8);
    inverse(10, 7) =
        + ( Linv(10,LH_HFE) * Linv(7,LH_HFE) )
        + ( Linv(10,LH_HAA) * Linv(7,LH_HAA) )
        ;
    inverse(7, 10) = inverse(10, 7);
    inverse(10, 6) =
        + ( Linv(10,LH_HAA) * Linv(6,LH_HAA) )
        ;
    inverse(6, 10) = inverse(10, 6);
    inverse(10, 5) =
        + ( Linv(10,RF_KFE) * Linv(5,RF_KFE) )
        + ( Linv(10,RF_HFE) * Linv(5,RF_HFE) )
        + ( Linv(10,RF_HAA) * Linv(5,RF_HAA) )
        ;
    inverse(5, 10) = inverse(10, 5);
    inverse(10, 4) =
        + ( Linv(10,RF_HFE) * Linv(4,RF_HFE) )
        + ( Linv(10,RF_HAA) * Linv(4,RF_HAA) )
        ;
    inverse(4, 10) = inverse(10, 4);
    inverse(10, 3) =
        + ( Linv(10,RF_HAA) * Linv(3,RF_HAA) )
        ;
    inverse(3, 10) = inverse(10, 3);
    inverse(10, 2) =
        + ( Linv(10,LF_KFE) * Linv(2,LF_KFE) )
        + ( Linv(10,LF_HFE) * Linv(2,LF_HFE) )
        + ( Linv(10,LF_HAA) * Linv(2,LF_HAA) )
        ;
    inverse(2, 10) = inverse(10, 2);
    inverse(10, 1) =
        + ( Linv(10,LF_HFE) * Linv(1,LF_HFE) )
        + ( Linv(10,LF_HAA) * Linv(1,LF_HAA) )
        ;
    inverse(1, 10) = inverse(10, 1);
    inverse(10, 0) =
        + ( Linv(10,LF_HAA) * Linv(0,LF_HAA) )
        ;
    inverse(0, 10) = inverse(10, 0);
    inverse(11, 11) =
        + ( Linv(11,RH_KFE) * Linv(11,RH_KFE) )
        + ( Linv(11,RH_HFE) * Linv(11,RH_HFE) )
        + ( Linv(11,RH_HAA) * Linv(11,RH_HAA) )
        ;
    inverse(11, 10) =
        + ( Linv(11,RH_HFE) * Linv(10,RH_HFE) )
        + ( Linv(11,RH_HAA) * Linv(10,RH_HAA) )
        ;
    inverse(10, 11) = inverse(11, 10);
    inverse(11, 9) =
        + ( Linv(11,RH_HAA) * Linv(9,RH_HAA) )
        ;
    inverse(9, 11) = inverse(11, 9);
    inverse(11, 8) =
        + ( Linv(11,LH_KFE) * Linv(8,LH_KFE) )
        + ( Linv(11,LH_HFE) * Linv(8,LH_HFE) )
        + ( Linv(11,LH_HAA) * Linv(8,LH_HAA) )
        ;
    inverse(8, 11) = inverse(11, 8);
    inverse(11, 7) =
        + ( Linv(11,LH_HFE) * Linv(7,LH_HFE) )
        + ( Linv(11,LH_HAA) * Linv(7,LH_HAA) )
        ;
    inverse(7, 11) = inverse(11, 7);
    inverse(11, 6) =
        + ( Linv(11,LH_HAA) * Linv(6,LH_HAA) )
        ;
    inverse(6, 11) = inverse(11, 6);
    inverse(11, 5) =
        + ( Linv(11,RF_KFE) * Linv(5,RF_KFE) )
        + ( Linv(11,RF_HFE) * Linv(5,RF_HFE) )
        + ( Linv(11,RF_HAA) * Linv(5,RF_HAA) )
        ;
    inverse(5, 11) = inverse(11, 5);
    inverse(11, 4) =
        + ( Linv(11,RF_HFE) * Linv(4,RF_HFE) )
        + ( Linv(11,RF_HAA) * Linv(4,RF_HAA) )
        ;
    inverse(4, 11) = inverse(11, 4);
    inverse(11, 3) =
        + ( Linv(11,RF_HAA) * Linv(3,RF_HAA) )
        ;
    inverse(3, 11) = inverse(11, 3);
    inverse(11, 2) =
        + ( Linv(11,LF_KFE) * Linv(2,LF_KFE) )
        + ( Linv(11,LF_HFE) * Linv(2,LF_HFE) )
        + ( Linv(11,LF_HAA) * Linv(2,LF_HAA) )
        ;
    inverse(2, 11) = inverse(11, 2);
    inverse(11, 1) =
        + ( Linv(11,LF_HFE) * Linv(1,LF_HFE) )
        + ( Linv(11,LF_HAA) * Linv(1,LF_HAA) )
        ;
    inverse(1, 11) = inverse(11, 1);
    inverse(11, 0) =
        + ( Linv(11,LF_HAA) * Linv(0,LF_HAA) )
        ;
    inverse(0, 11) = inverse(11, 0);
}
