#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>


//Implementation of default constructor

fancy::rcg2::JSIM::JSIM(const InertiaProperties& ip, Transforms& xt) :
    ip(ip),
    xt( xt ),
    Ic_link5(ip.getTensor_link5())
{
    //Initialize the matrix itself
    this->setZero();
    this->L.setZero();
    this->Linv.setZero();
}

#define DATA this->operator()


const fancy::rcg2::JSIM& fancy::rcg2::JSIM::update(const JState_t& state)
{
    using namespace iit::rbd;
    Force F;

    // Precomputes only once the coordinate transforms:
    xt.m_link2_X_link1(state);
    xt.m_link3_X_link2(state);
    xt.m_link4_X_link3(state);
    xt.m_link5_X_link4(state);

    // Initializes the composite inertia tensors
    Ic_link1 = ip.getTensor_link1();
    Ic_link2 = ip.getTensor_link2();
    Ic_link3 = ip.getTensor_link3();
    Ic_link4 = ip.getTensor_link4();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration
    // Link link5:
    transformInertia<Scalar>(Ic_link5, xt.m_link5_X_link4.ct, Ic_spare);
    Ic_link4 += Ic_spare;

    F = Ic_link5.col(AZ);
    DATA(jE, jE) = F(AZ);

    F = xt.m_link5_X_link4.link4_XF_link5() * F;
    DATA(jE, jD) = DATA(jD, jE) = F(LZ);
    F = xt.m_link4_X_link3.link3_XF_link4() * F;
    DATA(jE, jC) = DATA(jC, jE) = F(AZ);
    F = xt.m_link3_X_link2.link2_XF_link3() * F;
    DATA(jE, jB) = DATA(jB, jE) = F(LZ);
    F = xt.m_link2_X_link1.link1_XF_link2() * F;
    DATA(jE, jA) = DATA(jA, jE) = F(AZ);

    // Link link4:
    transformInertia<Scalar>(Ic_link4, xt.m_link4_X_link3.ct, Ic_spare);
    Ic_link3 += Ic_spare;

    F = Ic_link4.col(LZ);
    DATA(jD, jD) = F(LZ);

    F = xt.m_link4_X_link3.link3_XF_link4() * F;
    DATA(jD, jC) = DATA(jC, jD) = F(AZ);
    F = xt.m_link3_X_link2.link2_XF_link3() * F;
    DATA(jD, jB) = DATA(jB, jD) = F(LZ);
    F = xt.m_link2_X_link1.link1_XF_link2() * F;
    DATA(jD, jA) = DATA(jA, jD) = F(AZ);

    // Link link3:
    transformInertia<Scalar>(Ic_link3, xt.m_link3_X_link2.ct, Ic_spare);
    Ic_link2 += Ic_spare;

    F = Ic_link3.col(AZ);
    DATA(jC, jC) = F(AZ);

    F = xt.m_link3_X_link2.link2_XF_link3() * F;
    DATA(jC, jB) = DATA(jB, jC) = F(LZ);
    F = xt.m_link2_X_link1.link1_XF_link2() * F;
    DATA(jC, jA) = DATA(jA, jC) = F(AZ);

    // Link link2:
    transformInertia<Scalar>(Ic_link2, xt.m_link2_X_link1.ct, Ic_spare);
    Ic_link1 += Ic_spare;

    F = Ic_link2.col(LZ);
    DATA(jB, jB) = F(LZ);

    F = xt.m_link2_X_link1.link1_XF_link2() * F;
    DATA(jB, jA) = DATA(jA, jB) = F(AZ);

    // Link link1:

    F = Ic_link1.col(AZ);
    DATA(jA, jA) = F(AZ);



    return *this;
}



void fancy::rcg2::JSIM::computeL()
{
    // Joint jE, index jE
    L(jE, jE) = ScalarTraits::sqrt( DATA(jE, jE) );
    L(jE, jD) = DATA(jE, jD) / L(jE, jE);
    L(jE, jC) = DATA(jE, jC) / L(jE, jE);
    L(jE, jB) = DATA(jE, jB) / L(jE, jE);
    L(jE, jA) = DATA(jE, jA) / L(jE, jE);
    L(jD, jD) = DATA(jD, jD) - L(jE, jD) * L(jE, jD);
    L(jD, jC) = DATA(jD, jC) - L(jE, jD) * L(jE, jC);
    L(jD, jB) = DATA(jD, jB) - L(jE, jD) * L(jE, jB);
    L(jD, jA) = DATA(jD, jA) - L(jE, jD) * L(jE, jA);
    L(jC, jC) = DATA(jC, jC) - L(jE, jC) * L(jE, jC);
    L(jC, jB) = DATA(jC, jB) - L(jE, jC) * L(jE, jB);
    L(jC, jA) = DATA(jC, jA) - L(jE, jC) * L(jE, jA);
    L(jB, jB) = DATA(jB, jB) - L(jE, jB) * L(jE, jB);
    L(jB, jA) = DATA(jB, jA) - L(jE, jB) * L(jE, jA);
    L(jA, jA) = DATA(jA, jA) - L(jE, jA) * L(jE, jA);

    // Joint jD, index jD
    L(jD, jD) = ScalarTraits::sqrt( L(jD, jD) );
    L(jD, jC) /= L(jD, jD);
    L(jD, jB) /= L(jD, jD);
    L(jD, jA) /= L(jD, jD);
    L(jC, jC) -= L(jD, jC) * L(jD, jC);
    L(jC, jB) -= L(jD, jC) * L(jD, jB);
    L(jC, jA) -= L(jD, jC) * L(jD, jA);
    L(jB, jB) -= L(jD, jB) * L(jD, jB);
    L(jB, jA) -= L(jD, jB) * L(jD, jA);
    L(jA, jA) -= L(jD, jA) * L(jD, jA);

    // Joint jC, index jC
    L(jC, jC) = ScalarTraits::sqrt( L(jC, jC) );
    L(jC, jB) /= L(jC, jC);
    L(jC, jA) /= L(jC, jC);
    L(jB, jB) -= L(jC, jB) * L(jC, jB);
    L(jB, jA) -= L(jC, jB) * L(jC, jA);
    L(jA, jA) -= L(jC, jA) * L(jC, jA);

    // Joint jB, index jB
    L(jB, jB) = ScalarTraits::sqrt( L(jB, jB) );
    L(jB, jA) /= L(jB, jB);
    L(jA, jA) -= L(jB, jA) * L(jB, jA);

    // Joint jA, index jA
    L(jA, jA) = ScalarTraits::sqrt( L(jA, jA) );

}

#undef DATA


void fancy::rcg2::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(jA, jA) = 1 / L(jA, jA);
    Linv(jB, jB) = 1 / L(jB, jB);
    Linv(jC, jC) = 1 / L(jC, jC);
    Linv(jD, jD) = 1 / L(jD, jD);
    Linv(jE, jE) = 1 / L(jE, jE);
    Linv(jB, jA) = - Linv(jA, jA) * (
        ( Linv(jB, jB) * L(jB, jA) ) +
            0);
    Linv(jC, jB) = - Linv(jB, jB) * (
        ( Linv(jC, jC) * L(jC, jB) ) +
            0);
    Linv(jC, jA) = - Linv(jA, jA) * (
        ( Linv(jC, jC) * L(jC, jA) ) +
        ( Linv(jC, jB) * L(jB, jA) ) +
            0);
    Linv(jD, jC) = - Linv(jC, jC) * (
        ( Linv(jD, jD) * L(jD, jC) ) +
            0);
    Linv(jD, jB) = - Linv(jB, jB) * (
        ( Linv(jD, jD) * L(jD, jB) ) +
        ( Linv(jD, jC) * L(jC, jB) ) +
            0);
    Linv(jD, jA) = - Linv(jA, jA) * (
        ( Linv(jD, jD) * L(jD, jA) ) +
        ( Linv(jD, jC) * L(jC, jA) ) +
        ( Linv(jD, jB) * L(jB, jA) ) +
            0);
    Linv(jE, jD) = - Linv(jD, jD) * (
        ( Linv(jE, jE) * L(jE, jD) ) +
            0);
    Linv(jE, jC) = - Linv(jC, jC) * (
        ( Linv(jE, jE) * L(jE, jC) ) +
        ( Linv(jE, jD) * L(jD, jC) ) +
            0);
    Linv(jE, jB) = - Linv(jB, jB) * (
        ( Linv(jE, jE) * L(jE, jB) ) +
        ( Linv(jE, jD) * L(jD, jB) ) +
        ( Linv(jE, jC) * L(jC, jB) ) +
            0);
    Linv(jE, jA) = - Linv(jA, jA) * (
        ( Linv(jE, jE) * L(jE, jA) ) +
        ( Linv(jE, jD) * L(jD, jA) ) +
        ( Linv(jE, jC) * L(jC, jA) ) +
        ( Linv(jE, jB) * L(jB, jA) ) +
            0);
}


void fancy::rcg2::JSIM::computeInverse()
{
    computeLInverse();

    inverse(0, 0) =
        + ( Linv(0,jA) * Linv(0,jA) )
        ;
    inverse(1, 1) =
        + ( Linv(1,jB) * Linv(1,jB) )
        + ( Linv(1,jA) * Linv(1,jA) )
        ;
    inverse(1, 0) =
        + ( Linv(1,jA) * Linv(0,jA) )
        ;
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =
        + ( Linv(2,jC) * Linv(2,jC) )
        + ( Linv(2,jB) * Linv(2,jB) )
        + ( Linv(2,jA) * Linv(2,jA) )
        ;
    inverse(2, 1) =
        + ( Linv(2,jB) * Linv(1,jB) )
        + ( Linv(2,jA) * Linv(1,jA) )
        ;
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) =
        + ( Linv(2,jA) * Linv(0,jA) )
        ;
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) =
        + ( Linv(3,jD) * Linv(3,jD) )
        + ( Linv(3,jC) * Linv(3,jC) )
        + ( Linv(3,jB) * Linv(3,jB) )
        + ( Linv(3,jA) * Linv(3,jA) )
        ;
    inverse(3, 2) =
        + ( Linv(3,jC) * Linv(2,jC) )
        + ( Linv(3,jB) * Linv(2,jB) )
        + ( Linv(3,jA) * Linv(2,jA) )
        ;
    inverse(2, 3) = inverse(3, 2);
    inverse(3, 1) =
        + ( Linv(3,jB) * Linv(1,jB) )
        + ( Linv(3,jA) * Linv(1,jA) )
        ;
    inverse(1, 3) = inverse(3, 1);
    inverse(3, 0) =
        + ( Linv(3,jA) * Linv(0,jA) )
        ;
    inverse(0, 3) = inverse(3, 0);
    inverse(4, 4) =
        + ( Linv(4,jE) * Linv(4,jE) )
        + ( Linv(4,jD) * Linv(4,jD) )
        + ( Linv(4,jC) * Linv(4,jC) )
        + ( Linv(4,jB) * Linv(4,jB) )
        + ( Linv(4,jA) * Linv(4,jA) )
        ;
    inverse(4, 3) =
        + ( Linv(4,jD) * Linv(3,jD) )
        + ( Linv(4,jC) * Linv(3,jC) )
        + ( Linv(4,jB) * Linv(3,jB) )
        + ( Linv(4,jA) * Linv(3,jA) )
        ;
    inverse(3, 4) = inverse(4, 3);
    inverse(4, 2) =
        + ( Linv(4,jC) * Linv(2,jC) )
        + ( Linv(4,jB) * Linv(2,jB) )
        + ( Linv(4,jA) * Linv(2,jA) )
        ;
    inverse(2, 4) = inverse(4, 2);
    inverse(4, 1) =
        + ( Linv(4,jB) * Linv(1,jB) )
        + ( Linv(4,jA) * Linv(1,jA) )
        ;
    inverse(1, 4) = inverse(4, 1);
    inverse(4, 0) =
        + ( Linv(4,jA) * Linv(0,jA) )
        ;
    inverse(0, 4) = inverse(4, 0);
}
