#ifndef RCG2_FANCY_JSIM_H
#define RCG2_FANCY_JSIM_H

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentMatrix.h>

#include "declarations.h"
#include "rbd_types.h"
#include "inertia_properties.h"
#include "transforms.h"

namespace fancy {
namespace rcg2 {

/**
 * The type of the Joint Space Inertia Matrix (JSIM) of the robot Fancy.
 */

struct JSIM : public iit::rbd::StateDependentMatrix<JointState, 5, 5, JSIM >
{
    using JState_t = JointState;
    using Base = iit::rbd::StateDependentMatrix<JointState, 5, 5, JSIM >;
    using Index  = typename Base::Index;
    using Matrix_t = typename Base::MatrixType ;


    JSIM(const InertiaProperties&, Transforms&);
    ~JSIM() {}

    const JSIM& update(const JState_t&);

    /**
     * Computes the matrix L of the L^T L factorization of this JSIM.
     */
    void computeL();

    /**
     * Computes the inverse of the matrix L.
     * Assumes computeL() was called before.
     */
    void computeLInverse();

    /**
     * Computes and stores the inverse of this JSIM.
     * This function does call computeLInverse() first.
     * The algorithm takes advantage of the branch
     * induced sparsity of the robot, if any.
     */
    void computeInverse();

    /**
     * Returns an unmodifiable reference to the matrix L.
     * See also computeL()
     */
    const Matrix_t& getL() const { return L; }

    /**
     * Returns an unmodifiable reference to the inverse of L.
     * See also computeLInverse()
     */
    const Matrix_t& getLInverse() const { return Linv; }

    /**
     * Returns an unmodifiable reference to the last computed inverse of this JSIM
     */
    const Matrix_t& getInverse() const { return inverse; }


    /**
     * Computes L^{-T} times x using only L, exploiting sparsity.
     * Assumes L is already updated.
     */
    template<typename Derived>
    void LinvT_times_x(const iit::rbd::MatrixBase<Derived>& x_cnt);
    /**
     * Computes L^{-1} times x using only L, exploiting sparsity.
     * Assumes L is already updated.
     */
    template<typename Derived>
    void Linv_times_x(const iit::rbd::MatrixBase<Derived>& x_cnt);


    const InertiaProperties& ip;
    Transforms& xt;

    // The composite-inertia tensor for each link
    InertiaMatrix Ic_link1;
    InertiaMatrix Ic_link2;
    InertiaMatrix Ic_link3;
    InertiaMatrix Ic_link4;
    const InertiaMatrix& Ic_link5;
    InertiaMatrix Ic_spare;

    Matrix_t L;
    Matrix_t Linv;
    Matrix_t inverse;
};


template<typename Derived>
void JSIM::LinvT_times_x(const iit::rbd::MatrixBase<Derived>& x_cnt)
{
    auto& x = const_cast<iit::rbd::MatrixBase<Derived>& >(x_cnt);
    //assumes L has been computed already
    x(jE) /= L(jE, jE);
    x(jD) -= L(jE, jD) * x(jE);
    x(jC) -= L(jE, jC) * x(jE);
    x(jB) -= L(jE, jB) * x(jE);
    x(jA) -= L(jE, jA) * x(jE);
    x(jD) /= L(jD, jD);
    x(jC) -= L(jD, jC) * x(jD);
    x(jB) -= L(jD, jB) * x(jD);
    x(jA) -= L(jD, jA) * x(jD);
    x(jC) /= L(jC, jC);
    x(jB) -= L(jC, jB) * x(jC);
    x(jA) -= L(jC, jA) * x(jC);
    x(jB) /= L(jB, jB);
    x(jA) -= L(jB, jA) * x(jB);
    x(jA) /= L(jA, jA);
}


template<typename Derived>
void JSIM::Linv_times_x(const iit::rbd::MatrixBase<Derived>& x_cnt)
{
    auto& x = const_cast<iit::rbd::MatrixBase<Derived>& >(x_cnt);
    //assumes L has been computed already
    x(jA) /= L(jA, jA);
    x(jB) -= L(jB, jA) * x(jA);
    x(jB) /= L(jB, jB);
    x(jC) -= L(jC, jB) * x(jB);
    x(jC) -= L(jC, jA) * x(jA);
    x(jC) /= L(jC, jC);
    x(jD) -= L(jD, jC) * x(jC);
    x(jD) -= L(jD, jB) * x(jB);
    x(jD) -= L(jD, jA) * x(jA);
    x(jD) /= L(jD, jD);
    x(jE) -= L(jE, jD) * x(jD);
    x(jE) -= L(jE, jC) * x(jC);
    x(jE) -= L(jE, jB) * x(jB);
    x(jE) -= L(jE, jA) * x(jA);
    x(jE) /= L(jE, jE);
}


}
}


#endif
