#ifndef RCG2_HYQ_JSIM_H
#define RCG2_HYQ_JSIM_H

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentMatrix.h>

#include "declarations.h"
#include "rbd_types.h"
#include "inertia_properties.h"
#include "transforms.h"

namespace hyq {
namespace rcg2 {

/**
 * The type of the Joint Space Inertia Matrix (JSIM) of the robot HyQ.
 */

struct JSIM : public iit::rbd::StateDependentMatrix<JointState, 18, 18, JSIM >
{
    using JState_t = JointState;
    using Base = iit::rbd::StateDependentMatrix<JointState, 18, 18, JSIM >;
    using Index  = typename Base::Index;
    using Matrix_t = typename Base::MatrixType ;

    /** The block-type of the F sub-block of this JSIM */
    using FBlock_t = const iit::rbd::MatrixBlock<const Matrix_t,6,12> ;
    /** The block-type of the actuated-joints sub-block of this JSIM */
    using RealJointsBlock_t = const iit::rbd::MatrixBlock<const Matrix_t,12,12>;
    /** The matrix-type of the actuated-joints sub-block */
    using RealJointsBlock_matrix_t = iit::rbd::PlainMatrix<Scalar, 12, 12>;

    // For backward compatibility
    using BlockFixedBase_t = RealJointsBlock_t;
    using FixedBaseMx_t = RealJointsBlock_matrix_t;

    JSIM(const InertiaProperties&, Transforms&);
    ~JSIM() {}

    const JSIM& update(const JState_t&);

    /**
     * Computes the matrix L of the L^T L factorization *of the
     * actuated-joints block* of this JSIM.
     */
    void computeL();

    /**
     * Computes the inverse of the matrix L.
     * Assumes computeL() was called before.
     */
    void computeLInverse();

    /**
     * Computes the inverse of *of the actuated-joints block* of this JSIM.
     * This function does call computeLInverse() first.
     * The algorithm takes advantage of the branch
     * induced sparsity of the robot, if any.
     * Note that this matrix is NOT the full inverse of this JSIM.
     */
    void computeInverse();

    /**
     * Returns an unmodifiable reference to the matrix L.
     * See also computeL()
     */
    const RealJointsBlock_matrix_t& getL() const { return L; }

    /**
     * Returns an unmodifiable reference to the inverse of L.
     * See also computeLInverse()
     */
    const RealJointsBlock_matrix_t& getLInverse() const { return Linv; }

    /**
     * Returns an unmodifiable reference to the last computed inverse of the
     * real joints block.
     * See also computeInverse()
     */
    const RealJointsBlock_matrix_t& getInverse() const { return inverse; }

    /**
     * The spatial composite-inertia tensor of the robot base.
     *
     * Ie, the inertia of the whole robot for the current configuration.
     * According to the convention of this class about the layout of the
     * floating-base JSIM, this tensor is the 6x6 upper left corner of
     * the JSIM itself.
     * \return the 6x6 InertiaMatrix that correspond to the spatial inertia
     *   tensor of the whole robot, according to the last joints configuration
     *   used to update this JSIM
     */
    const InertiaMatrix& getWholeBodyInertia() const {
        return Ic_trunk;
    }
    /**
     * The matrix that maps accelerations in the actual joints of the robot
     * to the spatial force acting on the floating-base of the robot.
     * This matrix is the F sub-block of the JSIM in Featherstone's notation.
     * \return the 6x12 upper right block of this JSIM
     */
    const FBlock_t getF() const {
        return this->template block<6,12>(0,6);
    }
    /**
     * The submatrix of this JSIM related only to the actual joints of the
     * robot (as for a fixed-base robot).
     * This matrix is the H sub-block of the JSIM in Featherstone's notation.
     * \return the 12x12 lower right block of this JSIM,
     *   which correspond to the fixed-base JSIM
     */
    const RealJointsBlock_t getRealJointsBlock() const {
        return this->template block<12,12>(6,6);
    }
    const RealJointsBlock_t getFixedBaseBlock() const {
        return this->template block<12,12>(6,6);
    }

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
    InertiaMatrix Ic_trunk;
    InertiaMatrix Ic_LF_hipassembly;
    InertiaMatrix Ic_LF_upperleg;
    const InertiaMatrix& Ic_LF_lowerleg;
    InertiaMatrix Ic_RF_hipassembly;
    InertiaMatrix Ic_RF_upperleg;
    const InertiaMatrix& Ic_RF_lowerleg;
    InertiaMatrix Ic_LH_hipassembly;
    InertiaMatrix Ic_LH_upperleg;
    const InertiaMatrix& Ic_LH_lowerleg;
    InertiaMatrix Ic_RH_hipassembly;
    InertiaMatrix Ic_RH_upperleg;
    const InertiaMatrix& Ic_RH_lowerleg;
    InertiaMatrix Ic_spare;

    RealJointsBlock_matrix_t L;
    RealJointsBlock_matrix_t Linv;
    RealJointsBlock_matrix_t inverse;
};


template<typename Derived>
void JSIM::LinvT_times_x(const iit::rbd::MatrixBase<Derived>& x_cnt)
{
    auto& x = const_cast<iit::rbd::MatrixBase<Derived>& >(x_cnt);
    //assumes L has been computed already
    x(RH_KFE) /= L(RH_KFE, RH_KFE);
    x(RH_HFE) -= L(RH_KFE, RH_HFE) * x(RH_KFE);
    x(RH_HAA) -= L(RH_KFE, RH_HAA) * x(RH_KFE);
    x(RH_HFE) /= L(RH_HFE, RH_HFE);
    x(RH_HAA) -= L(RH_HFE, RH_HAA) * x(RH_HFE);
    x(RH_HAA) /= L(RH_HAA, RH_HAA);
    x(LH_KFE) /= L(LH_KFE, LH_KFE);
    x(LH_HFE) -= L(LH_KFE, LH_HFE) * x(LH_KFE);
    x(LH_HAA) -= L(LH_KFE, LH_HAA) * x(LH_KFE);
    x(LH_HFE) /= L(LH_HFE, LH_HFE);
    x(LH_HAA) -= L(LH_HFE, LH_HAA) * x(LH_HFE);
    x(LH_HAA) /= L(LH_HAA, LH_HAA);
    x(RF_KFE) /= L(RF_KFE, RF_KFE);
    x(RF_HFE) -= L(RF_KFE, RF_HFE) * x(RF_KFE);
    x(RF_HAA) -= L(RF_KFE, RF_HAA) * x(RF_KFE);
    x(RF_HFE) /= L(RF_HFE, RF_HFE);
    x(RF_HAA) -= L(RF_HFE, RF_HAA) * x(RF_HFE);
    x(RF_HAA) /= L(RF_HAA, RF_HAA);
    x(LF_KFE) /= L(LF_KFE, LF_KFE);
    x(LF_HFE) -= L(LF_KFE, LF_HFE) * x(LF_KFE);
    x(LF_HAA) -= L(LF_KFE, LF_HAA) * x(LF_KFE);
    x(LF_HFE) /= L(LF_HFE, LF_HFE);
    x(LF_HAA) -= L(LF_HFE, LF_HAA) * x(LF_HFE);
    x(LF_HAA) /= L(LF_HAA, LF_HAA);
}


template<typename Derived>
void JSIM::Linv_times_x(const iit::rbd::MatrixBase<Derived>& x_cnt)
{
    auto& x = const_cast<iit::rbd::MatrixBase<Derived>& >(x_cnt);
    //assumes L has been computed already
    x(LF_HAA) /= L(LF_HAA, LF_HAA);
    x(LF_HFE) -= L(LF_HFE, LF_HAA) * x(LF_HAA);
    x(LF_HFE) /= L(LF_HFE, LF_HFE);
    x(LF_KFE) -= L(LF_KFE, LF_HFE) * x(LF_HFE);
    x(LF_KFE) -= L(LF_KFE, LF_HAA) * x(LF_HAA);
    x(LF_KFE) /= L(LF_KFE, LF_KFE);
    x(RF_HAA) /= L(RF_HAA, RF_HAA);
    x(RF_HFE) -= L(RF_HFE, RF_HAA) * x(RF_HAA);
    x(RF_HFE) /= L(RF_HFE, RF_HFE);
    x(RF_KFE) -= L(RF_KFE, RF_HFE) * x(RF_HFE);
    x(RF_KFE) -= L(RF_KFE, RF_HAA) * x(RF_HAA);
    x(RF_KFE) /= L(RF_KFE, RF_KFE);
    x(LH_HAA) /= L(LH_HAA, LH_HAA);
    x(LH_HFE) -= L(LH_HFE, LH_HAA) * x(LH_HAA);
    x(LH_HFE) /= L(LH_HFE, LH_HFE);
    x(LH_KFE) -= L(LH_KFE, LH_HFE) * x(LH_HFE);
    x(LH_KFE) -= L(LH_KFE, LH_HAA) * x(LH_HAA);
    x(LH_KFE) /= L(LH_KFE, LH_KFE);
    x(RH_HAA) /= L(RH_HAA, RH_HAA);
    x(RH_HFE) -= L(RH_HFE, RH_HAA) * x(RH_HAA);
    x(RH_HFE) /= L(RH_HFE, RH_HFE);
    x(RH_KFE) -= L(RH_KFE, RH_HFE) * x(RH_HFE);
    x(RH_KFE) -= L(RH_KFE, RH_HAA) * x(RH_HAA);
    x(RH_KFE) /= L(RH_KFE, RH_KFE);
}


}
}


#endif
