#ifndef RCG2_HYQ_INVERSE_DYNAMICS_H
#define RCG2_HYQ_INVERSE_DYNAMICS_H

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "rbd_types.h"
#include "inertia_properties.h"
#include "transforms.h"

namespace hyq {
namespace rcg2 {

/**
 * The Inverse Dynamics routine for the robot HyQ.
 *
 * In addition to the full Newton-Euler algorithm, specialized versions
 * to compute only certain terms are provided.
 * The parameters common to most of the methods are the joint status
 * vector \c q, the joint velocity vector \c qd and the acceleration
 * vector \c qdd.
 *
 * Additional overloaded methods are provided without the \c q
 * parameter. These methods use the current configuration of the robot;
 * they are provided for the sake of efficiency, in case the kinematics
 * (motion coordinate transforms) of the robot was updated elsewhere
 * with the most recent configuration (eg by a call to setJointStatus()).
 *
 * Whenever present, the external forces parameter is a set of external
 * wrenches acting on the robot links. Each wrench must be expressed in
 * the reference frame of the link it is excerted on.
 */

struct InverseDynamics
{
    using JState_t = JointState;
    using ExtForces = LinkDataMap<Force>;

    /**
     * Default constructor
     * \param ip the inertia properties of the links
     * \param xt the container of all the spatial motion transforms of
     *     the robot HyQ, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(const InertiaProperties& ip, Transforms& xt);

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JState_t& q) {
        xt.update(q);
    }

    /** \name Inverse dynamics
     * The full algorithm for the inverse dynamics of this robot.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] tau the joint force vector required to achieve the desired accelerations
     * \param[out] a_trunk the spatial acceleration of the robot base
     * \param[in] gravity the gravity acceleration, as a spatial vector;
     *      gravity implicitly specifies the orientation of the base in space
     * \param[in] v_trunk the spatial velocity of the base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id(
        JState_t& tau, Acceleration& a_trunk,
        const Acceleration& gravity, const Velocity& v_trunk,
        const JState_t& q, const JState_t& qd, const JState_t& qdd,
        const ExtForces& fext = zeroExtForces)
    {
        setJointStatus(q);
        id(tau, a_trunk, gravity, v_trunk, qd, qdd, fext);
    }
    void id(
        JState_t& tau, Acceleration& a_trunk,
        const Acceleration& gravity, const Velocity& v_trunk,
        const JState_t& qd, const JState_t& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Inverse dynamics, fully actuated base
     * The inverse dynamics algorithm for the floating base robot,
     * in the assumption of a fully actuated base.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] base_f the spatial force to be applied to
     *   the robot base to achieve the desired accelerations
     * \param[out] tau the joint force vector required to achieve the desired accelerations
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] v_trunk the spatial velocity of the base
     * \param[in] a_trunk the desired spatial acceleration of the robot base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] const ExtForces& fext the external forces acting on the links; this parameter
     *            defaults to zero
     */ ///@{
    void id_fully_actuated(
        Force& base_f, JState_t& tau,
        const Acceleration& gravity, const Velocity& v_trunk, const Acceleration& base_a,
        const JState_t& q, const JState_t& qd, const JState_t& qdd,
        const ExtForces& fext = zeroExtForces)
    {
        setJointStatus(q);
        id_fully_actuated(base_f, tau, gravity, v_trunk,
            base_a, qd, qdd, fext);
    }
    void id_fully_actuated(
        Force& base_f, JState_t& tau,
        const Acceleration& gravity, const Velocity& v_trunk, const Acceleration& base_a,
        const JState_t& qd, const JState_t& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}

    /** \name Gravity terms, fully actuated base
     */
    ///@{
    void G_terms_fully_actuated(
        Force& base_f, JState_t& tau,
        const Acceleration& gravity, const JState_t& q)
    {
        setJointStatus(q);
        G_terms_fully_actuated(base_f, tau, gravity);
    }
    void G_terms_fully_actuated(
        Force& base_f, JState_t& tau,
        const Acceleration& gravity);
    ///@}

    /** \name Centrifugal and Coriolis terms, fully actuated base
     *
     * These functions take only velocity inputs, that is, they assume
     * a zero spatial acceleration of the base (in addition to zero acceleration
     * at the actuated joints).
     * Note that this is NOT the same as imposing zero acceleration
     * at the virtual 6-dof-floting-base joint, which would result, in general,
     * in a non-zero spatial acceleration of the base, due to velocity
     * product terms.
     */
    ///@{
    void C_terms_fully_actuated(
        Force& base_f, JState_t& tau,
        const Velocity& v_trunk, const JState_t& q, const JState_t& qd)
    {
        setJointStatus(q);
        C_terms_fully_actuated(base_f, tau, v_trunk, qd);
    }
    void C_terms_fully_actuated(
        Force& base_f, JState_t& tau,
        const Velocity& v_trunk, const JState_t& qd);
    ///@}


    /** \name State variables per link
     * The various spatial quantities used internally
     * by the inverse dynamics routines, like the spatial acceleration
     * of the links.
     *
     * These are not returned explicitly by the inverse dynamics
     * routines even though they are computed. For example, after a call
     * to the inverse dynamics,
     * the spatial velocity of all the links has been determined and
     * can be accessed.
     *
     * However, beware that certain routines might not use some of the
     * spatial quantities, which therefore would retain their last value
     * without being updated nor reset (for example, the spatial velocity
     * of the links is unaffected by the computation of the gravity terms).
     */
    ///@{
    // Link 'LF_hipassembly' :
    const InertiaMatrix& I_LF_hipassembly;
    Velocity      v_LF_hipassembly;
    Acceleration  a_LF_hipassembly;
    Force         f_LF_hipassembly;

    // Link 'LF_upperleg' :
    const InertiaMatrix& I_LF_upperleg;
    Velocity      v_LF_upperleg;
    Acceleration  a_LF_upperleg;
    Force         f_LF_upperleg;

    // Link 'LF_lowerleg' :
    const InertiaMatrix& I_LF_lowerleg;
    Velocity      v_LF_lowerleg;
    Acceleration  a_LF_lowerleg;
    Force         f_LF_lowerleg;

    // Link 'RF_hipassembly' :
    const InertiaMatrix& I_RF_hipassembly;
    Velocity      v_RF_hipassembly;
    Acceleration  a_RF_hipassembly;
    Force         f_RF_hipassembly;

    // Link 'RF_upperleg' :
    const InertiaMatrix& I_RF_upperleg;
    Velocity      v_RF_upperleg;
    Acceleration  a_RF_upperleg;
    Force         f_RF_upperleg;

    // Link 'RF_lowerleg' :
    const InertiaMatrix& I_RF_lowerleg;
    Velocity      v_RF_lowerleg;
    Acceleration  a_RF_lowerleg;
    Force         f_RF_lowerleg;

    // Link 'LH_hipassembly' :
    const InertiaMatrix& I_LH_hipassembly;
    Velocity      v_LH_hipassembly;
    Acceleration  a_LH_hipassembly;
    Force         f_LH_hipassembly;

    // Link 'LH_upperleg' :
    const InertiaMatrix& I_LH_upperleg;
    Velocity      v_LH_upperleg;
    Acceleration  a_LH_upperleg;
    Force         f_LH_upperleg;

    // Link 'LH_lowerleg' :
    const InertiaMatrix& I_LH_lowerleg;
    Velocity      v_LH_lowerleg;
    Acceleration  a_LH_lowerleg;
    Force         f_LH_lowerleg;

    // Link 'RH_hipassembly' :
    const InertiaMatrix& I_RH_hipassembly;
    Velocity      v_RH_hipassembly;
    Acceleration  a_RH_hipassembly;
    Force         f_RH_hipassembly;

    // Link 'RH_upperleg' :
    const InertiaMatrix& I_RH_upperleg;
    Velocity      v_RH_upperleg;
    Acceleration  a_RH_upperleg;
    Force         f_RH_upperleg;

    // Link 'RH_lowerleg' :
    const InertiaMatrix& I_RH_lowerleg;
    Velocity      v_RH_lowerleg;
    Acceleration  a_RH_lowerleg;
    Force         f_RH_lowerleg;


    // The robot base
    const InertiaMatrix& I_trunk;
    InertiaMatrix Ic_trunk;
    Force         f_trunk;

    // The composite inertia tensors
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
    ///@}


protected:
    void sweep_inwards_fully_actuated(JState_t& tau);

private:
    Transforms& xt;
    Matrix66 vcross; // support variable

private:
    static const ExtForces zeroExtForces;
};

}
}


#endif
