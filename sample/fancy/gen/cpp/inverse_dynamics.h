#ifndef RCG2_FANCY_INVERSE_DYNAMICS_H
#define RCG2_FANCY_INVERSE_DYNAMICS_H

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "rbd_types.h"
#include "inertia_properties.h"
#include "transforms.h"

namespace fancy {
namespace rcg2 {

/**
 * The Inverse Dynamics routine for the robot Fancy.
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
     *     the robot Fancy, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(const InertiaProperties& ip, Transforms& xt);

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JState_t& q) {
        xt.update(q);
    }

    /** \name Inverse dynamics
     * The full Newton-Euler algorithm for the inverse dynamics of this robot.
     *
     * \param[out] tau the joint force vector required to achieve the desired accelerations
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] const ExtForces& fext the external forces acting on the links; this parameters
     *            defaults to zero
     */
    ///@{
    void id(
        JState_t& tau,
        const JState_t& q, const JState_t& qd, const JState_t& qdd,
        const ExtForces& fext = zeroExtForces)
    {
        setJointStatus(q);
        id(tau, qd, qdd, fext);
    }

    void id(
        JState_t& tau,
        const JState_t& qd, const JState_t& qdd,
        const ExtForces& fext = zeroExtForces)
    {
        firstPass(qd, qdd, fext);
        secondPass(tau);
    }
    ///@}

    /** \name Gravity terms
     * The joint forces (linear or rotational) required to compensate
     * for the effect of gravity, in a specific configuration.
     */
    ///@{
    void G_terms(JState_t& tau, const JState_t& q) {
        setJointStatus(q);
        G_terms(tau);
    }
    void G_terms(JState_t& tau);
    ///@}

    /** \name Centrifugal and Coriolis terms
     * The forces (linear or rotational) acting on the joints due to centrifugal and
     * Coriolis effects, for a specific configuration.
     */
    ///@{
    void C_terms(JState_t& tau, const JState_t& q, const JState_t& qd) {
        setJointStatus(q);
        C_terms(tau, qd);
    }
    void C_terms(JState_t& tau, const JState_t& qd);
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
    // Link 'link1' :
    const InertiaMatrix& I_link1;
    Velocity      v_link1;
    Acceleration  a_link1;
    Force         f_link1;

    // Link 'link2' :
    const InertiaMatrix& I_link2;
    Velocity      v_link2;
    Acceleration  a_link2;
    Force         f_link2;

    // Link 'link3' :
    const InertiaMatrix& I_link3;
    Velocity      v_link3;
    Acceleration  a_link3;
    Force         f_link3;

    // Link 'link4' :
    const InertiaMatrix& I_link4;
    Velocity      v_link4;
    Acceleration  a_link4;
    Force         f_link4;

    // Link 'link5' :
    const InertiaMatrix& I_link5;
    Velocity      v_link5;
    Acceleration  a_link5;
    Force         f_link5;


    ///@}


protected:
    void firstPass(const JState_t& qd, const JState_t& qdd, const ExtForces& fext);
    void secondPass(JState_t& tau);

private:
    Transforms& xt;
    Matrix66 vcross; // support variable

private:
    static const ExtForces zeroExtForces;
};

}
}


#endif
