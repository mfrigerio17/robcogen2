#ifndef RCG2_FANCY_FORWARD_DYNAMICS_H
#define RCG2_FANCY_FORWARD_DYNAMICS_H

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/robcogen_commons.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"

namespace fancy {
namespace rcg2 {

/**
 * The Forward Dynamics solver for the robot Fancy.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values.
 *
 * Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */

struct ForwardDynamics
{
    using JState_t = JointState;
    using ExtForces = LinkDataMap<Force>;

    /**
     * Default constructor
     * \param ip the inertia properties of the links
     * \param xt the container of all the coordinate transforms of
     *     the robot Fancy, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(const InertiaProperties& ip,
                 Transforms& xt);

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) {
        xt.update(q);
    }

    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param[out] qdd the joint accelerations vector
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] tau
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
        JState_t& qdd,
        const JState_t& q, const JState_t& qd, const JState_t& tau,
        const ExtForces& fext = zeroExtForces)
    {
        setJointStatus(q);
        fd(qdd, qd, tau, fext);
    }
    void fd(
        JState_t& qdd,
        const JState_t& qd, const JState_t& tau,
        const ExtForces& fext = zeroExtForces);
    ///@}

    //void jsim_inverse(const JointState&, Hinv_t&);

public:
    const InertiaProperties& ip;
    Transforms& xt;

    mutable Matrix66 vcross; // support variables
    mutable Matrix66 IaB;    //

    // support variable for the propagation of articulated inertia
    // set to zero once, here; the zero coefficients are never touched, in the algorithms
    Matrix66 Ia_p{Matrix66::Zero()};   // for prismatic joint
    Matrix66 Ia_r{Matrix66::Zero()};   // for revolute joint

    // Link 'link1' :
    Matrix66      IA_link1;
    Velocity      v_link1;
    Acceleration  a_link1;
    Velocity      c_link1;
    Force         p_link1;

    Column6 link1_U;
    Scalar link1_D;
    Scalar link1_u;

    // Link 'link2' :
    Matrix66      IA_link2;
    Velocity      v_link2;
    Acceleration  a_link2;
    Velocity      c_link2;
    Force         p_link2;

    Column6 link2_U;
    Scalar link2_D;
    Scalar link2_u;

    // Link 'link3' :
    Matrix66      IA_link3;
    Velocity      v_link3;
    Acceleration  a_link3;
    Velocity      c_link3;
    Force         p_link3;

    Column6 link3_U;
    Scalar link3_D;
    Scalar link3_u;

    // Link 'link4' :
    Matrix66      IA_link4;
    Velocity      v_link4;
    Acceleration  a_link4;
    Velocity      c_link4;
    Force         p_link4;

    Column6 link4_U;
    Scalar link4_D;
    Scalar link4_u;

    // Link 'link5' :
    Matrix66      IA_link5;
    Velocity      v_link5;
    Acceleration  a_link5;
    Velocity      c_link5;
    Force         p_link5;

    Column6 link5_U;
    Scalar link5_D;
    Scalar link5_u;



private:
    static const ExtForces zeroExtForces;

};

}
}


#endif
