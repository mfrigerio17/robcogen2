#ifndef RCG2_HYQ_FORWARD_DYNAMICS_H
#define RCG2_HYQ_FORWARD_DYNAMICS_H

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/robcogen_commons.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"

namespace hyq {
namespace rcg2 {

/**
 * The Forward Dynamics solver for the robot HyQ.
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
     *     the robot HyQ, which will be used by this instance
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
     * \param[out] a_trunk
     * \param[in] v_trunk the spatial velocity of the base
     * \param[in] g the gravity acceleration, as a spatial vector in base
     *   coordinates;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] tau
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
       JState_t& qdd, Acceleration& a_trunk,
       const Velocity& v_trunk, const Acceleration& gravity,
       const JState_t& q, const JState_t& qd, const JState_t& tau,
       const ExtForces& fext = zeroExtForces)
    {
        setJointStatus(q);
        fd(qdd, a_trunk, v_trunk, gravity,
           qd, tau, fext);
    }
    void fd(
       JState_t& qdd, Acceleration& a_trunk,
       const Velocity& v_trunk, const Acceleration& gravity,
       const JState_t& qd, const JState_t& tau,
       const ExtForces& fext = zeroExtForces);
    ///@}

    //void jsim_inverse(const JointState&, Matrix66&, Finv_t&, Hinv_t&);

public:
    const InertiaProperties& ip;
    Transforms& xt;

    mutable Matrix66 vcross; // support variables
    mutable Matrix66 IaB;    //

    // support variable for the propagation of articulated inertia
    // set to zero once, here; the zero coefficients are never touched, in the algorithms
    Matrix66 Ia_r{Matrix66::Zero()};   // for revolute joint

    // Link 'LF_hipassembly' :
    Matrix66      IA_LF_hipassembly;
    Velocity      v_LF_hipassembly;
    Acceleration  a_LF_hipassembly;
    Velocity      c_LF_hipassembly;
    Force         p_LF_hipassembly;

    Column6 LF_hipassembly_U;
    Scalar LF_hipassembly_D;
    Scalar LF_hipassembly_u;

    // Link 'LF_upperleg' :
    Matrix66      IA_LF_upperleg;
    Velocity      v_LF_upperleg;
    Acceleration  a_LF_upperleg;
    Velocity      c_LF_upperleg;
    Force         p_LF_upperleg;

    Column6 LF_upperleg_U;
    Scalar LF_upperleg_D;
    Scalar LF_upperleg_u;

    // Link 'LF_lowerleg' :
    Matrix66      IA_LF_lowerleg;
    Velocity      v_LF_lowerleg;
    Acceleration  a_LF_lowerleg;
    Velocity      c_LF_lowerleg;
    Force         p_LF_lowerleg;

    Column6 LF_lowerleg_U;
    Scalar LF_lowerleg_D;
    Scalar LF_lowerleg_u;

    // Link 'RF_hipassembly' :
    Matrix66      IA_RF_hipassembly;
    Velocity      v_RF_hipassembly;
    Acceleration  a_RF_hipassembly;
    Velocity      c_RF_hipassembly;
    Force         p_RF_hipassembly;

    Column6 RF_hipassembly_U;
    Scalar RF_hipassembly_D;
    Scalar RF_hipassembly_u;

    // Link 'RF_upperleg' :
    Matrix66      IA_RF_upperleg;
    Velocity      v_RF_upperleg;
    Acceleration  a_RF_upperleg;
    Velocity      c_RF_upperleg;
    Force         p_RF_upperleg;

    Column6 RF_upperleg_U;
    Scalar RF_upperleg_D;
    Scalar RF_upperleg_u;

    // Link 'RF_lowerleg' :
    Matrix66      IA_RF_lowerleg;
    Velocity      v_RF_lowerleg;
    Acceleration  a_RF_lowerleg;
    Velocity      c_RF_lowerleg;
    Force         p_RF_lowerleg;

    Column6 RF_lowerleg_U;
    Scalar RF_lowerleg_D;
    Scalar RF_lowerleg_u;

    // Link 'LH_hipassembly' :
    Matrix66      IA_LH_hipassembly;
    Velocity      v_LH_hipassembly;
    Acceleration  a_LH_hipassembly;
    Velocity      c_LH_hipassembly;
    Force         p_LH_hipassembly;

    Column6 LH_hipassembly_U;
    Scalar LH_hipassembly_D;
    Scalar LH_hipassembly_u;

    // Link 'LH_upperleg' :
    Matrix66      IA_LH_upperleg;
    Velocity      v_LH_upperleg;
    Acceleration  a_LH_upperleg;
    Velocity      c_LH_upperleg;
    Force         p_LH_upperleg;

    Column6 LH_upperleg_U;
    Scalar LH_upperleg_D;
    Scalar LH_upperleg_u;

    // Link 'LH_lowerleg' :
    Matrix66      IA_LH_lowerleg;
    Velocity      v_LH_lowerleg;
    Acceleration  a_LH_lowerleg;
    Velocity      c_LH_lowerleg;
    Force         p_LH_lowerleg;

    Column6 LH_lowerleg_U;
    Scalar LH_lowerleg_D;
    Scalar LH_lowerleg_u;

    // Link 'RH_hipassembly' :
    Matrix66      IA_RH_hipassembly;
    Velocity      v_RH_hipassembly;
    Acceleration  a_RH_hipassembly;
    Velocity      c_RH_hipassembly;
    Force         p_RH_hipassembly;

    Column6 RH_hipassembly_U;
    Scalar RH_hipassembly_D;
    Scalar RH_hipassembly_u;

    // Link 'RH_upperleg' :
    Matrix66      IA_RH_upperleg;
    Velocity      v_RH_upperleg;
    Acceleration  a_RH_upperleg;
    Velocity      c_RH_upperleg;
    Force         p_RH_upperleg;

    Column6 RH_upperleg_U;
    Scalar RH_upperleg_D;
    Scalar RH_upperleg_u;

    // Link 'RH_lowerleg' :
    Matrix66      IA_RH_lowerleg;
    Velocity      v_RH_lowerleg;
    Acceleration  a_RH_lowerleg;
    Velocity      c_RH_lowerleg;
    Force         p_RH_lowerleg;

    Column6 RH_lowerleg_U;
    Scalar RH_lowerleg_D;
    Scalar RH_lowerleg_u;

    // The robot base
    Matrix66 IA_trunk;
    Force    p_trunk;


private:
    static const ExtForces zeroExtForces;

};

}
}


#endif
