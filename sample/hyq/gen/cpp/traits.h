#ifndef RCG2_HYQ_TRAITS_H
#define RCG2_HYQ_TRAITS_H

#include "declarations.h"
#include "rbd_types.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "forward_dynamics.h"
#include "inverse_dynamics.h"
#include "jsim.h"

namespace hyq {
namespace rcg2 {


struct Traits
{
    using ScalarTraits = hyq::rcg2::ScalarTraits;
    using JointState = hyq::rcg2::JointState;

    using JointID = hyq::rcg2::JointIDs;
    using LinkID  = hyq::rcg2::LinkIDs;

    static constexpr int joints_count{hyq::rcg2::jointsCount};
    static constexpr int links_count{hyq::rcg2::linksCount};
    static constexpr bool floating_base{true};
    using ExtForces = hyq::rcg2::LinkDataMap<Force>;
    using Transforms =  typename hyq::rcg2::Transforms;

    using InertiaProperties = typename hyq::rcg2::InertiaProperties;
    using InvDynEngine = typename hyq::rcg2::InverseDynamics;
    using JSIM = typename hyq::rcg2::JSIM;
    using FwdDynEngine = typename hyq::rcg2::ForwardDynamics;

    static inline constexpr const JointID* orderedJointIDs() {
        return hyq::rcg2::orderedJointIDs;
    }
    static inline constexpr const LinkID*  orderedLinkIDs() {
        return hyq::rcg2::orderedLinkIDs;
    }
};


}
}

#endif
