#ifndef RCG2_FANCY_TRAITS_H
#define RCG2_FANCY_TRAITS_H

#include "declarations.h"
#include "rbd_types.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "forward_dynamics.h"
#include "inverse_dynamics.h"
#include "jsim.h"

namespace fancy {
namespace rcg2 {


struct Traits
{
    using ScalarTraits = fancy::rcg2::ScalarTraits;
    using JointState = fancy::rcg2::JointState;

    using JointID = fancy::rcg2::JointIDs;
    using LinkID  = fancy::rcg2::LinkIDs;

    static constexpr int joints_count{fancy::rcg2::jointsCount};
    static constexpr int links_count{fancy::rcg2::linksCount};
    static constexpr bool floating_base{false};
    using ExtForces = fancy::rcg2::LinkDataMap<Force>;
    using Transforms =  typename fancy::rcg2::Transforms;

    using InertiaProperties = typename fancy::rcg2::InertiaProperties;
    using InvDynEngine = typename fancy::rcg2::InverseDynamics;
    using JSIM = typename fancy::rcg2::JSIM;
    using FwdDynEngine = typename fancy::rcg2::ForwardDynamics;

    static inline constexpr const JointID* orderedJointIDs() {
        return fancy::rcg2::orderedJointIDs;
    }
    static inline constexpr const LinkID*  orderedLinkIDs() {
        return fancy::rcg2::orderedLinkIDs;
    }
};


}
}

#endif
