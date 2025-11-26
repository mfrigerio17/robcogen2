local template = [[
#ifndef «include_guard»
#define «include_guard»

#include "«headers.main»"
#include "«headers.inertia»"
#include "«headers.transforms»"
//#include "«headers.fwd_dyn»"
#include "«headers.inv_dyn»"

${ns.open}

struct Traits
{
    using «types.scalarTraits» = «ns.qualifier»::«types.scalarTraits»;
    using «types.jointState» = «ns.qualifier»::«types.jointState»;

    using JointID = «ns.qualifier»::«types.jointIDs»;
    using LinkID  = «ns.qualifier»::«types.linkIDs»;

    using Transforms =  typename «ns.qualifier»::«classes.transforms»;

    using InertiaProperties = typename «ns.qualifier»::InertiaProperties ;
    //typedef typename «ns.qualifier»::ForwardDynamics FwdDynEngine;
    using InvDynEngine = typename «ns.qualifier»::InverseDynamics;
    //typedef typename JSIM JSIM; // TODO

    static constexpr int joints_count{«ns.qualifier»::jointsCount};
    static constexpr int links_count{«ns.qualifier»::linksCount};
    static const bool floating_base{«robot.isFloatingBase»};

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return «ns.qualifier»::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return «ns.qualifier»::orderedLinkIDs;
}

${ns.close}

#endif
]]

return template
