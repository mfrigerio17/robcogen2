
local types = [[
#ifndef «include_guard»
#define «include_guard»

#include <iit/rbd/rbd.h>
#include <iit/rbd/scalar_traits.h>
#include <iit/rbd/InertiaMatrix.h>

${ns.open}

@local tpl   = common.scalarTpl(classes.coreTypes)
@local rbdns = ns_iit_rbd.qualifier
@if templateAll then
«tpl.heading»
using «types.scalarTraits» = «rbdns»::ScalarTraits«tpl.suffix»;

«tpl.heading»
using TypesGen = «rbdns»::Core«tpl.suffix»;

template<typename «tpl.scalar_t», int R, int C>
using Matrix = typename «rbdns»::PlainMatrix<«tpl.scalar_t», R, C>;

#define «typesMacro» \
using Force        = typename TypesGen«tpl.suffix»::ForceVector;    \
using Velocity     = typename TypesGen«tpl.suffix»::VelocityVector; \
using Acceleration = typename TypesGen«tpl.suffix»::VelocityVector; \
using Matrix66     = typename TypesGen«tpl.suffix»::Matrix66;       \
using Column6      = typename TypesGen«tpl.suffix»::Column6D;        \
using «types.vec3» = typename TypesGen«tpl.suffix»::Vector3; \
using InertiaMatrix= «rbdns»::InertiaMat«tpl.suffix»;

@   else
typedef typename «rbdns»::DoubleTraits «types.scalarTraits»;
typedef typename «types.scalarTraits»::Scalar «types.scalar»;

typedef «rbdns»::Core<«types.scalar»> TypesGen;
typedef TypesGen::ForceVector     Force;
typedef TypesGen::VelocityVector  Velocity;
typedef TypesGen::VelocityVector  Acceleration;
typedef TypesGen::Matrix66        Matrix66;
typedef TypesGen::Column6D        Column6;
typedef TypesGen::Vector3         «types.vec3»;

template<int R, int C>
using Matrix = «rbdns»::PlainMatrix<«types.scalar», R, C>;

using InertiaMatrix = «rbdns»::InertiaMat<«types.scalar»>;

@end

${ns.close}
#endif
]]

local main = [[
#ifndef «include_guard»
#define «include_guard»

#include <iit/rbd/data_map.h>
#include "«headers.types»"

${ns.open}

@local jsize = robot.tree.nJ
constexpr int JointSpaceDimension{«jsize»};
constexpr int jointsCount{«jsize»};
/** The total number of rigid bodies that can move */
@if robot.isFloatingBase then
constexpr int linksCount{«robot.tree.nB»};
@else
constexpr int linksCount{«robot.tree.nB-1»};
@end

@if templateAll then
template<typename «types.scalar»>
using Column«jsize»d = Matrix<«types.scalar», «jsize», 1>;

template<typename «types.scalar»>
using «types.jointState» = Column«jsize»d<«types.scalar»>;
@else
typedef Matrix<«jsize», 1> Column«jsize»d;
typedef Column«jsize»d «types.jointState»;
@end

@local jointIDs = utils.comma_separated_list(utils.i_iterator_decorator(function() return sorted_joints() end, common.jointIdentifier))
@local linkIDs  = utils.comma_separated_list(utils.i_iterator_decorator(function() return sorted_links(robot.isFloatingBase) end,  common.linkIdentifier))
enum «types.jointIDs» {
    «jointIDs»
};

enum «types.linkIDs» {
    «linkIDs»
};

static constexpr const «types.jointIDs» orderedJointIDs[jointsCount] = {
    «jointIDs»
};

static constexpr const «types.linkIDs» orderedLinkIDs[linksCount] = {
    «linkIDs»
};

template<typename T>
using LinkDataMap = «ns_iit_rbd.qualifier»::DataMap<T, linksCount, «types.linkIDs»>;

template<typename T>
using JointDataMap = «ns_iit_rbd.qualifier»::DataMap<T, jointsCount, «types.jointIDs»>;

${ns.close}
#endif
]]


local traits = [[
@local tpl = common.scalarTpl("Traits")
#ifndef «include_guard»
#define «include_guard»

#include "«headers.main»"
#include "«headers.types»"
#include "«headers.inertia»"
#include "«headers.transforms»"
#include "«headers.fwd_dyn»"
#include "«headers.inv_dyn»"
#include "«headers.jsim»"

${ns.open}

«tpl.heading»
struct Traits
{
    using «types.scalarTraits» = «ns.qualifier»::«types.scalarTraits»«tpl.suffix»;
    using «types.jointState» = «ns.qualifier»::«types.jointState»«tpl.suffix»;

    using JointID = «ns.qualifier»::«types.jointIDs»;
    using LinkID  = «ns.qualifier»::«types.linkIDs»;

    static constexpr int joints_count{«ns.qualifier»::jointsCount};
    static constexpr int links_count{«ns.qualifier»::linksCount};
    static constexpr bool floating_base{«robot.isFloatingBase»};
@if templateAll then
    using ExtForces = «ns.qualifier»::LinkDataMap<typename TypesGen«tpl.suffix»::ForceVector>;
@else
    using ExtForces = «ns.qualifier»::LinkDataMap<Force>;
@end
    using Transforms =  typename «ns.qualifier»::«classes.transforms»«tpl.suffix»;

    using InertiaProperties = typename «ns.qualifier»::InertiaProperties«tpl.suffix»;
    using InvDynEngine = typename «ns.qualifier»::InverseDynamics«tpl.suffix»;
    using JSIM = typename «ns.qualifier»::JSIM«tpl.suffix»;
    using FwdDynEngine = typename «ns.qualifier»::ForwardDynamics«tpl.suffix»;

    static inline constexpr const JointID* orderedJointIDs() {
        return «ns.qualifier»::orderedJointIDs;
    }
    static inline constexpr const LinkID*  orderedLinkIDs() {
        return «ns.qualifier»::orderedLinkIDs;
    }
};


${ns.close}

#endif
]]




local function generators_core_headers(robot, configurator, env)
    local tpl_eval = RCG.utils.templates.tpl_eval
    env.types_core_header = configurator.files.h_types

    local function gen_types_header()
        env.include_guard = env.includeGuard(configurator.files.h_types)
        return tpl_eval(types, env)
    end

    local function gen_main_header()
        env.include_guard = env.includeGuard(configurator.files.h_main)
        return tpl_eval(main, env)
    end

    local function gen_traits()
        env.include_guard = env.includeGuard(configurator.files.h_traits)
        return tpl_eval(traits, env)
    end

    return {
        types = gen_types_header,
        main  = gen_main_header,
        traits= gen_traits,
    }
end

return generators_core_headers

