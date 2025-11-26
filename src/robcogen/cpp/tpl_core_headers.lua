-- Use a local alias for the expected global modules
local genutils  = RCG.utils.templates


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

#define «typesMacro» using Force        = typename TypesGen«tpl.suffix»::ForceVector;    \
using Velocity     = typename TypesGen«tpl.suffix»::VelocityVector; \
using Acceleration = typename TypesGen«tpl.suffix»::VelocityVector; \
using Matrix66     = typename TypesGen«tpl.suffix»::Matrix66;       \
using Column6      = typename TypesGen«tpl.suffix»::Column6D;        \
using «types.vec3» = typename TypesGen«tpl.suffix»::Vector3; \
template<int R, int C>                                              \
using Matrix = Matrix<«tpl.scalar_t», R, C>; \
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

@local jointIDs = utils.comma_separated_list(utils.i_iterator_decorator(function() return sorted_joints(robot) end, common.jointIdentifier))
@local linkIDs  = utils.comma_separated_list(utils.i_iterator_decorator(function() return sorted_links(robot, "include_base_if_floating") end,  common.linkIdentifier))
enum «types.jointIDs» {
    «jointIDs»
};

enum «types.linkIDs» {
    «linkIDs»
};

static const «types.jointIDs» orderedJointIDs[jointsCount] = {
    «jointIDs»
};

static const «types.linkIDs» orderedLinkIDs[linksCount] = {
    «linkIDs»
};


${ns.close}
#endif
]]


local traits = [[
#ifndef «include_guard»
#define «include_guard»

#include "«Names$Files::mainHeader(robot)».h"
#include "«Names$Files::transformsHeader(robot)».h"
#include "«Names$Files$RBD::invDynHeader(robot)».h"
#include "«Names$Files$RBD::fwdDynHeader(robot)».h"
#include "«Names$Files$RBD::jsimHeader(robot)».h"
#include "«Names$Files$RBD::inertiaHeader(robot)».h"

«Common.enclosingNamespacesOpen(robot)»
«val ns  = Common.enclosingNamespacesQualifier(robot)»
struct Traits {
    typedef typename «ns»::«Names$Types::scalarTraits» «Names$Types::scalarTraits»;

    typedef typename «ns»::«Names$Types::jointState» «Names$Types::jointState»;

    typedef typename «ns»::JointIdentifiers JointID;
    typedef typename «ns»::LinkIdentifiers  LinkID;

    typedef typename «ns»::«Names$Types$Transforms::homogeneous» «Names$Types$Transforms::homogeneous»;
    typedef typename «ns»::«Names$Types$Transforms::spatial_motion» «Names$Types$Transforms::spatial_motion»;
    typedef typename «ns»::«Names$Types$Transforms::spatial_force» «Names$Types$Transforms::spatial_force»;

    typedef typename «ns»::«LinkInertias::className(robot)» InertiaProperties;
    typedef typename «ns»::«ForwardDynamics::className(robot)» FwdDynEngine;
    typedef typename «ns»::«InverseDynamics::className(robot)» InvDynEngine;
    typedef typename «ns»::«Names$Types::jspaceMLocal» JSIM;

    static const int joints_count = «ns»::jointsCount;
    static const int links_count  = «ns»::linksCount;
    static const bool floating_base = «IF common.isFloating(robot.base)»true«ELSE»false«ENDIF»;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return «ns»::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return «ns»::orderedLinkIDs;
}

«Common::enclosingNamespacesClose(robot)»

#endif
]]




local function allGenerators(robot, configurator, env)

    env.types_core_header = configurator.files.h_types

    local function gen_types_header()
        env.include_guard = env.includeGuard(configurator.files.h_types)
        return genutils.tpl_eval(types, env)
    end

    local function gen_main_header()
        env.include_guard = env.includeGuard(configurator.files.h_main)
        return genutils.tpl_eval(main, env)
    end

    local function gen_traits()
        env.include_guard = env.includeGuard(configurator.files.h_traits)
        return genutils.tpl_eval(RCG.cpp.templates.traits, env)
    end

    return {
        types = gen_types_header,
        main  = gen_main_header,
        traits= gen_traits,
    }
end

generators.headers = allGenerators

