local header_template = [[
#ifndef «include_guard»
#define «include_guard»

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/robcogen_commons.h>

#include "«headers.main»"
#include "«headers.inertia»"
#include "«headers.transforms»"

${ns.open}

/**
 * The Forward Dynamics solver for the robot «robot.name».
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
«tpl.heading»
struct «self.class»
{
@if templateAll then
«typesMacro»
@end
    using «t_jstate» = «types.jointState»«tpl.suffix»;
    using «self.local_types.fext» = LinkDataMap<Force>;

    /**
     * Default constructor
     * \param ip the inertia properties of the links
     * \param xt the container of all the coordinate transforms of
     *     the robot «robot.name», which will be used by this instance
     *     to compute the dynamics.
     */
    «self.class»(const «meta.inertia_properties.class»«tpl.suffix»& ip,
                 «meta.transforms_container.class»«tpl.suffix»& xt);

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const «types.jointState»«tpl.suffix»& q) {
        «self.members.xt».update(q);
    }

    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param[out] «self.params.qdd» the joint accelerations vector
@if robot.isFloatingBase then
     * \param[out] «self.params.basea»
     * \param[in] «self.params.basev» the spatial velocity of the base
     * \param[in] g the gravity acceleration, as a spatial vector in base
     *   coordinates;
     *              gravity implicitly specifies the orientation of the base in space
@end
     * \param[in] «self.params.q» the joint position vector
     * \param[in] «self.params.qd» the joint velocity vector
     * \param[in] «self.params.tau»
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
@if robot.isFloatingBase then
    void fd(
       «self.fparam.qdd», «self.fparam.basea»,
       «self.fparam.basev», «self.fparam.g»,
       «self.fparam.q», «self.fparam.qd», «self.fparam.tau»,
       «self.fparam.fext» = zeroExtForces)
    {
        setJointStatus(q);
        fd(«self.params.qdd», «self.params.basea», «self.params.basev», «self.params.g»,
           «self.params.qd», «self.params.tau», «self.params.fext»);
    }
    void fd(
       «self.fparam.qdd», «self.fparam.basea»,
       «self.fparam.basev», «self.fparam.g»,
       «self.fparam.qd», «self.fparam.tau»,
       «self.fparam.fext» = zeroExtForces);
@else
    void fd(
        «self.fparam.qdd»,
        «self.fparam.q», «self.fparam.qd», «self.fparam.tau»,
        «self.fparam.fext» = zeroExtForces)
    {
        setJointStatus(q);
        fd(«self.params.qdd», «self.params.qd», «self.params.tau», «self.params.fext»);
    }
    void fd(
        «self.fparam.qdd»,
        «self.fparam.qd», «self.fparam.tau»,
        «self.fparam.fext» = zeroExtForces);
@end
    ///@}

@if robot.isFloatingBase then
    //void «self.members.jsim_inverse»(const «types.jointState»«tpl.suffix»&, Matrix66&, Finv_t&, Hinv_t&);
@else
    //void «self.members.jsim_inverse»(const «types.jointState»«tpl.suffix»&, Hinv_t&);
@end

public:
    const «meta.inertia_properties.class»«tpl.suffix»& «self.members.ip»;
    «meta.transforms_container.class»«tpl.suffix»& «self.members.xt»;

    mutable Matrix66 vcross; // support variables
    mutable Matrix66 IaB;    //

    // support variable for the propagation of articulated inertia
    // set to zero once, here; the zero coefficients are never touched, in the algorithms
@if  robot.hasPrismaticJoint then
    Matrix66 Ia_p{Matrix66::«mxops.zeroMx»()};   // for prismatic joint
@end
@if  robot.hasRevoluteJoint then
    Matrix66 Ia_r{Matrix66::«mxops.zeroMx»()};   // for revolute joint
@end

@for name,link in sorted_links() do
    // Link '«name»' :
@    if robot.treeutils.isLeaf(link) then
    const InertiaMatrix& «vars.IA(link)»;
@    else
    Matrix66      «vars.IA(link)»;
@    end
    Velocity      «vars.vel(link)»;
    Acceleration  «vars.acc(link)»;
    Velocity      «vars.biasA(link)»;
    Force         «vars.biasF(link)»;

    Column6 «UTermName(link)»;
    «types.scalar» «DTermName(link)»;
    «types.scalar» «uTermName(link)»;

@end
@ if robot.isFloatingBase then
    // The robot base
    Matrix66 «vars.IA(robot.base)»;
    Force    «vars.biasF(robot.base)»;
@end


private:
    static const «self.local_types.fext» zeroExtForces;

};

${ns.close}

@if templateAll then
#include "«impl_files.fwd_dyn»"
@end

#endif
]]


local source_template = [[
@if not templateAll then
#include "«headers.fwd_dyn»"
@end

@local qualifier = ns.qualifier .. '::' .. tpl.class.in_qualifier

// Initialization of static-const data
«tpl.heading»
const typename «qualifier»::«self.local_types.fext»
«qualifier»::zeroExtForces(Force::Zero());

«tpl.heading»
«qualifier»::«self.class»(const «meta.inertia_properties.class»«tpl.suffix»& inertia, «meta.transforms_container.class»«tpl.suffix»& transforms) :
   «self.members.ip»(inertia), «self.members.xt»(transforms),
@for _, link, comma in utils.i_iterator_with_separator(function() return ipairs(leafs) end, ",") do
    «vars.IA(link)»(ip.«meta.inertia_properties.members.tensorGetter(link)»())«comma»
@end
{
@ for  _,link in sorted_links() do
    «vars.vel(link)».setZero();
    «vars.biasA(link)».setZero();
@end
    vcross.setZero();
    IaB.setZero(); //not really necessary, but avoids warning
}

«tpl.heading»

void «qualifier»::«self.class»::fd(
    «self.fparam.qdd»,
@if robot.isFloatingBase then
    «self.fparam.basea»,
    «self.fparam.basev», «self.fparam.g»,
@end
    «self.fparam.qd», «self.fparam.tau», «self.fparam.fext»)
{
    using namespace «ns_iit_rbd.qualifier»;

@for  _,link in sorted_links(robot.isFloatingBase) do
@   if not robot.treeutils.isLeaf(link) then
    «vars.IA(link)» = «self.members.ip».«meta.inertia_properties.members.tensorGetter(link)»();
@   end
    «vars.biasF(link)» = - «self.params.fext»[«common.linkIdentifier(link)»];
@end

// ---------------------- FIRST PASS ---------------------- //
// Note that, during the first pass, the articulated inertias are really
//  just the spatial inertia of the links (see assignments above).
//  Afterwards things change, and articulated inertias shall not be used
//  in functions which work specifically with spatial inertias.

@ for  name,link in sorted_links() do
@   local parent   = robot.treeutils.parent(link)
@   local joint    = robot.treeutils.supportingJoint(link)
@   local velocity = vars.vel(link)
@   local cterm    = vars.biasA(link)
@   local biasF    = vars.biasF(link)
@   local jid      = common.jointIdentifier(joint)
@   local idx      = common.spatialVectorIndex(joint)
@   local child_X_parent = child_mx_parent(link)

    // + Link «name»
@   if (parent==robot.base and not robot.isFloatingBase) then
    //  - The spatial velocity:
    «velocity»(«idx») = «self.params.qd»(«jid»);

    //  - The bias force term:
@       if joint.kind == RCG.enums.JointKind.prismatic then
    // The first joint is prismatic, no bias force term
@       else
    «biasF» += vxIv(«self.params.qd»(«jid»), «vars.IA(link)»);
@       end
@   else
    //  - The spatial velocity:
    «velocity» = «child_X_parent» * «vars.vel(parent)»;
    «velocity»(«idx») += «self.params.qd»(«jid»);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<«types.scalar»>(«velocity», vcross);
    «cterm» = vcross.col(«idx») * «self.params.qd»(«jid»);

    //  - The bias force term:
    «biasF» += vxIv(«velocity», «vars.IA(link)»);
@   end
@end

@if robot.isFloatingBase then
    // Bias force on the floating base
    «vars.biasF(robot.base)» += vxIv(«vars.vel(robot.base)», «vars.IA(robot.base)»);
@end

// ---------------------- SECOND PASS ---------------------- //
    Force pa;
@for name,link in sorted_links_reversed() do
@   local parent= robot.treeutils.parent(link)
@   local joint = robot.treeutils.supportingJoint(link)
@   local idx   = common.spatialVectorIndex(joint)
@   local U     = UTermName(link)
@   local D     = DTermName(link)
@   local u     = uTermName(link)
@   local p     = vars.biasF(link)
@   local I     = vars.IA(link)
@   local child_X_parent = common.link_CT_parent(link, self.members.xt)

    // + Link «name»
    «u» = «self.params.tau»(«common.jointIdentifier(joint)») - «p»(«idx»);
    «U» = «I».col(«idx»);
    «D» = «U»(«idx»);

@   if (parent~=robot.base or robot.isFloatingBase) then
@       if joint.kind == RCG.enums.JointKind.prismatic then
    compute_Ia_prismatic(«I», «U», «D», Ia_p);  // same as: Ia_p = «I» - «U»/«D» * «U».transpose();
    pa = «p» + Ia_p * «vars.biasA(link)» + «U» * «u»/«D»;
    ctransform_Ia_prismatic(Ia_p, «child_X_parent».ct, IaB);
@       else
    compute_Ia_revolute(«I», «U», «D», Ia_r);  // same as: Ia_r = «I» - «U»/«D» * «U».transpose();
    pa = «p» + Ia_r * «vars.biasA(link)» + «U» * «u»/«D»;
    ctransform_Ia_revolute(Ia_r, «child_X_parent».ct, IaB);
@       end
    «vars.IA(parent)» += IaB;
    «vars.biasF(parent)» += «common.parent_XF_link(link, self.members.xt)» * pa;
@   end
@end

@if robot.isFloatingBase then
    // + The acceleration of the floating base «robot.base.name», without gravity
    «vars.acc(robot.base)» = - «vars.IA(robot.base)».llt().solve(«vars.biasF(robot.base)»);  // «vars.acc(robot.base)» = - IA^-1 * «vars.biasF(robot.base)»
@end

// ---------------------- THIRD PASS ---------------------- //
@ for  name,link in sorted_links() do
@   local parent = robot.treeutils.parent(link)
@   local joint  = robot.treeutils.supportingJoint(link)
@   local jid    = common.jointIdentifier(joint)
@   local acc    = vars.acc(link)
@   if (parent==robot.base and not robot.isFloatingBase) then
    «acc» = «common.link_XM_parent(link, self.members.xt)».matrix().col(LZ) * «ns_iit_rbd.qualifier»::g;
@   else
    «acc» = «common.link_XM_parent(link, self.members.xt)» * «vars.acc(parent)» + «vars.biasA(link)»;
@   end
    «self.params.qdd»(«jid») = («uTermName(link)» - «UTermName(link)».dot(«vars.acc(link)»)) / «DTermName(link)»;
    «acc»(«common.spatialVectorIndex(joint)») += «self.params.qdd»(«jid»);

@end

@if robot.isFloatingBase then
    // + Add gravity to the acceleration of the floating base
    «vars.acc(robot.base)» += «self.params.g»;
@end
}

]]




local genutils  = tpl_utils
local GLOB = generators

local function fd_generators(robot, configurator, given_env)
    -- shallow copy the template environment, then add the fields required for
    -- the local templates
    local env = {}
    for k,v in pairs(given_env) do  env[k] = v  end

    local t_jstate = env.types.classScopeAliases.jointState
    local self = configurator.txtCfg.meta.forward_dynamics

    -- The names for the base v/a cannot be chosen freely, they must match the
    -- naming patterns configured for the robot links.
    -- This is because the code generation templates rely on such patterns
    -- in generic loops that will fill in the methods bodies: we must
    -- make sure that the generated identifiers match the arguments names
    self.params.basev = configurator.txtCfg.vars.vel(robot.base)
    self.params.basea = configurator.txtCfg.vars.acc(robot.base)
    self.fparam = {
        q   = "const " .. t_jstate .. "& " .. self.params.q,
        qd  = "const " .. t_jstate .. "& " .. self.params.qd,
        qdd = t_jstate .. "& " .. self.params.qdd,
        tau = "const " .. t_jstate .. "& " .. self.params.tau,
        basev = "const Velocity& " .. self.params.basev,
        basea = "Acceleration& " .. self.params.basea,
        basea_in = "const Acceleration& " .. self.params.basea_in,
        basef = "Force& " .. self.params.basef,
        g = "const Acceleration& " .. self.params.g,
        fext = "const " .. self.local_types.fext .. "& " .. self.params.fext,
    }
    env.self = self
    env.t_jstate = t_jstate
    env.tpl  = env.common.scalarTpl( self.class )
    env.include_guard = env.includeGuard(configurator.files.h_fwd_dyn)

    env.UTermName = function(link) return link.name..'_U' end
    env.uTermName = function(link) return link.name..'_u' end
    env.DTermName = function(link) return link.name..'_D' end
    env.child_mx_parent = function(link)
        return env.common.link_XM_parent(link, self.members.xt)
    end

    local tpleval = RCG.utils.templates.tpl_eval
    return {
        header = function() return tpleval(header_template, env) end,
        source = function() return tpleval(source_template, env) end,
    }
end


return fd_generators
