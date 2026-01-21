local header_template = [[
#ifndef «include_guard»
#define «include_guard»

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "«headers.main»"
#include "«headers.types»"
#include "«headers.inertia»"
#include "«headers.transforms»"
#include "«headers.data_map»"

${ns.open}

/**
 * The Inverse Dynamics routine for the robot «robot.name».
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
«tpl.heading»
struct «self.class»
{
@if templateAll then
«typesMacro»
@end
    using «t_jstate» = «types.jointState»«tpl.suffix»;
    using «self.local_types.fext» = ::rcg2::DataMap<Force, linksCount, «types.linkIDs»>;

    /**
     * Default constructor
     * \param ip the inertia properties of the links
     * \param xt the container of all the spatial motion transforms of
     *     the robot «robot.name», which will be used by this instance
     *     to compute inverse-dynamics.
     */
    «self.class»(const «meta.inertia_properties.class»«tpl.suffix»& ip, «meta.transforms_container.class»«tpl.suffix»& xt);

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(«self.fparam.q») {
        «self.members.xt».«meta.transforms_container.members.update»(«self.params.q»);
    }

@if robot.isFloatingBase then
    /** \name Inverse dynamics
     * The full algorithm for the inverse dynamics of this robot.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] «self.params.tau» the joint force vector required to achieve the desired accelerations
     * \param[out] «self.params.basea» the spatial acceleration of the robot base
     * \param[in] «self.params.g» the gravity acceleration, as a spatial vector;
     *      gravity implicitly specifies the orientation of the base in space
     * \param[in] «self.params.basev» the spatial velocity of the base
     * \param[in] «self.params.q» the joint position vector
     * \param[in] «self.params.qd» the joint velocity vector
     * \param[in] «self.params.qdd» the desired joint acceleration vector
     * \param[in] «self.params.fext» the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id(
        «self.fparam.tau», «self.fparam.basea»,
        «self.fparam.g», «self.fparam.basev»,
        «self.fparam.q», «self.fparam.qd», «self.fparam.qdd»,
        «self.fparam.fext» = zeroExtForces)
    {
        setJointStatus(«self.params.q»);
        id(«self.params.tau», «self.params.basea», «self.params.g», «self.params.basev», «self.params.qd», «self.params.qdd», «self.params.fext»);
    }
    void id(
        «self.fparam.tau», «self.fparam.basea»,
        «self.fparam.g», «self.fparam.basev»,
        «self.fparam.qd», «self.fparam.qdd»,
        «self.fparam.fext» = zeroExtForces);
    ///@}
    /** \name Inverse dynamics, fully actuated base
     * The inverse dynamics algorithm for the floating base robot,
     * in the assumption of a fully actuated base.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] «self.params.basef» the spatial force to be applied to
     *   the robot base to achieve the desired accelerations
     * \param[out] «self.params.tau» the joint force vector required to achieve the desired accelerations
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] «self.params.basev» the spatial velocity of the base
     * \param[in] «self.params.basea» the desired spatial acceleration of the robot base
     * \param[in] «self.params.q» the joint position vector
     * \param[in] «self.params.qd» the joint velocity vector
     * \param[in] «self.params.qdd» the desired joint acceleration vector
     * \param[in] «self.fparam.fext» the external forces acting on the links; this parameter
     *            defaults to zero
     */ ///@{
    void id_fully_actuated(
        «self.fparam.basef», «self.fparam.tau»,
        «self.fparam.g», «self.fparam.basev», «self.fparam.basea_in»,
        «self.fparam.q», «self.fparam.qd», «self.fparam.qdd»,
        «self.fparam.fext» = zeroExtForces)
    {
        setJointStatus(«self.params.q»);
        id_fully_actuated(«self.params.basef», «self.params.tau», «self.params.g», «self.params.basev»,
            «self.params.basea_in», «self.params.qd», «self.params.qdd», fext);
    }
    void id_fully_actuated(
        «self.fparam.basef», «self.fparam.tau»,
        «self.fparam.g», «self.fparam.basev», «self.fparam.basea_in»,
        «self.fparam.qd», «self.fparam.qdd»,
        «self.fparam.fext» = zeroExtForces);
    ///@}

    /** \name Gravity terms, fully actuated base
     */
    ///@{
    void G_terms_fully_actuated(
        «self.fparam.basef», «self.fparam.tau»,
        «self.fparam.g», «self.fparam.q»)
    {
        setJointStatus(«self.params.q»);
        G_terms_fully_actuated(«self.params.basef», «self.params.tau», «self.params.g»);
    }
    void G_terms_fully_actuated(
        «self.fparam.basef», «self.fparam.tau»,
        «self.fparam.g»);
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
        «self.fparam.basef», «self.fparam.tau»,
        «self.fparam.basev», «self.fparam.q», «self.fparam.qd»)
    {
        setJointStatus(«self.params.q»);
        C_terms_fully_actuated(«self.params.basef», «self.params.tau», «self.params.basev», qd);
    }
    void C_terms_fully_actuated(
        «self.fparam.basef», «self.fparam.tau»,
        «self.fparam.basev», «self.fparam.qd»);
    ///@}
@ else
    /** \name Inverse dynamics
     * The full Newton-Euler algorithm for the inverse dynamics of this robot.
     *
     * \param[out] «self.params.tau» the joint force vector required to achieve the desired accelerations
     * \param[in] «self.params.q» the joint position vector
     * \param[in] «self.params.qd» the joint velocity vector
     * \param[in] «self.params.qdd» the desired joint acceleration vector
     * \param[in] «self.fparam.fext» the external forces acting on the links; this parameters
     *            defaults to zero
     */
    ///@{
    void id(
        «self.fparam.tau»,
        «self.fparam.q», «self.fparam.qd», «self.fparam.qdd»,
        «self.fparam.fext» = zeroExtForces)
    {
        setJointStatus(«self.params.q»);
        id(«self.params.tau», «self.params.qd», «self.params.qdd», «self.params.fext»);
    }

    void id(
        «self.fparam.tau»,
        «self.fparam.qd», «self.fparam.qdd»,
        «self.fparam.fext» = zeroExtForces)
    {
        firstPass(«self.params.qd», «self.params.qdd», «self.params.fext»);
        secondPass(«self.params.tau»);
    }
    ///@}

    /** \name Gravity terms
     * The joint forces (linear or rotational) required to compensate
     * for the effect of gravity, in a specific configuration.
     */
    ///@{
    void G_terms(«self.fparam.tau», «self.fparam.q») {
        setJointStatus(«self.params.q»);
        G_terms(«self.params.tau»);
    }
    void G_terms(«self.fparam.tau»);
    ///@}

    /** \name Centrifugal and Coriolis terms
     * The forces (linear or rotational) acting on the joints due to centrifugal and
     * Coriolis effects, for a specific configuration.
     */
    ///@{
    void C_terms(«self.fparam.tau», «self.fparam.q», «self.fparam.qd») {
        setJointStatus(«self.params.q»);
        C_terms(«self.params.tau», «self.params.qd»);
    }
    void C_terms(«self.fparam.tau», «self.fparam.qd»);
    ///@}
@ end


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
@for name,link in sorted_links() do
    // Link '«name»' :
    const InertiaMatrix& «vars.I(link)»;
    Velocity      «vars.vel(link)»;
    Acceleration  «vars.acc(link)»;
    Force         «vars.force(link)»;

@ end

@ if robot.isFloatingBase then
    // The robot base
    const InertiaMatrix& «vars.I(robot.base)»;
    InertiaMatrix «vars.Ic(robot.base)»;
    Force         «vars.force(robot.base)»;

    // The composite inertia tensors
@   for _,link in sorted_links() do
@     if robot.treeutils.isLeaf(link) then
    const InertiaMatrix& «vars.Ic(link)»;
@     else
    InertiaMatrix «vars.Ic(link)»;
@     end
@   end
@ end
    ///@}


protected:
@ if robot.isFloatingBase then
    void sweep_inwards_fully_actuated(«self.fparam.tau»);
@ else
    void firstPass(«self.fparam.qd», «self.fparam.qdd», «self.fparam.fext»);
    void secondPass(«self.fparam.tau»);
@ end

private:
    «meta.transforms_container.class»«tpl.suffix»& «self.members.xt»;
    Matrix66 vcross; // support variable

private:
    static const «self.local_types.fext» zeroExtForces;
};

${ns.close}

@if templateAll then
#include "«impl_files.inv_dyn»"
@end

#endif
]]

local source_template = [[
#include <iit/rbd/robcogen_commons.h>

@if not templateAll then
#include "«headers.inv_dyn»"
@end

@local qualifier = ns.qualifier .. '::' .. tpl.class.in_qualifier

// Initialization of static-const data
«tpl.heading»
const typename «qualifier»::«self.local_types.fext»
«qualifier»::zeroExtForces(Force::Zero());

«tpl.heading»
«qualifier»::«self.class»(const «meta.inertia_properties.class»«tpl.suffix»& inertia, «meta.transforms_container.class»«tpl.suffix»& transforms) :
    // the local aliases for the inertia tensors:
@ for  _,link in sorted_links() do
    «vars.I(link)»( inertia.«meta.inertia_properties.members.tensorGetter(link)»() ),
@end
@if robot.isFloatingBase then
    «vars.I(robot.base)»( inertia.«meta.inertia_properties.members.tensorGetter(robot.base)»() ),
    // the composite inertia of leaf links IS the regular inertia
@   for _,link in sorted_links() do
@       if robot.treeutils.isLeaf(link) then
    «vars.Ic(link)»(«vars.I(link)»),
@       end
@   end
@end
    «self.members.xt»(transforms)
{
@for name,link in sorted_links() do
    «vars.vel(link)».setZero();
@ end

    vcross.setZero();
}

@ if robot.isFloatingBase then
${floating_base_methods_definitions}
@ else
${fixed_base_methods_definitions}
@end
]]

local fixed_base_methods_definitions = [[
@local qualifier = ns.qualifier .. '::' .. tpl.class.in_qualifier
«tpl.heading»
void «qualifier»::«self.class»::G_terms(«self.fparam.tau»)
{
    using namespace «ns_iit_rbd.qualifier»;
    ${fixed_base_pass1_G}

    secondPass(«self.params.tau»);
}
«tpl.heading»
void «qualifier»::«self.class»::C_terms(«self.fparam.tau», «self.fparam.qd»)
{
    using namespace «ns_iit_rbd.qualifier»;
    ${fixed_base_pass1_C}

    secondPass(«self.params.tau»);
}

«tpl.heading»
void «qualifier»::«self.class»::firstPass(«self.fparam.qd», «self.fparam.qdd», «self.fparam.fext»)
{
    using namespace «ns_iit_rbd.qualifier»;
    ${fixed_base_pass1}
}

«tpl.heading»
void «qualifier»::«self.class»::secondPass(«self.fparam.tau»)
{
    using namespace «ns_iit_rbd.qualifier»;
    ${fixed_base_pass2}
}
]]


local fixed_base_pass1 = [[
@for name,link in sorted_links() do
@   local parent   = robot.treeutils.parent(link)
@   local joint    = robot.treeutils.supportingJoint(link)
@   local velocity = vars.vel(link)
@   local acceler  = vars.acc(link)
@   local jid      = common.jointIdentifier(joint)
@   local idx      = common.spatialVectorIndex(joint)
@   local child_X_parent = child_mx_parent(link)
// Link '«name»'
@   --
@   if (parent~=robot.base) or (parent==robot.base and robot.isFloatingBase and not hybridDynamics) then
@   --
«velocity» = «child_X_parent» * «vars.vel(parent)»;
«velocity»(«idx») += «self.params.qd»(«jid»);

motionCrossProductMx<«types.scalar»>(«velocity», vcross);

«acceler» = «child_X_parent» * «vars.acc(parent)» + vcross.col(«idx») * «self.params.qd»(«jid»);
«acceler»(«idx») += «self.params.qdd»(«jid»);

«vars.force(link)» = «vars.I(link)» * «acceler» + vxIv(«velocity», «vars.I(link)») - «self.params.fext»[«common.linkIdentifier(link)»];
@   --
@   elseif not robot.isFloatingBase then -- parent IS the fixed-base
@   --
«velocity»(«idx») = «self.params.qd»(«jid»);   // «velocity» = vJ, for the first link of a fixed base robot
«acceler» = «child_X_parent».matrix().col(LZ) * «ns_iit_rbd.qualifier»::g;
«acceler»(«idx») += «self.params.qdd»(«jid»);
@       if joint.kind == RCG.enums.JointKind.prismatic then
// The first joint is prismatic, no centripetal terms.
«vars.force(link)» = «vars.I(link)» * «acceler» - «self.params.fext»[«common.linkIdentifier(link)»];
@       else
«vars.force(link)» = «vars.I(link)» * «acceler» + vxIv(«self.params.qd»(«jid»), «vars.I(link)») - «self.params.fext»[«common.linkIdentifier(link)»];
@       end
@   --
@   else -- parent IS the floating-base and we are coding hybrid dynamics
@   --
«velocity» = «child_X_parent» * «vars.vel(parent)»;
«velocity»(«idx») += «self.params.qd»(«jid»);

motionCrossProductMx<«types.scalar»>(«velocity», vcross);

«acceler» = vcross.col(«idx») * «self.params.qd»(«jid»);
«acceler»(«idx») += «self.params.qdd»(«jid»);

«vars.force(link)» = «vars.I(link)» * «acceler» + vxIv(«velocity», «vars.I(link)») - «self.params.fext»[«common.linkIdentifier(link)»];
@   end

@end
]]


local fixed_base_pass2 = [[
@for name,link in sorted_links_reversed() do
// Link '«name»'
@   local parent   = robot.treeutils.parent(link)
@   local joint    = robot.treeutils.supportingJoint(link)
«self.params.tau»(«common.jointIdentifier(joint)») = «vars.force(link)»(«common.spatialVectorIndex(joint)»);
@   if (parent ~= robot.base) or robot.isFloatingBase then
«vars.force(parent)» += «common.parent_XF_link(link,self.members.xt)» * «vars.force(link)»;
@   end

@end]]

local fixed_base_pass1_G = [[
@for name,link in sorted_links() do
// Link '«name»'
@   local parent   = robot.treeutils.parent(link)
@   local child_X_parent = common.link_XM_parent(link, self.members.xt)
@   if (parent == robot.base) and not robot.isFloatingBase then
«vars.acc(link)» = («child_X_parent»).matrix().col(«ns_iit_rbd.qualifier»::LZ) * «ns_iit_rbd.qualifier»::g;
@   else
«vars.acc(link)» = («child_X_parent») * «vars.acc(parent)»;
@   end
«vars.force(link)» = «vars.I(link)» * «vars.acc(link)»;

@end]]


local fixed_base_pass1_C = [[
@for name,link in sorted_links() do
@   local parent   = robot.treeutils.parent(link)
@   local joint    = robot.treeutils.supportingJoint(link)
@   local velocity = vars.vel(link)
@   local acceler  = vars.acc(link)
@   local force    = vars.force(link)
@   local inertia  = vars.I(link)
@   local child_X_parent = child_mx_parent(link)
@   local jid      = common.jointIdentifier(joint)
@   local idx      = common.spatialVectorIndex(joint)
// Link '«name»'
@   if (parent == robot.base) and not robot.isFloatingBase then
«velocity»(«idx») = «self.params.qd»(«jid»);   // «velocity» = vJ, for the first link of a fixed base robot
@       if joint.kind == RCG.enums.JointKind.prismatic then
«force».setZero();  // first joint is prismatic, no centripetal terms
@       else
«force» = vxIv(«self.params.qd»(«jid»), «inertia»);
@       end
@   else
«velocity» = «child_X_parent» * «vars.vel(parent)»;
«velocity»(«idx») += «self.params.qd»(«jid»);
motionCrossProductMx<«types.scalar»>(«velocity», vcross);

@ -- Both children of floating bases and grandsons of fixed bases do not have
@ -- acceleration of their parent
@       if ((parent==robot.base) and robot.isFloatingBase) or (robot.treeutils.parent(parent)==robot.base and not robot.isFloatingBase) then
«acceler» = vcross.col(«idx») * «self.params.qd»(«jid»);
@       else
«acceler» = «child_X_parent» * «vars.acc(parent)» + vcross.col(«idx») * «self.params.qd»(«jid»);
@       end
«force» = «inertia» * «acceler» + vxIv(«velocity», «inertia»);
@   end

@end]]


local floating_base_methods_definitions = [[
@local qualifier = ns.qualifier .. '::' .. tpl.class.in_qualifier
«tpl.heading»
void «qualifier»::«self.class»::G_terms_fully_actuated(«self.fparam.basef», «self.fparam.tau»,
        «self.fparam.g»)
{
    using namespace «ns_iit_rbd.qualifier»;
    const Acceleration «vars.acc(robot.base)»{-«self.params.g»};
    «vars.force(robot.base)» = «vars.I(robot.base)» * «vars.acc(robot.base)»;

    ${fixed_base_pass1_G}

    sweep_inwards_fully_actuated(«self.params.tau»);

    «self.params.basef» = «vars.force(robot.base)»;
}

«tpl.heading»
void «qualifier»::«self.class»::C_terms_fully_actuated(«self.fparam.basef», «self.fparam.tau», «self.fparam.basev», «self.fparam.qd»)
{
    using namespace «ns_iit_rbd.qualifier»;
    «vars.force(robot.base)» = vxIv(«self.params.basev», «vars.I(robot.base)»);

    ${fixed_base_pass1_C}

    sweep_inwards_fully_actuated(«self.params.tau»);

    «self.params.basef» = «vars.force(robot.base)»;
}

«tpl.heading»
void «qualifier»::«self.class»::id_fully_actuated(
    «self.fparam.basef», «self.fparam.tau»,
    «self.fparam.g», «self.fparam.basev», «self.fparam.basea_in»,
    «self.fparam.qd», «self.fparam.qdd»,
    «self.fparam.fext» /*= zeroExtForces*/)
{
    using namespace «ns_iit_rbd.qualifier»;
    Acceleration «vars.acc(robot.base)» = «self.params.basea_in» - «self.params.g»;
    «vars.force(robot.base)» = «vars.I(robot.base)» * «vars.acc(robot.base)» + vxIv(«self.params.basev», «vars.I(robot.base)») - «self.params.fext»[«common.linkIdentifier(robot.base)»];

    ${fb_pass1_fully_actuated}

    sweep_inwards_fully_actuated(«self.params.tau»);

    «self.params.basef» = «vars.force(robot.base)»;
}

«tpl.heading»
void «qualifier»::«self.class»::sweep_inwards_fully_actuated(«self.fparam.tau»)
{
    using namespace «ns_iit_rbd.qualifier»;
    ${fixed_base_pass2}
}

«tpl.heading»
void «qualifier»::«self.class»::id(
    «self.fparam.tau», «self.fparam.basea»,
    «self.fparam.g», «self.fparam.basev»,
    «self.fparam.qd», «self.fparam.qdd»,
    «self.fparam.fext» /*= zeroExtForces*/)
{
    using namespace «ns_iit_rbd.qualifier»;
    «vars.force(robot.base)» = vxIv(«self.params.basev», «vars.I(robot.base)») - «self.params.fext»[«common.linkIdentifier(robot.base)»];

    ${fb_pass1_hybrid_dynamics}

    // Second pass //
    // ----------- //
    // propagate inwards wrenches and composite inertia

    InertiaMatrix Ic_aux;
    // initialize the composite inertias
@for _,link in sorted_links(robot.isFloatingBase) do
@   if not robot.treeutils.isLeaf(link) then
    «vars.Ic(link)» = «vars.I(link)»;
@   end
@end

@for name,link in sorted_links_reversed() do
    // Link '«name»'
    @   local parent   = robot.treeutils.parent(link)
    «ns_iit_rbd.qualifier»::transformInertia<«types.scalar»>(«vars.Ic(link)», «common.link_CT_parent(link,self.members.xt)».ct, Ic_aux);
    «vars.Ic(parent)» += Ic_aux;
    «vars.force(parent)» += «common.parent_XF_link(link,self.members.xt)» * «vars.force(link)»;

@end

    // The base acceleration due to the force due to the motion of the links
    «vars.acc(robot.base)» = - «vars.Ic(robot.base)».inverse() * «vars.force(robot.base)»;

    // Third pass //
    // ---------- //
    // propagate outwards the base acceleration and get the joint torques

@for name,link in sorted_links() do
@   local parent   = robot.treeutils.parent(link)
@   local joint    = robot.treeutils.supportingJoint(link)
@   local idx      = common.spatialVectorIndex(joint)
    «vars.acc(link)» = «child_mx_parent(link)» * «vars.acc(parent)»;
    «self.params.tau»(«common.jointIdentifier(joint)») = («vars.Ic(link)».row(«idx») * «vars.acc(link)» + «vars.force(link)»(«idx»));

@end

    «vars.acc(robot.base)» += «self.params.g»;
}
]]

local function generators_inverse_dynamics(robot, configurator, given_env)
    -- shallow copy the template environment, then add the fields required for
    -- the local templates
    local env = {}
    for k,v in pairs(given_env) do  env[k] = v  end

    local t_jstate = env.types.classScopeAliases.jointState
    local self = configurator.txtCfg.meta.inverse_dynamics

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
        qdd = "const " .. t_jstate .. "& " .. self.params.qdd,
        tau = t_jstate .. "& " .. self.params.tau,
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
    env.D = function(link) return 'D_' .. link.name end
    env.child_mx_parent = function(link)
        return env.common.link_XM_parent(link, self.members.xt)
    end

    local tpleval = RCG.utils.templates.tpl_eval

    return {
        header = function()
            env.include_guard = env.includeGuard(configurator.files.h_inv_dyn)
            return tpleval(header_template, env)
        end,
        source = function()
            local _, fixed_base_pass1_G = tpleval(fixed_base_pass1_G, env, {returnTable=true})
            local _, fixed_base_pass1_C = tpleval(fixed_base_pass1_C, env, {returnTable=true})
            local _, fixed_base_pass2 = tpleval(fixed_base_pass2, env, {returnTable=true})
            env.fixed_base_pass2 = fixed_base_pass2
            env.fixed_base_pass1_G = fixed_base_pass1_G
            env.fixed_base_pass1_C = fixed_base_pass1_C

            local ok, text
            if robot.isFloatingBase then
                env.hybridDynamics = true
                ok, text = tpleval(fixed_base_pass1, env, {returnTable=true})
                env.fb_pass1_hybrid_dynamics = text

                env.hybridDynamics = false
                ok, text = tpleval(fixed_base_pass1, env, {returnTable=true})
                env.fb_pass1_fully_actuated = text

                ok, text = tpleval(floating_base_methods_definitions, env, {returnTable=true})
                env.floating_base_methods_definitions = text
            else
                ok, text = tpleval(fixed_base_pass1, env, {returnTable=true})
                env.fixed_base_pass1 = text

                ok, text = tpleval(fixed_base_methods_definitions,    env, {returnTable=true})
                env.fixed_base_methods_definitions = text
            end

            ok, text = tpleval(source_template, env)
            return ok, text
        end
    }
end



return generators_inverse_dynamics
