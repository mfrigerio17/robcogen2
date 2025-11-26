-- local-ize the expected global variables
local GLOB = generators


local function class_code_meta(robot, configurator, env)
    local types = configurator.txtCfg.types

    local function getter_name_func_factory(quantityKind)
        return function(link)
            return "get" .. quantityKind .. "_" .. link.name
        end
    end

    local meta = {
        class = 'InverseDynamics', --TODO read from config
        members = {
            ip = 'ip',
            xt = 'xt',
            jsim_inverse = 'jsim_inverse'
        },
        getters = {
            force = getter_name_func_factory("Force"),
            vel   = getter_name_func_factory("Velocity"),
            acc   = getter_name_func_factory("Acceleration"),
        },
        params = {
            q= "q", qd= "qd", qdd= "qdd", tau= "tau",

            -- Base v/a cannot be chosen freely, they must match the pattern
            -- configured for the robot links
            basev= configurator.txtCfg.vars.vel(robot.base),
            basea= configurator.txtCfg.vars.acc(robot.base),

            basea_in= "base_a", basef= "base_f", g= "gravity",
            fext= "fext"
        },
        local_types = {
            fext = types.externalForces,
        },
        other_classes = {
            inertia    = GLOB.inertia.meta(robot, configurator, env).inertia_properties,
            transforms = {
                class = env.common.transformsContainerMeta.class_name,
                members = env.common.transformsContainerMeta.members,
            },
        },
    }
    meta.fparam = {
        q   = "const " .. types.jointState .. "& " .. meta.params.q,
        qd  = "const " .. types.jointState .. "& " .. meta.params.qd,
        qdd = "const " .. types.jointState .. "& " .. meta.params.qdd,
        tau = types.jointState .. "& " .. meta.params.tau,
        basev = "const Velocity& " .. meta.params.basev,
        basea = "Acceleration& " .. meta.params.basea,
        basea_in = "const Acceleration& " .. meta.params.basea_in,
        basef = "Force& " .. meta.params.basef,
        g = "const Acceleration& " .. meta.params.g,
        fext = "const " .. meta.local_types.fext .. "& " .. meta.params.fext,
    }
    return meta
end



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
struct «meta.class»
{
@if templateAll then
«typesMacro»
using «types.jointState» = «types.jointState»«tpl.suffix»;

@end
    using «meta.local_types.fext» = ::rcg2::DataMap<Force, linksCount, «types.linkIDs»>;

    /**
     * Default constructor
     * \param ip the inertia properties of the links
     * \param xt the container of all the spatial motion transforms of
     *     the robot «robot.name», which will be used by this instance
     *     to compute inverse-dynamics.
     */
    «meta.class»(const «meta.other_classes.inertia.class»«tpl.suffix»& ip, «meta.other_classes.transforms.class»«tpl.suffix»& xt);

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const «types.jointState»& q) {
        «meta.members.xt».«meta.other_classes.transforms.members.update»(q);
    }

@if robot.isFloatingBase then
    /** \name Inverse dynamics
     * The full algorithm for the inverse dynamics of this robot.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] «meta.params.tau» the joint force vector required to achieve the desired accelerations
     * \param[out] «meta.params.basea» the spatial acceleration of the robot base
     * \param[in] «meta.params.g» the gravity acceleration, as a spatial vector;
     *      gravity implicitly specifies the orientation of the base in space
     * \param[in] «meta.params.basev» the spatial velocity of the base
     * \param[in] «meta.params.q» the joint position vector
     * \param[in] «meta.params.qd» the joint velocity vector
     * \param[in] «meta.params.qdd» the desired joint acceleration vector
     * \param[in] «meta.params.fext» the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id(
        «meta.fparam.tau», «meta.fparam.basea»,
        «meta.fparam.g», «meta.fparam.basev»,
        «meta.fparam.q», «meta.fparam.qd», «meta.fparam.qdd»,
        «meta.fparam.fext» = zeroExtForces)
    {
        setJointStatus(«meta.params.q»);
        id(«meta.params.tau», «meta.params.basea», «meta.params.g», «meta.params.basev», «meta.params.qd», «meta.params.qdd», «meta.params.fext»);
    }
    void id(
        «meta.fparam.tau», «meta.fparam.basea»,
        «meta.fparam.g», «meta.fparam.basev»,
        «meta.fparam.qd», «meta.fparam.qdd»,
        «meta.fparam.fext» = zeroExtForces);
    ///@}
    /** \name Inverse dynamics, fully actuated base
     * The inverse dynamics algorithm for the floating base robot,
     * in the assumption of a fully actuated base.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] «meta.params.basef» the spatial force to be applied to
     *   the robot base to achieve the desired accelerations
     * \param[out] «meta.params.tau» the joint force vector required to achieve the desired accelerations
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] «meta.params.basev» the spatial velocity of the base
     * \param[in] «meta.params.basea» the desired spatial acceleration of the robot base
     * \param[in] «meta.params.q» the joint position vector
     * \param[in] «meta.params.qd» the joint velocity vector
     * \param[in] «meta.params.qdd» the desired joint acceleration vector
     * \param[in] «meta.fparam.fext» the external forces acting on the links; this parameter
     *            defaults to zero
     */ ///@{
    void id_fully_actuated(
        «meta.fparam.basef», «meta.fparam.tau»,
        «meta.fparam.g», «meta.fparam.basev», «meta.fparam.basea_in»,
        «meta.fparam.q», «meta.fparam.qd», «meta.fparam.qdd»,
        «meta.fparam.fext» = zeroExtForces)
    {
        setJointStatus(«meta.params.q»);
        id_fully_actuated(«meta.params.basef», «meta.params.tau», «meta.params.g», «meta.params.basev»,
            «meta.params.basea_in», «meta.params.qd», «meta.params.qdd», fext);
    }
    void id_fully_actuated(
        «meta.fparam.basef», «meta.fparam.tau»,
        «meta.fparam.g», «meta.fparam.basev», «meta.fparam.basea_in»,
        «meta.fparam.qd», «meta.fparam.qdd»,
        «meta.fparam.fext» = zeroExtForces);
    ///@}

    /** \name Gravity terms, fully actuated base
     */
    ///@{
    void G_terms_fully_actuated(
        «meta.fparam.basef», «meta.fparam.tau»,
        «meta.fparam.g», «meta.fparam.q»)
    {
        setJointStatus(«meta.params.q»);
        G_terms_fully_actuated(«meta.params.basef», «meta.params.tau», «meta.params.g»);
    }
    void G_terms_fully_actuated(
        «meta.fparam.basef», «meta.fparam.tau»,
        «meta.fparam.g»);
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
        «meta.fparam.basef», «meta.fparam.tau»,
        «meta.fparam.basev», «meta.fparam.q», «meta.fparam.qd»)
    {
        setJointStatus(«meta.params.q»);
        C_terms_fully_actuated(«meta.params.basef», «meta.params.tau», «meta.params.basev», qd);
    }
    void C_terms_fully_actuated(
        «meta.fparam.basef», «meta.fparam.tau»,
        «meta.fparam.basev», «meta.fparam.qd»);
    ///@}
@ else
    /** \name Inverse dynamics
     * The full Newton-Euler algorithm for the inverse dynamics of this robot.
     *
     * \param[out] «meta.params.tau» the joint force vector required to achieve the desired accelerations
     * \param[in] «meta.params.q» the joint position vector
     * \param[in] «meta.params.qd» the joint velocity vector
     * \param[in] «meta.params.qdd» the desired joint acceleration vector
     * \param[in] «meta.fparam.fext» the external forces acting on the links; this parameters
     *            defaults to zero
     */
    ///@{
    void id(
        «meta.fparam.tau»,
        «meta.fparam.q», «meta.fparam.qd», «meta.fparam.qdd»,
        «meta.fparam.fext» = zeroExtForces)
    {
        setJointStatus(«meta.params.q»);
        id(«meta.params.tau», «meta.params.qd», «meta.params.qdd», «meta.params.fext»);
    }

    void id(
        «meta.fparam.tau»,
        «meta.fparam.qd», «meta.fparam.qdd»,
        «meta.fparam.fext» = zeroExtForces)
    {
        firstPass(«meta.params.qd», «meta.params.qdd», «meta.params.fext»);
        secondPass(«meta.params.tau»);
    }
    ///@}

    /** \name Gravity terms
     * The joint forces (linear or rotational) required to compensate
     * for the effect of gravity, in a specific configuration.
     */
    ///@{
    void G_terms(«meta.fparam.tau», «meta.fparam.q») {
        setJointStatus(«meta.params.q»);
        G_terms(«meta.params.tau»);
    }
    void G_terms(«meta.fparam.tau»);
    ///@}

    /** \name Centrifugal and Coriolis terms
     * The forces (linear or rotational) acting on the joints due to centrifugal and
     * Coriolis effects, for a specific configuration.
     */
    ///@{
    void C_terms(«meta.fparam.tau», «meta.fparam.q», «meta.fparam.qd») {
        setJointStatus(«meta.params.q»);
        C_terms(«meta.params.tau», «meta.params.qd»);
    }
    void C_terms(«meta.fparam.tau», «meta.fparam.qd»);
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
@for name,link in sorted_links(robot) do
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
@   for _,link in sorted_links(robot) do
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
    void sweep_inwards_fully_actuated(«meta.fparam.tau»);
@ else
    void firstPass(«meta.fparam.qd», «meta.fparam.qdd», «meta.fparam.fext»);
    void secondPass(«meta.fparam.tau»);
@ end

private:
    «meta.other_classes.transforms.class»& «meta.members.xt»;
    Matrix66 vcross; // support variable

private:
    static const «meta.local_types.fext» zeroExtForces;
};


${ns.close}

#endif
]]

local source_template = [[
#include <iit/rbd/robcogen_commons.h>

#include "«headers.inv_dyn»"

using namespace std;
using namespace «ns_iit_rbd.qualifier»;

@local qualifier = ns.qualifier .. '::' .. tpl.class.in_qualifier

// Initialization of static-const data
const «qualifier»::«meta.local_types.fext»
«qualifier»::zeroExtForces(Force::Zero());

«qualifier»::«meta.class»(const «meta.other_classes.inertia.class»& inertia, «meta.other_classes.transforms.class»& transforms) :
    // the local aliases for the inertia tensors:
@ for  _,link,comma in sorted_links(robot) do
    «vars.I(link)»( inertia.«meta.other_classes.inertia.members.tensorGetter(link)»() ),
@end
@if robot.isFloatingBase then
    «vars.I(robot.base)»( inertia.«meta.other_classes.inertia.members.tensorGetter(robot.base)»() ),
    // the composite inertia of leaf links IS the regular inertia
@   for _,link in sorted_links(robot) do
@       if robot.treeutils.isLeaf(link) then
    «vars.Ic(link)»(«vars.I(link)»),
@       end
@   end
@end
    «meta.members.xt»(transforms)
{
@for name,link in sorted_links(robot) do
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

void «qualifier»::«meta.class»::G_terms(«meta.fparam.tau»)
{
    ${fixed_base_pass1_G}

    secondPass(«meta.params.tau»);
}

void «qualifier»::«meta.class»::C_terms(«meta.fparam.tau», «meta.fparam.qd»)
{
    ${fixed_base_pass1_C}

    secondPass(«meta.params.tau»);
}


void «qualifier»::«meta.class»::firstPass(«meta.fparam.qd», «meta.fparam.qdd», «meta.fparam.fext»)
{
    ${fixed_base_pass1}
}

void «qualifier»::«meta.class»::secondPass(«meta.fparam.tau»)
{
    ${fixed_base_pass2}
}
]]


local fixed_base_pass1 = [[
@for name,link in sorted_links(robot) do
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
«velocity»(«idx») += «meta.params.qd»(«jid»);

motionCrossProductMx<«types.scalar»>(«velocity», vcross);

«acceler» = «child_X_parent» * «vars.acc(parent)» + vcross.col(«idx») * «meta.params.qd»(«jid»);
«acceler»(«idx») += «meta.params.qdd»(«jid»);

«vars.force(link)» = «vars.I(link)» * «acceler» + vxIv(«velocity», «vars.I(link)») - «meta.params.fext»[«common.linkIdentifier(link)»];
@   --
@   elseif not robot.isFloatingBase then -- parent IS the fixed-base
@   --
«velocity»(«idx») = «meta.params.qd»(«jid»);   // «velocity» = vJ, for the first link of a fixed base robot
«acceler» = «child_X_parent».matrix().col(LZ) * «ns_iit_rbd.qualifier»::g;
«acceler»(«idx») += «meta.params.qdd»(«jid»);
@       if joint.kind == RCG.enums.JointKind.prismatic then
// The first joint is prismatic, no centripetal terms.
«vars.force(link)» = «vars.I(link)» * «acceler» - «meta.params.fext»[«common.linkIdentifier(link)»];
@       else
«vars.force(link)» = «vars.I(link)» * «acceler» + vxIv(«meta.params.qd»(«jid»), «vars.I(link)») - «meta.params.fext»[«common.linkIdentifier(link)»];
@       end
@   --
@   else -- parent IS the floating-base and we are coding hybrid dynamics
@   --
«velocity» = «child_X_parent» * «vars.vel(parent)»;
«velocity»(«idx») += «meta.params.qd»(«jid»);

motionCrossProductMx<«types.scalar»>(«velocity», vcross);

«acceler» = vcross.col(«idx») * «meta.params.qd»(«jid»);
«acceler»(«idx») += «meta.params.qdd»(«jid»);

«vars.force(link)» = «vars.I(link)» * «acceler» + vxIv(«velocity», «vars.I(link)») - «meta.params.fext»[«common.linkIdentifier(link)»];
@   end

@end
]]


local fixed_base_pass2 = [[
@for name,link in sorted_links_reversed(robot) do
// Link '«name»'
@   local parent   = robot.treeutils.parent(link)
@   local joint    = robot.treeutils.supportingJoint(link)
«meta.params.tau»(«common.jointIdentifier(joint)») = «vars.force(link)»(«common.spatialVectorIndex(joint)»);
@   if (parent ~= robot.base) or robot.isFloatingBase then
«vars.force(parent)» += «common.parent_XF_link(link,meta.members.xt)» * «vars.force(link)»;
@   end

@end]]

local fixed_base_pass1_G = [[
@for name,link in sorted_links(robot) do
// Link '«name»'
@   local parent   = robot.treeutils.parent(link)
@   local child_X_parent = common.link_XM_parent(link, meta.members.xt)
@   if (parent == robot.base) and not robot.isFloatingBase then
«vars.acc(link)» = («child_X_parent»).matrix().col(«ns_iit_rbd.qualifier»::LZ) * «ns_iit_rbd.qualifier»::g;
@   else
«vars.acc(link)» = («child_X_parent») * «vars.acc(parent)»;
@   end
«vars.force(link)» = «vars.I(link)» * «vars.acc(link)»;

@end]]


local fixed_base_pass1_C = [[
@for name,link in sorted_links(robot) do
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
«velocity»(«idx») = «meta.params.qd»(«jid»);   // «velocity» = vJ, for the first link of a fixed base robot
@       if joint.kind == RCG.enums.JointKind.prismatic then
«force».setZero();  // first joint is prismatic, no centripetal terms
@       else
«force» = vxIv(«meta.params.qd»(«jid»), «inertia»);
@       end
@   else
«velocity» = «child_X_parent» * «vars.vel(parent)»;
«velocity»(«idx») += «meta.params.qd»(«jid»);
motionCrossProductMx<«types.scalar»>(«velocity», vcross);

@ -- Both children of floating bases and grandsons of fixed bases do not have
@ -- acceleration of their parent
@       if ((parent==robot.base) and robot.isFloatingBase) or (robot.treeutils.parent(parent)==robot.base and not robot.isFloatingBase) then
«acceler» = vcross.col(«idx») * «meta.params.qd»(«jid»);
@       else
«acceler» = «child_X_parent» * «vars.acc(parent)» + vcross.col(«idx») * «meta.params.qd»(«jid»);
@       end
«force» = «inertia» * «acceler» + vxIv(«velocity», «inertia»);
@   end

@end]]


local floating_base_methods_definitions = [[
@local qualifier = ns.qualifier .. '::' .. tpl.class.in_qualifier

void «qualifier»::«meta.class»::G_terms_fully_actuated(«meta.fparam.basef», «meta.fparam.tau»,
        «meta.fparam.g»)
{
    const Acceleration «vars.acc(robot.base)»{-«meta.params.g»};
    «vars.force(robot.base)» = «vars.I(robot.base)» * «vars.acc(robot.base)»;

    ${fixed_base_pass1_G}

    sweep_inwards_fully_actuated(«meta.params.tau»);

    «meta.params.basef» = «vars.force(robot.base)»;
}

void «qualifier»::«meta.class»::C_terms_fully_actuated(«meta.fparam.basef», «meta.fparam.tau», «meta.fparam.basev», «meta.fparam.qd»)
{
    «vars.force(robot.base)» = vxIv(«meta.params.basev», «vars.I(robot.base)»);

    ${fixed_base_pass1_C}

    sweep_inwards_fully_actuated(«meta.params.tau»);

    «meta.params.basef» = «vars.force(robot.base)»;
}

void «qualifier»::«meta.class»::id_fully_actuated(
    «meta.fparam.basef», «meta.fparam.tau»,
    «meta.fparam.g», «meta.fparam.basev», «meta.fparam.basea_in»,
    «meta.fparam.qd», «meta.fparam.qdd»,
    «meta.fparam.fext» /*= zeroExtForces*/)
{
    Acceleration «vars.acc(robot.base)» = «meta.params.basea_in» - «meta.params.g»;
    «vars.force(robot.base)» = «vars.I(robot.base)» * «vars.acc(robot.base)» + vxIv(«meta.params.basev», «vars.I(robot.base)») - «meta.params.fext»[«common.linkIdentifier(robot.base)»];

    ${fb_pass1_fully_actuated}

    sweep_inwards_fully_actuated(«meta.params.tau»);

    «meta.params.basef» = «vars.force(robot.base)»;
}

void «qualifier»::«meta.class»::sweep_inwards_fully_actuated(«meta.fparam.tau»)
{
    ${fixed_base_pass2}
}

void «qualifier»::«meta.class»::id(
    «meta.fparam.tau», «meta.fparam.basea»,
    «meta.fparam.g», «meta.fparam.basev»,
    «meta.fparam.qd», «meta.fparam.qdd»,
    «meta.fparam.fext» /*= zeroExtForces*/)
{
    «vars.force(robot.base)» = vxIv(«meta.params.basev», «vars.I(robot.base)») - «meta.params.fext»[«common.linkIdentifier(robot.base)»];

    ${fb_pass1_hybrid_dynamics}

    // Second pass //
    // ----------- //
    // propagate inwards wrenches and composite inertia

    InertiaMatrix Ic_aux;
    // initialize the composite inertias
@for _,link in sorted_links(robot, "include_base_if_floating") do
@   if not robot.treeutils.isLeaf(link) then
    «vars.Ic(link)» = «vars.I(link)»;
@   end
@end

@for name,link in sorted_links_reversed(robot) do
    // Link '«name»'
    @   local parent   = robot.treeutils.parent(link)
    «ns_iit_rbd.qualifier»::transformInertia<«types.scalar»>(«vars.Ic(link)», «common.link_CT_parent(link,meta.members.xt)».ct, Ic_aux);
    «vars.Ic(parent)» += Ic_aux;
    «vars.force(parent)» += «common.parent_XF_link(link,meta.members.xt)» * «vars.force(link)»;

@end

    // The base acceleration due to the force due to the motion of the links
    «vars.acc(robot.base)» = - «vars.Ic(robot.base)».inverse() * «vars.force(robot.base)»;

    // Third pass //
    // ---------- //
    // propagate outwards the base acceleration and get the joint torques

@for name,link in sorted_links(robot) do
@   local parent   = robot.treeutils.parent(link)
@   local joint    = robot.treeutils.supportingJoint(link)
@   local idx      = common.spatialVectorIndex(joint)
    «vars.acc(link)» = «child_mx_parent(link)» * «vars.acc(parent)»;
    «meta.params.tau»(«common.jointIdentifier(joint)») = («vars.Ic(link)».row(«idx») * «vars.acc(link)» + «vars.force(link)»(«idx»));

@end

    «vars.acc(robot.base)» += «meta.params.g»;
}
]]

local function id_generators(robot, configurator, given_env)
    -- shallow copy the template environment, then add the fields required for
    -- the local templates
    local env = {}
    for k,v in pairs(given_env) do  env[k] = v  end

    env.meta  = class_code_meta(robot, configurator, env)
    env.include_guard = env.includeGuard(configurator.files.h_inv_dyn)
    env.tpl   = env.common.scalarTpl( env.meta.class, env.templateAll )
    env.D = function(link) return 'D_' .. link.name end
    env.child_mx_parent = function(link)
        return env.common.link_XM_parent(link, env.meta.members.xt)
    end

    local tpleval = RCG.utils.templates.tpl_eval

    return {
        header = function() return tpleval(header_template, env) end,
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

            return tpleval(source_template, env)
        end
    }
end



GLOB.id = {
    generators = id_generators,
    meta = meta,
}
