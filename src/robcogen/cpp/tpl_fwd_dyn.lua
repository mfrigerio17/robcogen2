

local function meta(robot, configurator, env)
    return {
        class = 'ForwardDynamics', --TODO read from config
        members = {
            ip = 'ip',
            xt = 'xt',
            jsim_inverse = 'jsim_inverse'
        }
    }
end

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

«tpl_help.heading»
class «meta.class»
{
@if templateAll then
«typesMacro»
@end
public:
    using Hinv_t = Matrix<JointSpaceDimension,JointSpaceDimension>;
@if robot.isFloatingBase then
    using Finv_t = Matrix<6, JointSpaceDimension>;
@end
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot «robot.name», which will be used by this instance
     *     to compute the dynamics.
     */
    «meta.class»(const «meta.inertia.class»«tpl_help.suffix»&, «meta.transforms.class»«tpl_help.suffix»&);

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const «types.jointState»«tpl_help.suffix»& q) {
        «meta.members.xt».update(q);
    }

@if robot.isFloatingBase then
    void «meta.members.jsim_inverse»(const «types.jointState»«tpl_help.suffix»&, Matrix66&, Finv_t&, Hinv_t&);
@else
    void «meta.members.jsim_inverse»(const «types.jointState»«tpl_help.suffix»&, Hinv_t&);
@end

public:
    const «meta.inertia.class»«tpl_help.suffix»& «meta.members.ip»;
    «meta.transforms.class»«tpl_help.suffix»& «meta.members.xt»;

    //Matrix66 vcross; // support variable
    
    // support variable for the propagation of articulated inertia
    // set to zero once, here; the zero coefficients are never touched, in the algorithms 
@if  robot.hasPrismaticJoint then
    Matrix66 aux_Ia_p = Matrix66::«mxops.zeroMx»();   // for prismatic joint
@end
@if  robot.hasRevoluteJoint then
    Matrix66 aux_Ia_r = Matrix66::«mxops.zeroMx»();   // for revolute joint
@end

@for i,l in ipairs(links) do
    // Link '«l.name»' :
@   if tree.isLeaf(l) then
    const InertiaMatrix& «vars.IA(l)» = «meta.members.ip».«meta.inertia.members.tensorGetter(l)»();//
@   else
    Matrix66 «vars.IA(l)»;
@   end
    Velocity «vars.acc(l)»;
    Velocity «vars.vel(l)»;
    Velocity «vars.biasA(l)»;
    Force    «vars.biasF(l)»;
    Force    «vars.T(l)»;
@   if not same(l, robot.base) then
    const «types.scalar»& «D(l)»;
@   end
@end

};

${ns.close}

@if templateAll then
#include "«impl_files.fwd_dyn»"
@end

#endif
]]


local impl_template = [[
@if not templateAll then
#include "«headers.fwd_dyn»"
@end

@local qualifier = ns.qualifier .. '::' .. tpl_help.class.in_qualifier
«tpl_help.heading»
«qualifier»::«meta.class»(const «meta.inertia.class»«tpl_help.suffix»& ip, «meta.transforms.class»«tpl_help.suffix»& tf)
  : «meta.members.ip»(ip), «meta.members.xt»(tf)
@for i,link in ipairs(links) do
@   if not same(link, robot.base) then
@       local joint = robot.treeutils.supportingJoint(link)
@       local jx    = common.spatialVectorIndex(joint)
    , «D(link)»(«vars.IA(link)»(«ns_iit_rbd.qualifier»::«jx»,«ns_iit_rbd.qualifier»::«jx»))
@   end
@end
{
@for i,l in ipairs(links) do
    «vars.vel(l)».«mxops.setzero»();
    «vars.biasA(l)».«mxops.setzero»();
@end
}

«tpl_help.heading»
@if robot.isFloatingBase then
void «qualifier»::«meta.class»::«meta.members.jsim_inverse»(const «types.jointState»«tpl_help.suffix»& q, Matrix66& phi0, Finv_t& Finv, Hinv_t& Hi)
@else
void «qualifier»::«meta.class»::«meta.members.jsim_inverse»(const «types.jointState»«tpl_help.suffix»& q, Hinv_t& Hi)
@end
{
    using namespace «ns_iit_rbd.qualifier»;
@for i,l in ipairs(links) do
@   if not tree.isLeaf(l) then
    «vars.IA(l)» = «meta.members.ip».«meta.inertia.members.tensorGetter(l)»();
@   end
@end

    // Inward sweep for articulated inertia //

    Matrix66 IaB;
    Column6 U;
@local link, joint, parent, ancestor, jid, lid, idx
@for i = #links, 2, -1 do
@   link  = links[i]
@   joint = tree.supportingJoint(link)
@   idx   = common.spatialVectorIndex(joint)
@   parent= tree.parent(link)
@   local IA = vars.IA(link)
@   local child_X_parent = common.link_CT_parent(link, meta.members.xt)

    @if (not robot.isBase(parent)) or robot.isFloatingBase then
    «child_X_parent»(q);
    U = «IA».«mxops.col»(«idx»);
    «vars.T(link)» = U / «D(link)»;
        @if joint.kind == RCG.enums.JointKind.prismatic then
    propagate_IA_across_prismatic_joint(«IA», aux_Ia_p);
    ctransform_Ia_prismatic(aux_Ia_p, «child_X_parent».ct, IaB);
        @else
    propagate_IA_across_revolute_joint(«IA», aux_Ia_r);
    ctransform_Ia_revolute(aux_Ia_r, «child_X_parent».ct, IaB);
        @end
    «vars.IA(parent)» += IaB;
    @end

@end

@if robot.isFloatingBase then
    phi0 = «vars.IA(robot.base)».inverse();//TODO use specialized routine

@end
    // Inward sweep for the bias force //

    Force pIn;
    «types.scalar» aux;
@for i = #links, 2, -1 do
@   link    = links[i]
@   lid     = common.linkIdentifier(link)
@   ancestor= robot.treeutils.parent(link)
@   joint   = robot.treeutils.supportingJoint(link)
@   jid     = common.jointIdentifier(joint)
    //
    // Joint `«jid»` . . .
    //
    pIn = «common.parent_XF_link(link, meta.members.xt)» * «vars.T(link)»;
@   local lastlink = link
@   while not robot.isBase(ancestor) do
@       local joint2 = tree.supportingJoint(ancestor)
@       local jid2   = common.jointIdentifier(joint2)
@       local idx    = common.spatialVectorIndex(joint2)
@       local parent_XF_link = common.parent_XF_link(ancestor, meta.members.xt)
    //    . . . on joint `«jid2»`
    aux = - pIn(«idx») / «D(ancestor)»;
    Hi(«jid2»,«jid») = Hi(«jid»,«jid2») = aux;
@       parent = tree.parent(ancestor)
@       if not robot.isBase( parent ) or robot.isFloatingBase then
    pIn = «parent_XF_link» * ( pIn + «vars.IA(ancestor)».«mxops.col»(«idx») * aux );
@       end
@       lastlink = ancestor
@       ancestor = parent  -- goes up the kinematic chain
@   end
@   if robot.isFloatingBase then
@       local base_XF_link = common.parent_XF_link(lastlink, meta.members.xt)
@       local ab_var = varname_abounce(jid)
    Acceleration «ab_var» = phi0 * pIn;  // a_|«robot.base.name»  ↶ «jid»|
    Finv.«mxops.col»(«jid») = - «ab_var»;
@   else
    «types.scalar» «varname_abounce_top(jid)» = -aux; // a_|«lid»  ↶ «jid»| (a scalar, first link is special case)
@   end

@end

@if not robot.isFloatingBase then
    @link  = links[1]
    @joint = tree.supportingJoint(link)
    @jid   = common.jointIdentifier(joint)
    Hi(«jid»,«jid») = 1/«D(link)»; // special case, first moving body
@end

    // ------------------- //
    // Outward sweep start //
    // ------------------- //

@local j0_idx = common.spatialVectorIndex( robot.joints[1] )
@for i = 2, #links, 1 do
@   link  = links[i]
@   parent= tree.parent(link)
@   joint = tree.supportingJoint(link)
@   jid   = common.jointIdentifier(joint)
@   idx   = common.spatialVectorIndex(joint)
@   lid   = common.linkIdentifier(link)
@   local link_XM_parent = common.link_XM_parent(link, meta.members.xt)
@   local bouncing_from_root = robot.isBase( tree.parent(parent) )
@   local link_C_parent = link.name .. '_C_' .. parent.name
    //
    // Joint `«jid»` . . .
    //
@   if bouncing_from_root and not robot.isFloatingBase then
    auto «link_C_parent» = «link_XM_parent».matrix().«mxops.col»(«j0_idx»);
    Hi(«jid»,«jid») = 1/«D(link)» + «vars.T(link)».«mxops.T»() * («link_C_parent» * «varname_abounce_top(jid)»);
@   else
    Hi(«jid»,«jid») = 1/«D(link)» + «vars.T(link)».«mxops.T»() * («link_XM_parent» * «varname_abounce(jid)»);
@   end
@--
@   local link_j
@   for j = i+1, #links, 1 do
@       link_j = links[j]
@       joint  = tree.supportingJoint(link_j)
@       jid2   = common.jointIdentifier(joint)
@       local ab_help = abounce_helper(joint, link_j, link)

    //    . . . affected by `«jid2»`
@if robot.isFloatingBase then
@ if ab_help.first_off then
    Acceleration «ab_help.varname» = «link_XM_parent» * «ab_help.varname_prev»;
@ else
    «ab_help.varname» = «link_XM_parent» * «ab_help.varname»;
@ end
@else
@ if bouncing_from_root then
    Acceleration «ab_help.varname» = «link_C_parent» * «varname_abounce_top(jid2)»;
@ else
@  if ab_help.first_off then
    Acceleration «ab_help.varname» = «link_XM_parent» * «ab_help.varname_prev»;
@  else
    «ab_help.varname» = «link_XM_parent» * «ab_help.varname»;
@  end
@ end
@end
    Hi(«jid»,«jid2») = Hi(«jid»,«jid2») + «vars.T(link)».«mxops.T»() * «ab_help.varname»;
    Hi(«jid2»,«jid») = Hi(«jid»,«jid2»);

    «ab_help.varname»(«idx») -= Hi(«jid»,«jid2»);  // a_|«lid»  ↶ «jid2»|
@   end

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

    env.links = robot.movingLinks
    env.tree  = robot.treeutils
    env.same  = function(l1,l2) return l1==l2 end
    env.meta  = meta(robot, configurator, env)
    env.include_guard = env.includeGuard(configurator.files.h_fwd_dyn)
    env.tpl_help = env.common.scalarTpl( env.meta.class, env.templateAll )
    env.D = function(link) return 'D_' .. link.name end

    --
    -- note that we must use joint/link names as keys in the tables, because
    -- equality check of python objects is bugged in Lupa
    --
    local first_link = env.links[1]
    local firstLinkId = env.common.linkIdentifier( first_link )
    local abounce_vars = {}
    env.varname_abounce_top = function(jid) return 'a__' .. firstLinkId .. '_' .. jid end
    env.varname_abounce     = function(jid) return 'a_bounce_' .. jid end
    local varname_abounce_top = env.varname_abounce_top
    if robot.isFloatingBase then varname_abounce_top = env.varname_abounce end
    for i,joint in ipairs(robot.joints) do
        local jid = env.common.jointIdentifier(joint)
        abounce_vars[jid] = {
            own_chain    = env.varname_abounce(jid),
            [first_link.name] = varname_abounce_top(jid)
        }
    end
    env.abounce_helper = function(joint_N, link_N, link_n)
        local common_ancestor = env.tree.lowestCommonAncestor(link_n, link_N)
        local same_chain = env.same(common_ancestor, link_n)
        local first_off = nil
        local varname = nil
        local varname_prev = nil
        local parent_of_link_n = env.tree.parent(link_n)
        local jid = env.common.jointIdentifier(joint_N)
        if same_chain then
            varname = abounce_vars[jid].own_chain
            abounce_vars[jid][link_n.name] = varname
        else
            first_off = env.same(common_ancestor, parent_of_link_n)
            if first_off then
                varname = 'a__' .. jid .. '_via_' ..
                            env.common.linkIdentifier(common_ancestor) .. '_' ..
                            env.common.linkIdentifier(link_n)
                varname_prev = abounce_vars[jid][parent_of_link_n.name]
                --print(link_N, link_n, common_ancestor, varname, varname_prev)
                --print(parent_of_link_n.name)
                --for k,v in pairs(abounce_vars[jid]) do print(k,v) end
            else
                varname = abounce_vars[jid][parent_of_link_n.name]
            end
            abounce_vars[jid][link_n.name] = varname
        end
        --print(link_N, link_n, common_ancestor, same_chain, first_off, varname, varname_prev)
        return {
            common_ancestor = common_ancestor,
            same_chain = same_chain,
            first_off = first_off,
            varname = varname,
            varname_prev = varname_prev
        }
    end


    local aux = GLOB.inertia.meta(robot, configurator, given_env)
    env.meta.inertia = aux.inertia_properties
    env.meta.transforms = {
        class = env.classes.transforms
    }
    
    return {
        header = function() return genutils.tpl_eval(header_template, env) end,
        impl   = function() return genutils.tpl_eval(impl_template, env) end,
    }
end


GLOB.fd = {
    generators = fd_generators,
    meta = meta
}
