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

«tpl.heading»
struct «self.class»
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
    «self.class»(const «meta.inertia_properties.class»«tpl.suffix»&, «meta.transforms_container.class»«tpl.suffix»&);

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const «types.jointState»«tpl.suffix»& q) {
        «self.members.xt».update(q);
    }

@if robot.isFloatingBase then
    void «self.members.jsim_inverse»(const «types.jointState»«tpl.suffix»&, Matrix66&, Finv_t&, Hinv_t&);
@else
    void «self.members.jsim_inverse»(const «types.jointState»«tpl.suffix»&, Hinv_t&);
@end

public:
    const «meta.inertia_properties.class»«tpl.suffix»& «self.members.ip»;
    «meta.transforms_container.class»«tpl.suffix»& «self.members.xt»;

    mutable Matrix66 IaB;  // support variable

    // support variable for the propagation of articulated inertia
    // set to zero once, here; the zero coefficients are never touched, in the algorithms
@if  robot.hasPrismaticJoint then
    Matrix66 aux_Ia_p = Matrix66::«mxops.zeroMx»();   // for prismatic joint
@end
@if  robot.hasRevoluteJoint then
    Matrix66 aux_Ia_r = Matrix66::«mxops.zeroMx»();   // for revolute joint
@end

@for name,l in sorted_links() do
    // Link '«name»' :
@   if tree.isLeaf(l) then
    const InertiaMatrix& «vars.IA(l)»;
@   else
    Matrix66 «vars.IA(l)»;
@   end
    Velocity «vars.acc(l)»;
    Velocity «vars.vel(l)»;
    Velocity «vars.biasA(l)»;
    Force    «vars.biasF(l)»;
    Force    «vars.T(l)»;
@end

@for _,link in sorted_links() do
    const «types.scalar»& «D(link)»;
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
#include "«headers.aba_hinv»"
@end

@local qualifier = ns.qualifier .. '::' .. tpl.class.in_qualifier
«tpl.heading»
«qualifier»::«self.class»(const «meta.inertia_properties.class»«tpl.suffix»& ip, «meta.transforms_container.class»«tpl.suffix»& tf)
  : «self.members.ip»(ip), «self.members.xt»(tf),
@for _, link in ipairs(leafs) do
    «vars.IA(link)»(ip.«meta.inertia_properties.members.tensorGetter(link)»()),
@end
@for _, link, comma in utils.i_iterator_with_separator(sorted_links, ",") do
@   local jx  = common.spatialVectorIndex(robot.treeutils.supportingJoint(link))
    «D(link)»(«vars.IA(link)»(«ns_iit_rbd.qualifier»::«jx»,«ns_iit_rbd.qualifier»::«jx»))«comma»
@end
{
@for i,l in sorted_links() do
    «vars.vel(l)».«mxops.setzero»();
    «vars.biasA(l)».«mxops.setzero»();
@end
    IaB.setZero();
}

«tpl.heading»
@if robot.isFloatingBase then
void «qualifier»::«self.class»::«self.members.jsim_inverse»(const «types.jointState»«tpl.suffix»& q, Matrix66& phi0, Finv_t& Finv, Hinv_t& Hi)
@else
void «qualifier»::«self.class»::«self.members.jsim_inverse»(const «types.jointState»«tpl.suffix»& q, Hinv_t& Hi)
@end
{
    using namespace «ns_iit_rbd.qualifier»;
@for i,l in sorted_links() do
@   if not tree.isLeaf(l) then
    «vars.IA(l)» = «self.members.ip».«meta.inertia_properties.members.tensorGetter(l)»();
@   end
@end

    // Inward sweep for articulated inertia //

    Column6 U;
@local link, joint, parent, ancestor, jid, lid, idx
@for _,link in sorted_links_reversed() do
@   joint = tree.supportingJoint(link)
@   idx   = common.spatialVectorIndex(joint)
@   parent= tree.parent(link)
@   local IA = vars.IA(link)
@   local child_X_parent = common.link_CT_parent(link, self.members.xt)

@   if (parent ~= robot.base) or robot.isFloatingBase then
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
@   end

@end

@if robot.isFloatingBase then
    phi0 = «vars.IA(robot.base)».inverse();//TODO use specialized routine
@end

    // Inward sweep for the bias force //
    Force pIn;
    «types.scalar» aux;
@for _,link in sorted_links_reversed() do
@   lid     = common.linkIdentifier(link)
@   ancestor= robot.treeutils.parent(link)
@   joint   = robot.treeutils.supportingJoint(link)
@   jid     = common.jointIdentifier(joint)
@   if (ancestor~=robot.base) or robot.isFloatingBase then
    //
    // Joint «jid» ...
    //
    pIn = «common.parent_XF_link(link, self.members.xt)» * «vars.T(link)»;
@      local lastlink = link
@      while (ancestor ~= robot.base) do
@          local joint2 = tree.supportingJoint(ancestor)
@          local jid2   = common.jointIdentifier(joint2)
@          local idx    = common.spatialVectorIndex(joint2)
@          local parent_XF_link = common.parent_XF_link(ancestor, self.members.xt)
    //    ... on joint «jid2»
    aux = - pIn(«idx») / «D(ancestor)»;
@   if ancestor==first_link and (not robot.isFloatingBase) then
    Hi(«jid2»,«jid») = Hi(«jid»,«jid2») = aux;
@   else
    Hi(«jid2»,«jid») = aux;
@   end
@          parent = tree.parent(ancestor)
@          if (parent~=robot.base) or robot.isFloatingBase then
    pIn = «parent_XF_link» * ( pIn + «vars.IA(ancestor)».«mxops.col»(«idx») * aux );
@          end
@          lastlink = ancestor
@          ancestor = parent  -- goes up the kinematic chain
@      end
@      if robot.isFloatingBase then
@          local base_XF_link = common.parent_XF_link(lastlink, self.members.xt)
@          local ab_var = varname_abounce(jid)
    Acceleration «ab_var» = phi0 * pIn;  // a_|«robot.base.name»  ↶ «jid»|
    Finv.«mxops.col»(«jid») = - «ab_var»;
@      else
    «types.scalar» «varname_abounce_top(jid)» = -aux; // a_|«first_link.name»  ↶ «jid»| (a scalar, first link is special case)
@      end
@   end
@end

@if not robot.isFloatingBase then
    @joint = tree.supportingJoint(first_link)
    @jid   = common.jointIdentifier(joint)
    Hi(«jid»,«jid») = 1/«D(first_link)»; // special case, first moving body
@end

    // ------------------- //
    // Outward sweep start //
    // ------------------- //

@local j0_idx
@if not robot.isFloatingBase then
@   j0_idx = common.spatialVectorIndex( tree.supportingJoint(first_link) )
@end
@for _,link in sorted_links() do
@   parent= tree.parent(link)
@   if parent==robot.base and not robot.isFloatingBase then
@       goto skip
@   end
@   joint = tree.supportingJoint(link)
@   jid   = common.jointIdentifier(joint)
@   idx   = common.spatialVectorIndex(joint)
@   lid   = common.linkIdentifier(link)
@   local link_XM_parent = common.link_XM_parent(link, self.members.xt)
@   local bouncing_from_root = (robot.base == tree.parent(parent))
@   local link_C_parent = link.name .. '_C_' .. parent.name
    //
    // Joint «jid» ...
    //
@   if bouncing_from_root and not robot.isFloatingBase then
    Acceleration «link_C_parent» = «link_XM_parent».matrix().«mxops.col»(«j0_idx»);
    Hi(«jid»,«jid») = 1/«D(link)» + «vars.T(link)».«mxops.T»() * («link_C_parent» * «varname_abounce_top(jid)»);
@   else
    Hi(«jid»,«jid») = 1/«D(link)» + «vars.T(link)».«mxops.T»() * («link_XM_parent» * «varname_abounce(jid)»);
@   end
@   local link_j
@   local nB = robot.tree.nB
@   if not robot.isFloatingBase then nB = nB-1 end
@   for j = robot.tree.linkNum(link)+1, nB, 1 do
@       link_j = robot.tree.codeToLink[j]
@       joint  = tree.supportingJoint(link_j)
@       jid2   = common.jointIdentifier(joint)
@       local ab_help = abounce_helper(joint, link_j, link)

    //    ... affected by «jid2»
@       if robot.isFloatingBase then
@           if ab_help.first_off then
    Acceleration «ab_help.varname» = «link_XM_parent» * «ab_help.varname_prev»;
@           else
    «ab_help.varname» = «link_XM_parent» * «ab_help.varname»;
@           end
@       elseif bouncing_from_root then
    Acceleration «ab_help.varname» = «link_C_parent» * «varname_abounce_top(jid2)»;
@       elseif ab_help.first_off then
    Acceleration «ab_help.varname» = «link_XM_parent» * «ab_help.varname_prev»;
@       else
    «ab_help.varname» = «link_XM_parent» * «ab_help.varname»;
@       end

    Hi(«jid»,«jid2») = Hi(«jid»,«jid2») + «vars.T(link)».«mxops.T»() * «ab_help.varname»;
    Hi(«jid2»,«jid») = Hi(«jid»,«jid2»);

    «ab_help.varname»(«idx») -= Hi(«jid»,«jid2»);  // a_|«lid»  ↶ «jid2»|
@   end
@   ::skip::
@end
}

]]



local function fd_generators(robot, configurator, given_env)
    -- shallow copy the template environment, then add the fields required for
    -- the local templates
    local env = {}
    for k,v in pairs(given_env) do  env[k] = v  end
    env.tree  = robot.treeutils
    env.self  = configurator.txtCfg.meta.aba_hinv
    env.include_guard = env.includeGuard(configurator.files.h_aba_hinv)
    env.tpl = env.common.scalarTpl( env.self.class )
    env.D = function(link) return 'D_' .. link.name end

    local first_link = python.as_attrgetter(robot.movingLinks).values().__iter__().__next__()
    env.first_link = first_link
    local firstLinkId = env.common.linkIdentifier( first_link )
    local abounce_vars = {}
    env.varname_abounce_top = function(jid) return 'a__' .. firstLinkId .. '_' .. jid end
    env.varname_abounce     = function(jid) return 'a_bounce_' .. jid end
    local varname_abounce_top = env.varname_abounce_top
    if robot.isFloatingBase then varname_abounce_top = env.varname_abounce end
    for i,joint in env.sorted_joints() do
        local jid = env.common.jointIdentifier(joint)
        abounce_vars[jid] = {
            own_chain    = env.varname_abounce(jid),
            [first_link.name] = varname_abounce_top(jid)
        }
    end
    env.abounce_helper = function(joint_N, link_N, link_n)
        local common_ancestor = env.tree.lowestCommonAncestor(link_n, link_N)
        local same_chain = (common_ancestor==link_n)
        local first_off = nil
        local varname = nil
        local varname_prev = nil
        local parent_of_link_n = env.tree.parent(link_n)
        local jid = env.common.jointIdentifier(joint_N)
        if same_chain then
            varname = abounce_vars[jid].own_chain
            abounce_vars[jid][link_n.name] = varname
        else
            first_off = (common_ancestor == parent_of_link_n)
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

    local tpleval = RCG.utils.templates.tpl_eval
    return {
        header = function() return tpleval(header_template, env) end,
        source = function() return tpleval(impl_template, env) end,
    }
end


return fd_generators


