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

@ if robot.isFloatingBase then
    // The robot base
    Matrix66 «vars.IA(robot.base)»;
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
@for i,l in sorted_links(robot.isFloatingBase) do
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
@   local lid     = common.linkIdentifier(link)
@   local ancestor= robot.treeutils.parent(link)
@   local joint   = robot.treeutils.supportingJoint(link)
@   local jid     = common.jointIdentifier(joint)
@   if (ancestor~=robot.base) or robot.isFloatingBase then
    //
    // Joint «jid» ...
    //
    pIn = «common.parent_XF_link(link, self.members.xt)» * «vars.T(link)»;
@       local lastlink = link
@       while (ancestor ~= robot.base) do
@           local joint2 = tree.supportingJoint(ancestor)
@           local jid2   = common.jointIdentifier(joint2)
@           local idx    = common.spatialVectorIndex(joint2)
@           local parent_XF_link = common.parent_XF_link(ancestor, self.members.xt)
    //    ... on joint «jid2»
    aux = - pIn(«idx») / «D(ancestor)»;
@           if ancestor==first_link and (not robot.isFloatingBase) then
    Hi(«jid2»,«jid») = Hi(«jid»,«jid2») = aux;
@           else
    Hi(«jid2»,«jid») = aux;
@           end
@           lastlink = ancestor
@           ancestor = tree.parent(ancestor) -- goes up the kinematic chain
@           if (ancestor~=robot.base) or robot.isFloatingBase then
    pIn = «parent_XF_link» * ( pIn + «vars.IA(lastlink)».«mxops.col»(«idx») * aux );
@           end
@       end
@       local ab_var = abounce_vars[first_link.name][joint.name]
@       if robot.isFloatingBase then
@           local base_XF_link = common.parent_XF_link(lastlink, self.members.xt)
    //    ... on the robot base
    Acceleration «ab_var» = phi0 * pIn;  // a_{«robot.base.name» ↶ «jid»}
    Finv.«mxops.col»(«jid») = - «ab_var»;
@       else
    //    ... on «first_link.name»; a_{«first_link.name» ↶ «jid»} is sparse, we store just a scalar
    «types.scalar» «ab_var» = -aux;
@       end
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
@   local parent= tree.parent(link)
@   if parent==robot.base and not robot.isFloatingBase then
@       goto skip
@   end
@   local joint = tree.supportingJoint(link)
@   local jid   = common.jointIdentifier(joint)
@   local idx   = common.spatialVectorIndex(joint)
@   local lid   = common.linkIdentifier(link)
@   local link_XM_parent = common.link_XM_parent(link, self.members.xt)
@   local bouncing_from_root = (robot.base == tree.parent(parent))
@   local link_C_parent = link.name .. '_C_' .. parent.name
@   local linkHasSiblings = robot.hasSiblings(link)
    //
    // Joint «jid» ...
    //
@   if bouncing_from_root and not robot.isFloatingBase then
    Acceleration «link_C_parent» = «link_XM_parent».matrix().«mxops.col»(«j0_idx»);
    Hi(«jid»,«jid») = 1/«D(link)» + «vars.T(link)».«mxops.T»() * («link_C_parent» * «abounce_vars[parent.name][joint.name]»);
@   else
    Hi(«jid»,«jid») = 1/«D(link)» + «vars.T(link)».«mxops.T»() * («link_XM_parent» * «abounce_vars[parent.name][joint.name]»);
@   end
@   for j = robot.tree.linkNum(link)+1, robot.tree.nB-1, 1 do
@       local link_j = robot.tree.codeToLink[j]
@       joint = tree.supportingJoint(link_j)
@       local jid2   = common.jointIdentifier(joint)
@       local a_bounce_parent = abounce_vars[parent.name][joint.name]
@       local a_bounce        = abounce_vars[link.name][joint.name]
    //    ... affected by «jid2»
@       if (bouncing_from_root and not robot.isFloatingBase) then
    Acceleration «a_bounce» = «link_C_parent» * «a_bounce_parent»;
@       elseif linkHasSiblings then
    Acceleration «a_bounce» = «link_XM_parent» * «a_bounce_parent»;
@       else
    «a_bounce» = «link_XM_parent» * «a_bounce_parent»;
@       end

    Hi(«jid»,«jid2») = Hi(«jid»,«jid2») + «vars.T(link)».«mxops.T»() * «a_bounce»;
    Hi(«jid2»,«jid») = Hi(«jid»,«jid2»);

    «a_bounce»(«idx») -= Hi(«jid»,«jid2»);  // a_{«lid» ↶ «jid2»}
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

    local varname_abounce = function(jid, lid) return string.format('a__%s_chain__by_%s', lid, jid) end
    local aux = {}
    for jname,_ in env.sorted_joints() do
        aux[jname] = varname_abounce(jname, first_link.name)
    end
    local abounce_vars = {
        [first_link.name] = aux
    }
    for name, link in env.sorted_links() do
        if link ~= first_link then
            aux = {}
            local parent = robot.treeutils.parent(link)
            for jname,_ in env.sorted_joints() do
                if (parent~=first_link or robot.isFloatingBase) and (not robot.hasSiblings(link)) then
                    aux[jname] = abounce_vars[parent.name][jname]
                else
                    aux[jname] = varname_abounce(jname, name)
                end
            end
            abounce_vars[name] = aux
        end
    end
    env.abounce_vars = abounce_vars

    local tpleval = RCG.utils.templates.tpl_eval
    return {
        header = function() return tpleval(header_template, env) end,
        source = function() return tpleval(impl_template, env) end,
    }
end


return fd_generators


