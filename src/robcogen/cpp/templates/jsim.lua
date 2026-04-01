local template_header = [[
#ifndef «include_guard»
#define «include_guard»

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentMatrix.h>

#include "«headers.main»"
#include "«headers.types»"
#include "«headers.inertia»"
#include "«headers.transforms»"

${ns.open}

/**
 * The type of the Joint Space Inertia Matrix (JSIM) of the robot «robot.name».
 */
«tpl.heading»
struct «self.class» : public «ns_iit_rbd.qualifier»::StateDependentMatrix<«types.jointState»«tpl.suffix», «robot.DOFs», «robot.DOFs», «self.class»«tpl.suffix» >
{
@if templateAll then
«typesMacro»
@end
    using «t_jstate» = «types.jointState»«tpl.suffix»;
    using Base = «ns_iit_rbd.qualifier»::StateDependentMatrix<«types.jointState»«tpl.suffix», «robot.DOFs», «robot.DOFs», «self.class»«tpl.suffix» >;
    using Index  = typename Base::Index;
    using «self.local_types.mx_full» = typename Base::MatrixType ;

@if robot.isFloatingBase  then
    /** The block-type of the F sub-block of this JSIM */
    using «self.local_types.block_F» = const «ns_iit_rbd.qualifier»::MatrixBlock<const «self.local_types.mx_full»,6,«robot.DOFs-6»> ;
    /** The block-type of the actuated-joints sub-block of this JSIM */
    using «self.local_types.block_realJoints» = const «ns_iit_rbd.qualifier»::MatrixBlock<const «self.local_types.mx_full»,«robot.DOFs-6»,«robot.DOFs-6»>;
    /** The matrix-type of the actuated-joints sub-block */
    using «self.local_types.mx_realJoints» = «ns_iit_rbd.qualifier»::PlainMatrix<Scalar, «robot.DOFs-6», «robot.DOFs-6»>;

    // For backward compatibility
    using BlockFixedBase_t = «self.local_types.block_realJoints»;
    using FixedBaseMx_t = «self.local_types.mx_realJoints»;
@end

    «self.class»(const «meta.inertia_properties.class»«tpl.suffix»&, «meta.transforms_container.class»«tpl.suffix»&);
    ~«self.class»() {}

    const «self.class»& update(const «t_jstate»&);

@if robot.isFloatingBase then
    /**
     * Computes the matrix L of the L^T L factorization *of the
     * actuated-joints block* of this JSIM.
     */
@else
    /**
     * Computes the matrix L of the L^T L factorization of this JSIM.
     */
@end
    void computeL();

    /**
     * Computes the inverse of the matrix L.
     * Assumes computeL() was called before.
     */
    void computeLInverse();

@if robot.isFloatingBase then
    /**
     * Computes the inverse of *of the actuated-joints block* of this JSIM.
     * This function does call computeLInverse() first.
     * The algorithm takes advantage of the branch
     * induced sparsity of the robot, if any.
     * Note that this matrix is NOT the full inverse of this JSIM.
     */
@else
    /**
     * Computes and stores the inverse of this JSIM.
     * This function does call computeLInverse() first.
     * The algorithm takes advantage of the branch
     * induced sparsity of the robot, if any.
     */
@end
    void computeInverse();

    /**
     * Returns an unmodifiable reference to the matrix L.
     * See also computeL()
     */
    const «self.local_types.mx_L»& getL() const { return L; }

    /**
     * Returns an unmodifiable reference to the inverse of L.
     * See also computeLInverse()
     */
    const «self.local_types.mx_L»& getLInverse() const { return Linv; }

@if robot.isFloatingBase then
    /**
     * Returns an unmodifiable reference to the last computed inverse of the
     * real joints block.
     * See also computeInverse()
     */
    const «self.local_types.mx_realJoints»& getInverse() const { return inverse; }
@else
    /**
     * Returns an unmodifiable reference to the last computed inverse of this JSIM
     */
    const «self.local_types.mx_full»& getInverse() const { return inverse; }
@end

@if robot.isFloatingBase then
    /**
     * The spatial composite-inertia tensor of the robot base.
     *
     * Ie, the inertia of the whole robot for the current configuration.
     * According to the convention of this class about the layout of the
     * floating-base JSIM, this tensor is the 6x6 upper left corner of
     * the JSIM itself.
     * \return the 6x6 InertiaMatrix that correspond to the spatial inertia
     *   tensor of the whole robot, according to the last joints configuration
     *   used to update this JSIM
     */
    const InertiaMatrix& getWholeBodyInertia() const {
        return «vars.Ic(robot.base)»;
    }
    /**
     * The matrix that maps accelerations in the actual joints of the robot
     * to the spatial force acting on the floating-base of the robot.
     * This matrix is the F sub-block of the JSIM in Featherstone's notation.
     * \return the 6x«robot.DOFs-6» upper right block of this JSIM
     */
    const «self.local_types.block_F» getF() const {
        return this->template block<6,«robot.DOFs-6»>(0,6);
    }
    /**
     * The submatrix of this JSIM related only to the actual joints of the
     * robot (as for a fixed-base robot).
     * This matrix is the H sub-block of the JSIM in Featherstone's notation.
     * \return the «robot.DOFs-6»x«robot.DOFs-6» lower right block of this JSIM,
     *   which correspond to the fixed-base JSIM
     */
    const «self.local_types.block_realJoints» «self.members.getters.realJointsBlock»() const {
        return this->template block<«robot.DOFs-6»,«robot.DOFs-6»>(6,6);
    }
    const «self.local_types.block_realJoints» getFixedBaseBlock() const {
        return this->template block<«robot.DOFs-6»,«robot.DOFs-6»>(6,6);
    }
@end

    /**
     * Computes L^{-T} times x using only L, exploiting sparsity.
     * Assumes L is already updated.
     */
    template<typename Derived>
    void LinvT_times_x(const «ns_iit_rbd.qualifier»::MatrixBase<Derived>& x_cnt);
    /**
     * Computes L^{-1} times x using only L, exploiting sparsity.
     * Assumes L is already updated.
     */
    template<typename Derived>
    void Linv_times_x(const «ns_iit_rbd.qualifier»::MatrixBase<Derived>& x_cnt);


    const «meta.inertia_properties.class»«tpl.suffix»& «self.members.inertia»;
    «meta.transforms_container.class»«tpl.suffix»& «self.members.transforms»;

    // The composite-inertia tensor for each link
@   for _, link in sorted_links(robot.isFloatingBase) do
@       if robot.treeutils.isLeaf(link) then
    const InertiaMatrix& «vars.Ic(link)»;
@       else
    InertiaMatrix «vars.Ic(link)»;
@       end
@   end
    InertiaMatrix Ic_spare;

    «self.local_types.mx_L» L;
    «self.local_types.mx_L» Linv;
    «self.local_types.mx_L» inverse;
};

«tpl.heading»
template<typename Derived>
void «self.class»«tpl.suffix»::LinvT_times_x(const «ns_iit_rbd.qualifier»::MatrixBase<Derived>& x_cnt)
{
    auto& x = const_cast<«ns_iit_rbd.qualifier»::MatrixBase<Derived>& >(x_cnt);
    //assumes L has been computed already
@for name, joint in sorted_joints_reversed() do
@   local i = common.jointIdentifier(joint)
    x(«i») /= L(«i», «i»);
@   local predecessor = robot.tree.predecessor(joint)
@   while predecessor ~= robot.base do
@       local j = common.jointIdentifier( robot.treeutils.supportingJoint(predecessor) )
@       predecessor = robot.treeutils.parent(predecessor)
    x(«j») -= L(«i», «j») * x(«i»);
@   end
@end
}

«tpl.heading»
template<typename Derived>
void «self.class»«tpl.suffix»::Linv_times_x(const «ns_iit_rbd.qualifier»::MatrixBase<Derived>& x_cnt)
{
    auto& x = const_cast<iit::rbd::MatrixBase<Derived>& >(x_cnt);
    //assumes L has been computed already
@for name, joint in sorted_joints() do
@   local i = common.jointIdentifier(joint)
@   local predecessor = robot.tree.predecessor(joint)
@   while predecessor ~= robot.base do
@       local j = common.jointIdentifier( robot.treeutils.supportingJoint(predecessor) )
@       predecessor = robot.treeutils.parent(predecessor)
    x(«i») -= L(«i», «j») * x(«j»);
@   end
    x(«i») /= L(«i», «i»);
@end
}


${ns.close}

@if templateAll then
#include "«impl_files.jsim»"
@end

#endif
]]

local template_source = [[
@if not templateAll then
#include "«headers.jsim»"
@end

#include <iit/rbd/robcogen_commons.h>

@local qualifier = ns.qualifier .. '::' .. tpl.class.in_qualifier

//Implementation of default constructor
«tpl.heading»
«qualifier»::«self.class»(const «meta.inertia_properties.class»«tpl.suffix»& ip, «meta.transforms_container.class»«tpl.suffix»& xt) :
    «self.members.inertia»(ip),
    «self.members.transforms»( xt ),
@   for _, link, comma in utils.i_iterator_with_separator(function() return ipairs(leafs) end, ",") do
    «vars.Ic(link)»(ip.«meta.inertia_properties.members.tensorGetter(link)»())«comma»
@   end
{
    //Initialize the matrix itself
    this->setZero();
    this->L.setZero();
    this->Linv.setZero();
}

@if robot.isFloatingBase  then
#define DATA(r,c) this->operator()(r+6,c+6)
#define FCOL(c) this->template block<6,1>(0,c+6)
@else
#define DATA this->operator()
@end

«tpl.heading»
const «qualifier»& «qualifier»::update(const «t_jstate»& state)
{
    using namespace «ns_iit_rbd.qualifier»;
    Force F;

    // Precomputes only once the coordinate transforms:
@for _, link in sorted_links() do
@   if robot.isFloatingBase or (robot.treeutils.parent(link) ~= robot.base) then
    «common.link_CT_parent(link, self.members.transforms)»(state);
@   end
@end

    // Initializes the composite inertia tensors
@for  _, link in sorted_links(robot.isFloatingBase) do
@   if not robot.treeutils.isLeaf(link) then
    «vars.Ic(link)» = «self.members.inertia».«meta.inertia_properties.members.tensorGetter(link)»();
@   end
@end

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration
@for name, link in sorted_links_reversed() do
    // Link «name»:
@   local parent = robot.treeutils.parent(link)
@   if robot.isFloatingBase  or (parent ~= robot.base) then
    transformInertia<«types.scalar»>(«vars.Ic(link)», «common.link_CT_parent(link, self.members.transforms)».ct, Ic_spare);
    «vars.Ic(parent)» += Ic_spare;
@   end

@   local jt = robot.tree.linkPairToJoint(parent, link)
@   local jointIndex = common.jointIdentifier(jt)
    F = «vars.Ic(link)».col(«common.spatialVectorIndex(jt)»);
    DATA(«jointIndex», «jointIndex») = F(«common.spatialVectorIndex(jt)»);

@   local ancestor = parent
@   local last = link
@   while ancestor ~= robot.base do
@       local ancestor2 = robot.treeutils.parent(ancestor)
@       local joint  = robot.tree.linkPairToJoint(ancestor2, ancestor)
@       local icol   = common.jointIdentifier(joint)
    F = «parent_XF_link(last)» * F;
    DATA(«jointIndex», «icol») = DATA(«icol», «jointIndex») = F(«common.spatialVectorIndex(joint)»);
@       last     = ancestor
@       ancestor = ancestor2
@   end
@   if robot.isFloatingBase  then
    FCOL(«jointIndex») = «parent_XF_link(last)» * F;
@   end

@end

@if robot.isFloatingBase  then
    // Copies the upper-right block into the lower-left block, after transposing
    this->template block<«robot.DOFs-6», 6>(6,0) = (this->template block<6, «robot.DOFs-6»>(0,6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    this->template block<6,6>(0,0) = «vars.Ic(robot.base)»;
@end
    return *this;
}

@if robot.isFloatingBase then
#undef FCOL
@end

«tpl.heading»
void «qualifier»::computeL()
{
@for name, joint in sorted_joints_reversed() do
@   local row  = common.jointIdentifier(joint)
@   local leaf = robot.treeutils.isLeaf(robot.tree.successor(joint))
    // Joint «name», index «row»
@   if not leaf then
    L(«row», «row») = «types.scalarTraits»«tpl.suffix»::sqrt( L(«row», «row») );
@   else
    L(«row», «row») = «types.scalarTraits»«tpl.suffix»::sqrt( DATA(«row», «row») );
@   end
@   local ancestor = robot.tree.predecessor(joint)
@   while ancestor ~= robot.base do
@       local col = common.jointIdentifier( robot.treeutils.supportingJoint(ancestor) )
@       ancestor = robot.treeutils.parent(ancestor)
@       if not leaf then
    L(«row», «col») /= L(«row», «row»);
@       else
    L(«row», «col») = DATA(«row», «col») / L(«row», «row»);
@       end
@   end
@   ancestor = robot.tree.predecessor(joint)
@   while ancestor ~= robot.base do
@       local i = common.jointIdentifier( robot.treeutils.supportingJoint(ancestor) )
@       local ancestor2 = ancestor
@       while ancestor2 ~= robot.base do
@           local j = common.jointIdentifier( robot.treeutils.supportingJoint(ancestor2) )
@           if not leaf then
    L(«i», «j») -= L(«row», «i») * L(«row», «j»);
@           else
    L(«i», «j») = DATA(«i», «j») - L(«row», «i») * L(«row», «j»);
@           end
@           ancestor2 = robot.treeutils.parent(ancestor2)
@       end
@       ancestor = robot.treeutils.parent(ancestor)
@   end

@end
}

#undef DATA

«tpl.heading»
void «qualifier»::computeLInverse() {
    //assumes L has been computed already
@for name, joint in sorted_joints() do
@   local i = common.jointIdentifier(joint)
    Linv(«i», «i») = 1 / L(«i», «i»);
@end
@for name, joint in sorted_joints() do
@   if robot.tree.jointNum(joint) > 1 then -- ignore the very first joint
@       local link     = robot.tree.successor(joint)
@       local ancestor = robot.tree.predecessor(joint)
@       local i        = common.jointIdentifier(joint)
@       while ancestor ~= robot.base do
@           local parent = robot.treeutils.parent(ancestor)
@           local joint2 = robot.tree.linkPairToJoint(ancestor, parent)
@           local j      = common.jointIdentifier(joint2)
    Linv(«i», «j») = - Linv(«j», «j») * (
@           local auxl = link
@           while auxl ~= ancestor do
@               local auxp   = robot.treeutils.parent(auxl)
@               local joint3 = robot.tree.linkPairToJoint(auxl, auxp)
@               local k      = common.jointIdentifier(joint3)
        ( Linv(«i», «k») * L(«k», «j») ) +
@               auxl = auxp
@           end
            0);
@           ancestor = parent -- go up the chain
@       end
@   end
@end
}

«tpl.heading»
void «qualifier»::computeInverse()
{
    computeLInverse();

@for i = 0,robot.tree.nJ-1 do
@   for j = i,0,-1 do
    inverse(«i», «j») =
@       -- follow the chain all the way up to the base, starting from jo_j itself
@       local jo_j = robot.tree.codeToJoint[j+1]
@       local link = robot.tree.successor(jo_j)
@       while link~= robot.base do
@           local ancestor = robot.treeutils.parent(link)
@           local jo_k = robot.tree.linkPairToJoint(ancestor, link)
@           local k    = common.jointIdentifier(jo_k)
        + ( Linv(«i»,«k») * Linv(«j»,«k») )
@           link = ancestor
@       end
        ;
@       if i~= j then
    inverse(«j», «i») = inverse(«i», «j»);
@       end
@   end
@end
}
]]



local function generators_jsim(robot, configurator, given_env)
    -- shallow copy the template environment, then add the fields required for
    -- the local templates
    local env = {}
    for k,v in pairs(given_env) do  env[k] = v  end

    env.self = configurator.txtCfg.meta.jsim
    env.tpl  = env.common.scalarTpl( env.self.class )
    env.t_jstate = env.types.classScopeAliases.jointState

    if robot.isFloatingBase then
        env.self.local_types.mx_L = env.self.local_types.mx_realJoints
    else
        env.self.local_types.mx_L = env.self.local_types.mx_full
    end

    env.parent_XF_link = function(link)
        return env.common.parent_XF_link(link, env.self.members.transforms)
    end

    return {
        header = function()
            env.include_guard = env.includeGuard(configurator.files.h_jsim)
            return RCG.utils.templates.tpl_eval(template_header, env)
        end,
        source = function()
            return RCG.utils.templates.tpl_eval(template_source, env)
        end,
    }
end



return generators_jsim
