local template = [[
classdef «thisclass.name» < handle
properties
    «thisclass.members.inertia»
    «thisclass.members.transforms»
@for name, link in sorted_links(robot.isFloatingBase) do
    «ids.Ic(link)»
@end
    «thisclass.members.H»
@if robot.isFloatingBase then
    «thisclass.members.F»
@end
end

methods
    function obj = «thisclass.name»(«thisclass.ctor.inertia», «thisclass.ctor.transforms»)
    % Arguments:
    %  - thisclass.ctor.inertia» : a structure with the inertia properties
    %  - «thisclass.ctor.transforms» : the container of the coordinate transformation
    %    matrices for spatial motion vectors

        obj.«thisclass.members.inertia» = «thisclass.ctor.inertia»;
        obj.«thisclass.members.transforms» = «thisclass.ctor.transforms»;
@for name, link in sorted_links(robot.isFloatingBase) do
@   if robot.treeutils.isLeaf(link) then
        obj.«ids.Ic(link)» = «thisclass.ctor.inertia».«meta.class_inertia_properties.members.linkip(link)».tensor6D;
@   else
        obj.«ids.Ic(link)» = zeros(6,6);
@   end
@end
        obj.«thisclass.members.H» = zeros(«robot.properDOFs»,«robot.properDOFs»);
@if robot.isFloatingBase then
        obj.«thisclass.members.F» = zeros(6,«robot.properDOFs»);
@end
    end

    function «thisclass.methods.update_ci»(obj)
    % Computes the spatial composite inertia of each link of the robot.
    % This method uses the current robot configuration. To use another one,
    % update first the coordinate transforms used by this instance.
    % The computed inertia are available as public members of this instance.

@local notfirst = {}
@for name,link in sorted_links_reversed() do
@   local parent = robot.treeutils.parent(link)
@   if robot.isFloatingBase or ( parent~=robot.tree.base ) then
        % Contribution of «name» on «parent.name»
@       if notfirst[parent.name] then
        obj.«ids.Ic(parent)» = obj.«ids.Ic(parent)» + obj.«link_XM_parent(link)».mx' * obj.«ids.Ic(link)» * obj.«link_XM_parent(link)».mx;
@       else
        obj.«ids.Ic(parent)» = obj.«thisclass.members.inertia».«meta.class_inertia_properties.members.linkip(parent)».tensor6D + obj.«link_XM_parent(link)».mx' * obj.«ids.Ic(link)» * obj.«link_XM_parent(link)».mx;
@       notfirst[parent.name] = true
@       end
@   end

@   end
    end

    function update_JSIM(obj)
    % Computes the Joint Space Inertia Matrix of the robot «robot.name».
    % This method uses the current composite inertia values.
    % See 'update_composite_inertia'.
    % The computed value is available in the public member '«thisclass.members.H»'
    % of this instance (and in '«thisclass.members.F»' for floating base models).

@for name, link in sorted_links_reversed() do
@   local parent= robot.treeutils.parent(link)
@   local joint = robot.tree.linkPairToJoint(parent, link)
@   local jnt_i = commons.jointStateVectorIndex(joint)
        F = obj.«ids.Ic(link)»(:,«commons.spatialVectorIndex(joint)»);
        obj.«thisclass.members.H»(«jnt_i», «jnt_i») = F(«commons.spatialVectorIndex(joint)»);

@   local ancestor = parent
@   local last = link
@   while ancestor ~= robot.base do
@       local ancestor2 = robot.treeutils.parent(ancestor)
@       local joint  = robot.tree.linkPairToJoint(ancestor2, ancestor)
@       local icol   = commons.jointStateVectorIndex(joint)
        F = obj.«link_XM_parent(last)».mx' * F;
        obj.«thisclass.members.H»(«jnt_i», «icol») = obj.«thisclass.members.H»(«icol», «jnt_i») = F(«commons.spatialVectorIndex(joint)»);
@       last     = ancestor
@       ancestor = ancestor2
@   end
@   if robot.isFloatingBase  then
        obj.«thisclass.members.F»(:,«jnt_i») = obj.«link_XM_parent(last)».mx' * F;
@   end

@end
    end

@if robot.isFloatingBase  then
    function ic = base_Ic(obj)
        ic = obj.«ids.Ic(robot.base)»;
    end
@end

end % methods
end % class
]]




return template



