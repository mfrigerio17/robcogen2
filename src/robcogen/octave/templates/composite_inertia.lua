local template = [[
function ret = «thisFunc.name»(«thisFunc.args.ip», «thisFunc.args.transforms», transformsType)

% Computes the spatial composite inertia of each link of the robot.
% Arguments:
% - «thisFunc.args.ip» : the structure with the inertia properties
% - «thisFunc.args.transforms» : the structure with the spatial coordinate transformation matrices
% - transformsType : a string specifying which is the type of the given
%      coordinate transforms, either velocity ('motion') or force ('force').
%      Optional argument, default is 'force'.

if nargin < 3
    transformsType = 'force';
end

%
% Initialization of the composite-inertia matrices
%
ret = struct(«table.concat(struct_init_list, ",")»);

%
% Leafs-to-root pass to update the composite inertia of
%     each link, for the current configuration:
%
if strcmp(transformsType, 'motion')  % we have transforms for motion vectors
@for name,link in sorted_links_reversed(robot) do
@   local parent = robot.treeutils.parent(link)
@   if robot.isFloatingBase or ( parent~=robot.tree.base ) then
@       local child_X_parent = thisFunc.args.transforms..'.'..meta.class_transforms_container.members.individual_tf( link_XM_parent(link) )..'.mx'
% Contribution of link «name»
ret.«ids.Ic(parent)» = ret.«ids.Ic(parent)» + «child_X_parent»' * ret.«ids.Ic(link)» * «child_X_parent»;
@   end

@end

else % we have transforms for force vectors
@for name,link in sorted_links(robot) do
@   local parent = robot.treeutils.parent(link)
@   if robot.isFloatingBase or ( parent~=robot.tree.base ) then
@       local child_X_parent = thisFunc.args.transforms..'.'..meta.class_transforms_container.members.individual_tf( link_XF_parent(link) )..'.mx'
% Contribution of link «name»
ret.«ids.Ic(parent)» = ret.«ids.Ic(parent)» + «child_X_parent» * ret.«ids.Ic(link)» * «child_X_parent»';
@   end

@end

end
]]

return template
