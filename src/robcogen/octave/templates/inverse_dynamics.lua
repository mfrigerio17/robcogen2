local template = [[
${heading}

% TODO: possibly, in the future, optimize the spatial cross product (i.e.
%  avoid code that computes the whole 6x6 matrix)
@local base = robot.tree.base -- just an alias to simplify this template
@if robot.isFloatingBase then
function [tau «ids.acc(base)»] = «here.name»(«here.args.ip», «here.args.transforms», «ids.vel(base)», gravity, «here.args.qd», «here.args.qdd», «here.args.fext»)
@else
function tau = «here.name»(«here.args.ip», «here.args.transforms», «here.args.qd», «here.args.qdd», «here.args.fext»)

g = 9.81;
@end

@if robot.isFloatingBase then
if nargin < 7
    «here.args.fext» = cell(«robot.tree.nB»,1);
@else
if nargin < 5
    «here.args.fext» = cell(«robot.tree.nB-1»,1);
@ end
    «here.args.fext»(:) = {zeros(6,1)};
end

%
% Pass 1. Forward propagate velocities and accelerations
%
@for name,link in sorted_links() do
@  local parent   = robot.treeutils.parent(link)
@  local myJoint  = robot.treeutils.supportingJoint(link)
@  local velocity = ids.vel(link)
@  local acceler  = ids.acc(link)
@  local tensor   = inertia(link)
@  local child_X_parent = child_mx_parent(link)
@  local jid         = commons.jointStateVectorIndex(myJoint)
@  local subspaceIdx = commons.spatialVectorIndex(myJoint)
@  local lid = commons.linkArrayIndex(link)

% Link '«link.name»'
@   if robot.isFloatingBase then
«velocity» = «child_X_parent» * «ids.vel(parent)»;
«velocity»(«subspaceIdx») = «velocity»(«subspaceIdx») + «here.args.qd»(«jid»);

vcross = vcross_mx(«velocity»);

@    if parent == base then -- parent is the floating base
«acceler» = vcross(:,«subspaceIdx») * «here.args.qd»(«jid»);
@    else
«acceler» = «child_X_parent» * «ids.acc(parent)» + (vcross(:,«subspaceIdx») * «here.args.qd»(«jid»));
@    end
«acceler»(«subspaceIdx») = «acceler»(«subspaceIdx») + «here.args.qdd»(«jid»);

«ids.force(link)» = -«here.args.fext»{«lid»} + «tensor» * «acceler» + (-vcross' * «tensor» * «velocity»);

@   else -- fixed base
@    if parent == base then
«acceler» = «child_X_parent»(:,6) * g; % TODO hide 6
«acceler»(«subspaceIdx») = «acceler»(«subspaceIdx») + «here.args.qdd»(«jid»);
«velocity» = «spatialVelDueToJointOnly(myJoint, here.args.qd, jid)»;
@
@      if myJoint.kind.name == "prismatic" then
% The first joint is prismatic, no centripetal terms.
«ids.force(link)» = -fext{«lid»} + «tensor» * «acceler»;
@      else
w2 = «here.args.qd»(«jid»)*«here.args.qd»(«jid»);
vxIv = [-«tensor»(2,3) * w2; ...
         «tensor»(1,2) * w2; ...
         0; ...
         «tensor»(2,6) * w2; ...
         «tensor»(3,4) * w2; ...
         0];
«ids.force(link)» = -«here.args.fext»{«lid»} + «tensor» * «acceler» + vxIv;
@      end
@    else
«velocity» = ((«child_X_parent») * «ids.vel(parent)»);
«velocity»(«subspaceIdx») = «velocity»(«subspaceIdx») + «here.args.qd»(«jid»);

vcross = vcross_mx(«velocity»);

«acceler» = «child_X_parent» * «ids.acc(parent)» + (vcross(:,«subspaceIdx») * «here.args.qd»(«jid»));
«acceler»(«subspaceIdx») = «acceler»(«subspaceIdx») + «here.args.qdd»(«jid»);

«ids.force(link)» = -«here.args.fext»{«lid»} + «tensor» * «acceler» + (-vcross' * «tensor» * «velocity»);
@    end
@  end
@end

@if not robot.isFloatingBase then

%
% Pass 2. Compute the joint torques while back propagating the spatial forces
%
tau = zeros(size(«here.args.qd»));
@  for name,link in sorted_links_reversed() do

% Link '«name»'
@    local parent = robot.treeutils.parent(link)
@    local joint  = robot.treeutils.supportingJoint(link)
tau(«commons.jointStateVectorIndex(joint)») = «ids.force(link)»(«commons.spatialVectorIndex(joint)»);
@    if ( parent~=base ) then
@      local child_X_parent = here.args.transforms..'.'..meta.class_transforms_container.members.individual_tf( link_XM_parent(link) )..'.mx'
«ids.force(parent)» = «ids.force(parent)» + «child_X_parent»' * «ids.force(link)»;
@    end
@  end

@else -- floating base case

@  local base = base
%
% The force exerted on the floating base by the links
%
vcross = vcross_mx(«ids.vel(base)»);
«ids.force(base)» = -«here.args.fext»{1} - vcross' * «inertia(base)» * «ids.vel(base)»;

%
% Pass 2. Compute the composite inertia and the spatial forces
%
ci = «ns_qualifier»«meta.class_jsim.name»(«here.args.ip», «here.args.transforms»);
ci.«meta.class_jsim.methods.update_ci»();
@   for name,link in sorted_links_reversed() do
@       local parent = robot.treeutils.parent(link)
«ids.force(parent)» = «ids.force(parent)» + «child_mx_parent(link)»' * «ids.force(link)»;
@   end

%
% The base acceleration due to the force due to the movement of the links
%
«ids.acc(base)» = - ci.«ids.Ic(base)» \ «ids.force(base)»; % TODO optimise the inversion

%
% Pass 3. Compute the joint forces while propagating back the floating base acceleration
%
tau = zeros(size(«here.args.qd»));
@ for name,link in sorted_links() do
@    local parent = robot.treeutils.parent(link)
@    local joint  = robot.treeutils.supportingJoint(link)
@    local idx    = commons.spatialVectorIndex(joint)
«ids.acc(link)» = «child_mx_parent(link)» * «ids.acc(parent)»;
tau(«commons.jointStateVectorIndex(joint)») = ci.«ids.Ic(link)»(«idx»,:) * «ids.acc(link)» + «ids.force(link)»(«idx»);

@ end

«ids.acc(base)» = «ids.acc(base)» + gravity;

@end
end

function vc = vcross_mx(v)
    vc = [   0    -v(3)  v(2)   0     0     0    ;
             v(3)  0    -v(1)   0     0     0    ;
            -v(2)  v(1)  0      0     0     0    ;
             0    -v(6)  v(5)   0    -v(3)  v(2) ;
             v(6)  0    -v(4)   v(3)  0    -v(1) ;
            -v(5)  v(4)  0     -v(2)  v(1)  0    ];
end
]]



return template
