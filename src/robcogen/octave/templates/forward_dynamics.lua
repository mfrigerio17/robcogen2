local template = [[
@if robot.isFloatingBase then
function [qdd «ids.acc(robot.base)»] = «here.name»(«here.args.ip», «here.args.transforms», «ids.vel(robot.base)», gravity, «here.args.qd», «here.args.tau», «here.args.fext»)
@else
function qdd = «here.name»(«here.args.ip», «here.args.transforms», «here.args.qd», «here.args.tau», «here.args.fext»)
g = 9.81;
@end

@if robot.isFloatingBase then
if nargin < 7
@else
if nargin < 5
@ end
@for  _,link in sorted_links(robot.isFloatingBase) do
    «ids.biasF(link)» = zeros(6,1);
@end
else
@for  _,link in sorted_links(robot.isFloatingBase) do
    «ids.biasF(link)» = - «here.args.fext»{«commons.linkArrayIndex(link)»};
@end
end

qdd = zeros(«robot.properDOFs»,1);

@for  _,link in sorted_links(robot.isFloatingBase) do
«ids.IA(link)» = «here.args.ip».«meta.class_inertia_properties.members.linkip(link)».tensor6D;
@end

@ for  name,link in sorted_links() do
@   local parent   = robot.treeutils.parent(link)
@   local joint    = robot.treeutils.supportingJoint(link)
@   local velocity = ids.vel(link)
@   local cterm    = ids.biasA(link)
@   local biasF    = ids.biasF(link)
@   local IA       = ids.IA(link)
@   local jid      = commons.jointStateVectorIndex(joint)
@   local idx      = commons.spatialVectorIndex(joint)
@   local child_X_parent = child_mx_parent(link)

% + Link «name»
@   if (parent==robot.base and not robot.isFloatingBase) then
%    body velocity
«velocity» = «spatialVelDueToJointOnly(joint, here.args.qd, jid)»;

%    bias force
@       if joint.kind == enums.JointKind.prismatic then
%    first joint is prismatic, no bias force term
@       else
«biasF» = «biasF» + vxIv(«here.args.qd»(«jid»), «IA»);
@       end
@   else
%    body velocity
«velocity» = «child_X_parent» * «ids.vel(parent)»;
«velocity»(«idx») = «velocity»(«idx») + «here.args.qd»(«jid»);

%    velocity-product acceleration term
vcross = vcross_mx(«velocity»);
«cterm» = vcross(:,«idx») * «here.args.qd»(«jid»);

%    bias force
«biasF» = «biasF» + -vcross' * «IA» * «velocity»; %%%vxIv(«velocity», «IA»);
@   end

@end

@if robot.isFloatingBase then
% Bias force on the floating base
vcross = vcross_mx(«ids.vel(robot.base)»);
«ids.biasF(robot.base)» = «ids.biasF(robot.base)» + -vcross' * «ids.IA(robot.base)» * «ids.vel(robot.base)»;%%%vxIv(«ids.vel(robot.base)», «ids.IA(robot.base)»);
@end

% ---------------------- SECOND PASS ---------------------- %

@for name,link in sorted_links_reversed() do
@   local parent= robot.treeutils.parent(link)
@   local joint = robot.treeutils.supportingJoint(link)
@   local idx   = commons.spatialVectorIndex(joint)
@   local U     = UTermName(link)
@   local D     = DTermName(link)
@   local u     = uTermName(link)
@   local p     = ids.biasF(link)
@   local I     = ids.IA(link)
@   local child_X_parent = child_mx_parent(link)

% + Link «name»
«u» = «here.args.tau»(«commons.jointStateVectorIndex(joint)») - «p»(«idx»);
«U» = «I»(:,«idx»);
«D» = «U»(«idx»);

@   if (parent~=robot.base or robot.isFloatingBase) then
@       if joint.kind == enums.JointKind.prismatic then
Ia_p = «I» - «U»/«D» * «U»';
pa = «p» + Ia_p * «ids.biasA(link)» + «U» * «u»/«D»;
IaB = «child_X_parent»' * Ia_p * «child_X_parent»;    %% ctransform_Ia_prismatic(Ia_p, «child_X_parent», IaB);
@       else
Ia_r = «I» - «U»/«D» * «U»';
pa = «p» + Ia_r * «ids.biasA(link)» + «U» * «u»/«D»;
IaB = «child_X_parent»' * Ia_r * «child_X_parent»;    %% ctransform_Ia_revolute(Ia_r, «child_X_parent».ct, IaB);
@       end
«ids.IA(parent)» = «ids.IA(parent)» + IaB;
«ids.biasF(parent)» = «ids.biasF(parent)» + «child_X_parent»' * pa;
@   end
@end

@if robot.isFloatingBase then
%  acceleration of the floating base «robot.base.name», without gravity
«ids.acc(robot.base)» = - «ids.IA(robot.base)» \ «ids.biasF(robot.base)»;  % «ids.acc(robot.base)» = - IA^-1 * «ids.biasF(robot.base)»
@end

% ---------------------- THIRD PASS ---------------------- %
@ for  name,link in sorted_links() do
@   local parent = robot.treeutils.parent(link)
@   local joint  = robot.treeutils.supportingJoint(link)
@   local jid    = commons.jointStateVectorIndex(joint)
@   local acc    = ids.acc(link)
@   if (parent==robot.base and not robot.isFloatingBase) then
«acc» = «child_mx_parent(link)»(:,«commons.linear_Z_coordinate») * g;
@   else
«acc» = «child_mx_parent(link)» * «ids.acc(parent)» + «ids.biasA(link)»;
@   end
qdd(«jid») = («uTermName(link)» - dot(«UTermName(link)», «ids.acc(link)»)) / «DTermName(link)»;
«acc»(«commons.spatialVectorIndex(joint)») = «acc»(«commons.spatialVectorIndex(joint)») + qdd(«jid»);

@end

@if robot.isFloatingBase then
%   add gravity to the acceleration of the floating base
«ids.acc(robot.base)» = «ids.acc(robot.base)» + gravity;
@end
end

function ret = vxIv(omegaz, I)
    wz2 = omegaz*omegaz;
    ret = zeros(6,1);
    ret(1) = -I(2,3) * wz2;
    ret(2) =  I(1,2) * wz2;
    %%ret(3) =  0;
    ret(4) =  I(2,6) * wz2;
    ret(5) =  I(3,4) * wz2;
    %%ret(6) =  0;
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
