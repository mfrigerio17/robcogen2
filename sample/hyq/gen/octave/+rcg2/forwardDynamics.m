function [qdd a_trunk] = forwardDynamics(ip, xm, v_trunk, gravity, qd, tau, fext)

if nargin < 7
    p_trunk = zeros(6,1);
    p_LF_hipassembly = zeros(6,1);
    p_LF_upperleg = zeros(6,1);
    p_LF_lowerleg = zeros(6,1);
    p_RF_hipassembly = zeros(6,1);
    p_RF_upperleg = zeros(6,1);
    p_RF_lowerleg = zeros(6,1);
    p_LH_hipassembly = zeros(6,1);
    p_LH_upperleg = zeros(6,1);
    p_LH_lowerleg = zeros(6,1);
    p_RH_hipassembly = zeros(6,1);
    p_RH_upperleg = zeros(6,1);
    p_RH_lowerleg = zeros(6,1);
else
    p_trunk = - fext{1};
    p_LF_hipassembly = - fext{2};
    p_LF_upperleg = - fext{3};
    p_LF_lowerleg = - fext{4};
    p_RF_hipassembly = - fext{5};
    p_RF_upperleg = - fext{6};
    p_RF_lowerleg = - fext{7};
    p_LH_hipassembly = - fext{8};
    p_LH_upperleg = - fext{9};
    p_LH_lowerleg = - fext{10};
    p_RH_hipassembly = - fext{11};
    p_RH_upperleg = - fext{12};
    p_RH_lowerleg = - fext{13};
end

qdd = zeros(12,1);

IA_trunk = ip.trunk.tensor6D;
IA_LF_hipassembly = ip.LF_hipassembly.tensor6D;
IA_LF_upperleg = ip.LF_upperleg.tensor6D;
IA_LF_lowerleg = ip.LF_lowerleg.tensor6D;
IA_RF_hipassembly = ip.RF_hipassembly.tensor6D;
IA_RF_upperleg = ip.RF_upperleg.tensor6D;
IA_RF_lowerleg = ip.RF_lowerleg.tensor6D;
IA_LH_hipassembly = ip.LH_hipassembly.tensor6D;
IA_LH_upperleg = ip.LH_upperleg.tensor6D;
IA_LH_lowerleg = ip.LH_lowerleg.tensor6D;
IA_RH_hipassembly = ip.RH_hipassembly.tensor6D;
IA_RH_upperleg = ip.RH_upperleg.tensor6D;
IA_RH_lowerleg = ip.RH_lowerleg.tensor6D;


% + Link LF_hipassembly
%    body velocity
v_LF_hipassembly = xm.LF_hipassembly_X_trunk.mx * v_trunk;
v_LF_hipassembly(3) = v_LF_hipassembly(3) + qd(1);

%    velocity-product acceleration term
vcross = vcross_mx(v_LF_hipassembly);
c_LF_hipassembly = vcross(:,3) * qd(1);

%    bias force
p_LF_hipassembly = p_LF_hipassembly + -vcross' * IA_LF_hipassembly * v_LF_hipassembly; %%%vxIv(v_LF_hipassembly, IA_LF_hipassembly);


% + Link LF_upperleg
%    body velocity
v_LF_upperleg = xm.LF_upperleg_X_LF_hipassembly.mx * v_LF_hipassembly;
v_LF_upperleg(3) = v_LF_upperleg(3) + qd(2);

%    velocity-product acceleration term
vcross = vcross_mx(v_LF_upperleg);
c_LF_upperleg = vcross(:,3) * qd(2);

%    bias force
p_LF_upperleg = p_LF_upperleg + -vcross' * IA_LF_upperleg * v_LF_upperleg; %%%vxIv(v_LF_upperleg, IA_LF_upperleg);


% + Link LF_lowerleg
%    body velocity
v_LF_lowerleg = xm.LF_lowerleg_X_LF_upperleg.mx * v_LF_upperleg;
v_LF_lowerleg(3) = v_LF_lowerleg(3) + qd(3);

%    velocity-product acceleration term
vcross = vcross_mx(v_LF_lowerleg);
c_LF_lowerleg = vcross(:,3) * qd(3);

%    bias force
p_LF_lowerleg = p_LF_lowerleg + -vcross' * IA_LF_lowerleg * v_LF_lowerleg; %%%vxIv(v_LF_lowerleg, IA_LF_lowerleg);


% + Link RF_hipassembly
%    body velocity
v_RF_hipassembly = xm.RF_hipassembly_X_trunk.mx * v_trunk;
v_RF_hipassembly(3) = v_RF_hipassembly(3) + qd(4);

%    velocity-product acceleration term
vcross = vcross_mx(v_RF_hipassembly);
c_RF_hipassembly = vcross(:,3) * qd(4);

%    bias force
p_RF_hipassembly = p_RF_hipassembly + -vcross' * IA_RF_hipassembly * v_RF_hipassembly; %%%vxIv(v_RF_hipassembly, IA_RF_hipassembly);


% + Link RF_upperleg
%    body velocity
v_RF_upperleg = xm.RF_upperleg_X_RF_hipassembly.mx * v_RF_hipassembly;
v_RF_upperleg(3) = v_RF_upperleg(3) + qd(5);

%    velocity-product acceleration term
vcross = vcross_mx(v_RF_upperleg);
c_RF_upperleg = vcross(:,3) * qd(5);

%    bias force
p_RF_upperleg = p_RF_upperleg + -vcross' * IA_RF_upperleg * v_RF_upperleg; %%%vxIv(v_RF_upperleg, IA_RF_upperleg);


% + Link RF_lowerleg
%    body velocity
v_RF_lowerleg = xm.RF_lowerleg_X_RF_upperleg.mx * v_RF_upperleg;
v_RF_lowerleg(3) = v_RF_lowerleg(3) + qd(6);

%    velocity-product acceleration term
vcross = vcross_mx(v_RF_lowerleg);
c_RF_lowerleg = vcross(:,3) * qd(6);

%    bias force
p_RF_lowerleg = p_RF_lowerleg + -vcross' * IA_RF_lowerleg * v_RF_lowerleg; %%%vxIv(v_RF_lowerleg, IA_RF_lowerleg);


% + Link LH_hipassembly
%    body velocity
v_LH_hipassembly = xm.LH_hipassembly_X_trunk.mx * v_trunk;
v_LH_hipassembly(3) = v_LH_hipassembly(3) + qd(7);

%    velocity-product acceleration term
vcross = vcross_mx(v_LH_hipassembly);
c_LH_hipassembly = vcross(:,3) * qd(7);

%    bias force
p_LH_hipassembly = p_LH_hipassembly + -vcross' * IA_LH_hipassembly * v_LH_hipassembly; %%%vxIv(v_LH_hipassembly, IA_LH_hipassembly);


% + Link LH_upperleg
%    body velocity
v_LH_upperleg = xm.LH_upperleg_X_LH_hipassembly.mx * v_LH_hipassembly;
v_LH_upperleg(3) = v_LH_upperleg(3) + qd(8);

%    velocity-product acceleration term
vcross = vcross_mx(v_LH_upperleg);
c_LH_upperleg = vcross(:,3) * qd(8);

%    bias force
p_LH_upperleg = p_LH_upperleg + -vcross' * IA_LH_upperleg * v_LH_upperleg; %%%vxIv(v_LH_upperleg, IA_LH_upperleg);


% + Link LH_lowerleg
%    body velocity
v_LH_lowerleg = xm.LH_lowerleg_X_LH_upperleg.mx * v_LH_upperleg;
v_LH_lowerleg(3) = v_LH_lowerleg(3) + qd(9);

%    velocity-product acceleration term
vcross = vcross_mx(v_LH_lowerleg);
c_LH_lowerleg = vcross(:,3) * qd(9);

%    bias force
p_LH_lowerleg = p_LH_lowerleg + -vcross' * IA_LH_lowerleg * v_LH_lowerleg; %%%vxIv(v_LH_lowerleg, IA_LH_lowerleg);


% + Link RH_hipassembly
%    body velocity
v_RH_hipassembly = xm.RH_hipassembly_X_trunk.mx * v_trunk;
v_RH_hipassembly(3) = v_RH_hipassembly(3) + qd(10);

%    velocity-product acceleration term
vcross = vcross_mx(v_RH_hipassembly);
c_RH_hipassembly = vcross(:,3) * qd(10);

%    bias force
p_RH_hipassembly = p_RH_hipassembly + -vcross' * IA_RH_hipassembly * v_RH_hipassembly; %%%vxIv(v_RH_hipassembly, IA_RH_hipassembly);


% + Link RH_upperleg
%    body velocity
v_RH_upperleg = xm.RH_upperleg_X_RH_hipassembly.mx * v_RH_hipassembly;
v_RH_upperleg(3) = v_RH_upperleg(3) + qd(11);

%    velocity-product acceleration term
vcross = vcross_mx(v_RH_upperleg);
c_RH_upperleg = vcross(:,3) * qd(11);

%    bias force
p_RH_upperleg = p_RH_upperleg + -vcross' * IA_RH_upperleg * v_RH_upperleg; %%%vxIv(v_RH_upperleg, IA_RH_upperleg);


% + Link RH_lowerleg
%    body velocity
v_RH_lowerleg = xm.RH_lowerleg_X_RH_upperleg.mx * v_RH_upperleg;
v_RH_lowerleg(3) = v_RH_lowerleg(3) + qd(12);

%    velocity-product acceleration term
vcross = vcross_mx(v_RH_lowerleg);
c_RH_lowerleg = vcross(:,3) * qd(12);

%    bias force
p_RH_lowerleg = p_RH_lowerleg + -vcross' * IA_RH_lowerleg * v_RH_lowerleg; %%%vxIv(v_RH_lowerleg, IA_RH_lowerleg);


% Bias force on the floating base
vcross = vcross_mx(v_trunk);
p_trunk = p_trunk + -vcross' * IA_trunk * v_trunk;%%%vxIv(v_trunk, IA_trunk);

% ---------------------- SECOND PASS ---------------------- %


% + Link RH_lowerleg
RH_lowerleg_u = tau(12) - p_RH_lowerleg(3);
RH_lowerleg_U = IA_RH_lowerleg(:,3);
RH_lowerleg_D = RH_lowerleg_U(3);

Ia_r = IA_RH_lowerleg - RH_lowerleg_U/RH_lowerleg_D * RH_lowerleg_U';
pa = p_RH_lowerleg + Ia_r * c_RH_lowerleg + RH_lowerleg_U * RH_lowerleg_u/RH_lowerleg_D;
IaB = xm.RH_lowerleg_X_RH_upperleg.mx' * Ia_r * xm.RH_lowerleg_X_RH_upperleg.mx;    %% ctransform_Ia_revolute(Ia_r, xm.RH_lowerleg_X_RH_upperleg.mx.ct, IaB);
IA_RH_upperleg = IA_RH_upperleg + IaB;
p_RH_upperleg = p_RH_upperleg + xm.RH_lowerleg_X_RH_upperleg.mx' * pa;

% + Link RH_upperleg
RH_upperleg_u = tau(11) - p_RH_upperleg(3);
RH_upperleg_U = IA_RH_upperleg(:,3);
RH_upperleg_D = RH_upperleg_U(3);

Ia_r = IA_RH_upperleg - RH_upperleg_U/RH_upperleg_D * RH_upperleg_U';
pa = p_RH_upperleg + Ia_r * c_RH_upperleg + RH_upperleg_U * RH_upperleg_u/RH_upperleg_D;
IaB = xm.RH_upperleg_X_RH_hipassembly.mx' * Ia_r * xm.RH_upperleg_X_RH_hipassembly.mx;    %% ctransform_Ia_revolute(Ia_r, xm.RH_upperleg_X_RH_hipassembly.mx.ct, IaB);
IA_RH_hipassembly = IA_RH_hipassembly + IaB;
p_RH_hipassembly = p_RH_hipassembly + xm.RH_upperleg_X_RH_hipassembly.mx' * pa;

% + Link RH_hipassembly
RH_hipassembly_u = tau(10) - p_RH_hipassembly(3);
RH_hipassembly_U = IA_RH_hipassembly(:,3);
RH_hipassembly_D = RH_hipassembly_U(3);

Ia_r = IA_RH_hipassembly - RH_hipassembly_U/RH_hipassembly_D * RH_hipassembly_U';
pa = p_RH_hipassembly + Ia_r * c_RH_hipassembly + RH_hipassembly_U * RH_hipassembly_u/RH_hipassembly_D;
IaB = xm.RH_hipassembly_X_trunk.mx' * Ia_r * xm.RH_hipassembly_X_trunk.mx;    %% ctransform_Ia_revolute(Ia_r, xm.RH_hipassembly_X_trunk.mx.ct, IaB);
IA_trunk = IA_trunk + IaB;
p_trunk = p_trunk + xm.RH_hipassembly_X_trunk.mx' * pa;

% + Link LH_lowerleg
LH_lowerleg_u = tau(9) - p_LH_lowerleg(3);
LH_lowerleg_U = IA_LH_lowerleg(:,3);
LH_lowerleg_D = LH_lowerleg_U(3);

Ia_r = IA_LH_lowerleg - LH_lowerleg_U/LH_lowerleg_D * LH_lowerleg_U';
pa = p_LH_lowerleg + Ia_r * c_LH_lowerleg + LH_lowerleg_U * LH_lowerleg_u/LH_lowerleg_D;
IaB = xm.LH_lowerleg_X_LH_upperleg.mx' * Ia_r * xm.LH_lowerleg_X_LH_upperleg.mx;    %% ctransform_Ia_revolute(Ia_r, xm.LH_lowerleg_X_LH_upperleg.mx.ct, IaB);
IA_LH_upperleg = IA_LH_upperleg + IaB;
p_LH_upperleg = p_LH_upperleg + xm.LH_lowerleg_X_LH_upperleg.mx' * pa;

% + Link LH_upperleg
LH_upperleg_u = tau(8) - p_LH_upperleg(3);
LH_upperleg_U = IA_LH_upperleg(:,3);
LH_upperleg_D = LH_upperleg_U(3);

Ia_r = IA_LH_upperleg - LH_upperleg_U/LH_upperleg_D * LH_upperleg_U';
pa = p_LH_upperleg + Ia_r * c_LH_upperleg + LH_upperleg_U * LH_upperleg_u/LH_upperleg_D;
IaB = xm.LH_upperleg_X_LH_hipassembly.mx' * Ia_r * xm.LH_upperleg_X_LH_hipassembly.mx;    %% ctransform_Ia_revolute(Ia_r, xm.LH_upperleg_X_LH_hipassembly.mx.ct, IaB);
IA_LH_hipassembly = IA_LH_hipassembly + IaB;
p_LH_hipassembly = p_LH_hipassembly + xm.LH_upperleg_X_LH_hipassembly.mx' * pa;

% + Link LH_hipassembly
LH_hipassembly_u = tau(7) - p_LH_hipassembly(3);
LH_hipassembly_U = IA_LH_hipassembly(:,3);
LH_hipassembly_D = LH_hipassembly_U(3);

Ia_r = IA_LH_hipassembly - LH_hipassembly_U/LH_hipassembly_D * LH_hipassembly_U';
pa = p_LH_hipassembly + Ia_r * c_LH_hipassembly + LH_hipassembly_U * LH_hipassembly_u/LH_hipassembly_D;
IaB = xm.LH_hipassembly_X_trunk.mx' * Ia_r * xm.LH_hipassembly_X_trunk.mx;    %% ctransform_Ia_revolute(Ia_r, xm.LH_hipassembly_X_trunk.mx.ct, IaB);
IA_trunk = IA_trunk + IaB;
p_trunk = p_trunk + xm.LH_hipassembly_X_trunk.mx' * pa;

% + Link RF_lowerleg
RF_lowerleg_u = tau(6) - p_RF_lowerleg(3);
RF_lowerleg_U = IA_RF_lowerleg(:,3);
RF_lowerleg_D = RF_lowerleg_U(3);

Ia_r = IA_RF_lowerleg - RF_lowerleg_U/RF_lowerleg_D * RF_lowerleg_U';
pa = p_RF_lowerleg + Ia_r * c_RF_lowerleg + RF_lowerleg_U * RF_lowerleg_u/RF_lowerleg_D;
IaB = xm.RF_lowerleg_X_RF_upperleg.mx' * Ia_r * xm.RF_lowerleg_X_RF_upperleg.mx;    %% ctransform_Ia_revolute(Ia_r, xm.RF_lowerleg_X_RF_upperleg.mx.ct, IaB);
IA_RF_upperleg = IA_RF_upperleg + IaB;
p_RF_upperleg = p_RF_upperleg + xm.RF_lowerleg_X_RF_upperleg.mx' * pa;

% + Link RF_upperleg
RF_upperleg_u = tau(5) - p_RF_upperleg(3);
RF_upperleg_U = IA_RF_upperleg(:,3);
RF_upperleg_D = RF_upperleg_U(3);

Ia_r = IA_RF_upperleg - RF_upperleg_U/RF_upperleg_D * RF_upperleg_U';
pa = p_RF_upperleg + Ia_r * c_RF_upperleg + RF_upperleg_U * RF_upperleg_u/RF_upperleg_D;
IaB = xm.RF_upperleg_X_RF_hipassembly.mx' * Ia_r * xm.RF_upperleg_X_RF_hipassembly.mx;    %% ctransform_Ia_revolute(Ia_r, xm.RF_upperleg_X_RF_hipassembly.mx.ct, IaB);
IA_RF_hipassembly = IA_RF_hipassembly + IaB;
p_RF_hipassembly = p_RF_hipassembly + xm.RF_upperleg_X_RF_hipassembly.mx' * pa;

% + Link RF_hipassembly
RF_hipassembly_u = tau(4) - p_RF_hipassembly(3);
RF_hipassembly_U = IA_RF_hipassembly(:,3);
RF_hipassembly_D = RF_hipassembly_U(3);

Ia_r = IA_RF_hipassembly - RF_hipassembly_U/RF_hipassembly_D * RF_hipassembly_U';
pa = p_RF_hipassembly + Ia_r * c_RF_hipassembly + RF_hipassembly_U * RF_hipassembly_u/RF_hipassembly_D;
IaB = xm.RF_hipassembly_X_trunk.mx' * Ia_r * xm.RF_hipassembly_X_trunk.mx;    %% ctransform_Ia_revolute(Ia_r, xm.RF_hipassembly_X_trunk.mx.ct, IaB);
IA_trunk = IA_trunk + IaB;
p_trunk = p_trunk + xm.RF_hipassembly_X_trunk.mx' * pa;

% + Link LF_lowerleg
LF_lowerleg_u = tau(3) - p_LF_lowerleg(3);
LF_lowerleg_U = IA_LF_lowerleg(:,3);
LF_lowerleg_D = LF_lowerleg_U(3);

Ia_r = IA_LF_lowerleg - LF_lowerleg_U/LF_lowerleg_D * LF_lowerleg_U';
pa = p_LF_lowerleg + Ia_r * c_LF_lowerleg + LF_lowerleg_U * LF_lowerleg_u/LF_lowerleg_D;
IaB = xm.LF_lowerleg_X_LF_upperleg.mx' * Ia_r * xm.LF_lowerleg_X_LF_upperleg.mx;    %% ctransform_Ia_revolute(Ia_r, xm.LF_lowerleg_X_LF_upperleg.mx.ct, IaB);
IA_LF_upperleg = IA_LF_upperleg + IaB;
p_LF_upperleg = p_LF_upperleg + xm.LF_lowerleg_X_LF_upperleg.mx' * pa;

% + Link LF_upperleg
LF_upperleg_u = tau(2) - p_LF_upperleg(3);
LF_upperleg_U = IA_LF_upperleg(:,3);
LF_upperleg_D = LF_upperleg_U(3);

Ia_r = IA_LF_upperleg - LF_upperleg_U/LF_upperleg_D * LF_upperleg_U';
pa = p_LF_upperleg + Ia_r * c_LF_upperleg + LF_upperleg_U * LF_upperleg_u/LF_upperleg_D;
IaB = xm.LF_upperleg_X_LF_hipassembly.mx' * Ia_r * xm.LF_upperleg_X_LF_hipassembly.mx;    %% ctransform_Ia_revolute(Ia_r, xm.LF_upperleg_X_LF_hipassembly.mx.ct, IaB);
IA_LF_hipassembly = IA_LF_hipassembly + IaB;
p_LF_hipassembly = p_LF_hipassembly + xm.LF_upperleg_X_LF_hipassembly.mx' * pa;

% + Link LF_hipassembly
LF_hipassembly_u = tau(1) - p_LF_hipassembly(3);
LF_hipassembly_U = IA_LF_hipassembly(:,3);
LF_hipassembly_D = LF_hipassembly_U(3);

Ia_r = IA_LF_hipassembly - LF_hipassembly_U/LF_hipassembly_D * LF_hipassembly_U';
pa = p_LF_hipassembly + Ia_r * c_LF_hipassembly + LF_hipassembly_U * LF_hipassembly_u/LF_hipassembly_D;
IaB = xm.LF_hipassembly_X_trunk.mx' * Ia_r * xm.LF_hipassembly_X_trunk.mx;    %% ctransform_Ia_revolute(Ia_r, xm.LF_hipassembly_X_trunk.mx.ct, IaB);
IA_trunk = IA_trunk + IaB;
p_trunk = p_trunk + xm.LF_hipassembly_X_trunk.mx' * pa;

%  acceleration of the floating base trunk, without gravity
a_trunk = - IA_trunk \ p_trunk;  % a_trunk = - IA^-1 * p_trunk

% ---------------------- THIRD PASS ---------------------- %
a_LF_hipassembly = xm.LF_hipassembly_X_trunk.mx * a_trunk + c_LF_hipassembly;
qdd(1) = (LF_hipassembly_u - dot(LF_hipassembly_U, a_LF_hipassembly)) / LF_hipassembly_D;
a_LF_hipassembly(3) = a_LF_hipassembly(3) + qdd(1);

a_LF_upperleg = xm.LF_upperleg_X_LF_hipassembly.mx * a_LF_hipassembly + c_LF_upperleg;
qdd(2) = (LF_upperleg_u - dot(LF_upperleg_U, a_LF_upperleg)) / LF_upperleg_D;
a_LF_upperleg(3) = a_LF_upperleg(3) + qdd(2);

a_LF_lowerleg = xm.LF_lowerleg_X_LF_upperleg.mx * a_LF_upperleg + c_LF_lowerleg;
qdd(3) = (LF_lowerleg_u - dot(LF_lowerleg_U, a_LF_lowerleg)) / LF_lowerleg_D;
a_LF_lowerleg(3) = a_LF_lowerleg(3) + qdd(3);

a_RF_hipassembly = xm.RF_hipassembly_X_trunk.mx * a_trunk + c_RF_hipassembly;
qdd(4) = (RF_hipassembly_u - dot(RF_hipassembly_U, a_RF_hipassembly)) / RF_hipassembly_D;
a_RF_hipassembly(3) = a_RF_hipassembly(3) + qdd(4);

a_RF_upperleg = xm.RF_upperleg_X_RF_hipassembly.mx * a_RF_hipassembly + c_RF_upperleg;
qdd(5) = (RF_upperleg_u - dot(RF_upperleg_U, a_RF_upperleg)) / RF_upperleg_D;
a_RF_upperleg(3) = a_RF_upperleg(3) + qdd(5);

a_RF_lowerleg = xm.RF_lowerleg_X_RF_upperleg.mx * a_RF_upperleg + c_RF_lowerleg;
qdd(6) = (RF_lowerleg_u - dot(RF_lowerleg_U, a_RF_lowerleg)) / RF_lowerleg_D;
a_RF_lowerleg(3) = a_RF_lowerleg(3) + qdd(6);

a_LH_hipassembly = xm.LH_hipassembly_X_trunk.mx * a_trunk + c_LH_hipassembly;
qdd(7) = (LH_hipassembly_u - dot(LH_hipassembly_U, a_LH_hipassembly)) / LH_hipassembly_D;
a_LH_hipassembly(3) = a_LH_hipassembly(3) + qdd(7);

a_LH_upperleg = xm.LH_upperleg_X_LH_hipassembly.mx * a_LH_hipassembly + c_LH_upperleg;
qdd(8) = (LH_upperleg_u - dot(LH_upperleg_U, a_LH_upperleg)) / LH_upperleg_D;
a_LH_upperleg(3) = a_LH_upperleg(3) + qdd(8);

a_LH_lowerleg = xm.LH_lowerleg_X_LH_upperleg.mx * a_LH_upperleg + c_LH_lowerleg;
qdd(9) = (LH_lowerleg_u - dot(LH_lowerleg_U, a_LH_lowerleg)) / LH_lowerleg_D;
a_LH_lowerleg(3) = a_LH_lowerleg(3) + qdd(9);

a_RH_hipassembly = xm.RH_hipassembly_X_trunk.mx * a_trunk + c_RH_hipassembly;
qdd(10) = (RH_hipassembly_u - dot(RH_hipassembly_U, a_RH_hipassembly)) / RH_hipassembly_D;
a_RH_hipassembly(3) = a_RH_hipassembly(3) + qdd(10);

a_RH_upperleg = xm.RH_upperleg_X_RH_hipassembly.mx * a_RH_hipassembly + c_RH_upperleg;
qdd(11) = (RH_upperleg_u - dot(RH_upperleg_U, a_RH_upperleg)) / RH_upperleg_D;
a_RH_upperleg(3) = a_RH_upperleg(3) + qdd(11);

a_RH_lowerleg = xm.RH_lowerleg_X_RH_upperleg.mx * a_RH_upperleg + c_RH_lowerleg;
qdd(12) = (RH_lowerleg_u - dot(RH_lowerleg_U, a_RH_lowerleg)) / RH_lowerleg_D;
a_RH_lowerleg(3) = a_RH_lowerleg(3) + qdd(12);


%   add gravity to the acceleration of the floating base
a_trunk = a_trunk + gravity;
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

