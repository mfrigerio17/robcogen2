function qdd = forwardDynamics(ip, xm, qd, tau, fext)
g = 9.81;

if nargin < 5
    p_link1 = zeros(6,1);
    p_link2 = zeros(6,1);
    p_link3 = zeros(6,1);
    p_link4 = zeros(6,1);
    p_link5 = zeros(6,1);
else
    p_link1 = - fext{1};
    p_link2 = - fext{2};
    p_link3 = - fext{3};
    p_link4 = - fext{4};
    p_link5 = - fext{5};
end

qdd = zeros(5,1);

IA_link1 = ip.link1.tensor6D;
IA_link2 = ip.link2.tensor6D;
IA_link3 = ip.link3.tensor6D;
IA_link4 = ip.link4.tensor6D;
IA_link5 = ip.link5.tensor6D;


% + Link link1
%    body velocity
v_link1 = [0;0;qd(1);0;0;0];

%    bias force
p_link1 = p_link1 + vxIv(qd(1), IA_link1);


% + Link link2
%    body velocity
v_link2 = xm.link2_X_link1.mx * v_link1;
v_link2(6) = v_link2(6) + qd(2);

%    velocity-product acceleration term
vcross = vcross_mx(v_link2);
c_link2 = vcross(:,6) * qd(2);

%    bias force
p_link2 = p_link2 + -vcross' * IA_link2 * v_link2; %%%vxIv(v_link2, IA_link2);


% + Link link3
%    body velocity
v_link3 = xm.link3_X_link2.mx * v_link2;
v_link3(3) = v_link3(3) + qd(3);

%    velocity-product acceleration term
vcross = vcross_mx(v_link3);
c_link3 = vcross(:,3) * qd(3);

%    bias force
p_link3 = p_link3 + -vcross' * IA_link3 * v_link3; %%%vxIv(v_link3, IA_link3);


% + Link link4
%    body velocity
v_link4 = xm.link4_X_link3.mx * v_link3;
v_link4(6) = v_link4(6) + qd(4);

%    velocity-product acceleration term
vcross = vcross_mx(v_link4);
c_link4 = vcross(:,6) * qd(4);

%    bias force
p_link4 = p_link4 + -vcross' * IA_link4 * v_link4; %%%vxIv(v_link4, IA_link4);


% + Link link5
%    body velocity
v_link5 = xm.link5_X_link4.mx * v_link4;
v_link5(3) = v_link5(3) + qd(5);

%    velocity-product acceleration term
vcross = vcross_mx(v_link5);
c_link5 = vcross(:,3) * qd(5);

%    bias force
p_link5 = p_link5 + -vcross' * IA_link5 * v_link5; %%%vxIv(v_link5, IA_link5);



% ---------------------- SECOND PASS ---------------------- %


% + Link link5
link5_u = tau(5) - p_link5(3);
link5_U = IA_link5(:,3);
link5_D = link5_U(3);

Ia_r = IA_link5 - link5_U/link5_D * link5_U';
pa = p_link5 + Ia_r * c_link5 + link5_U * link5_u/link5_D;
IaB = xm.link5_X_link4.mx' * Ia_r * xm.link5_X_link4.mx;    %% ctransform_Ia_revolute(Ia_r, xm.link5_X_link4.mx.ct, IaB);
IA_link4 = IA_link4 + IaB;
p_link4 = p_link4 + xm.link5_X_link4.mx' * pa;

% + Link link4
link4_u = tau(4) - p_link4(6);
link4_U = IA_link4(:,6);
link4_D = link4_U(6);

Ia_p = IA_link4 - link4_U/link4_D * link4_U';
pa = p_link4 + Ia_p * c_link4 + link4_U * link4_u/link4_D;
IaB = xm.link4_X_link3.mx' * Ia_p * xm.link4_X_link3.mx;    %% ctransform_Ia_prismatic(Ia_p, xm.link4_X_link3.mx, IaB);
IA_link3 = IA_link3 + IaB;
p_link3 = p_link3 + xm.link4_X_link3.mx' * pa;

% + Link link3
link3_u = tau(3) - p_link3(3);
link3_U = IA_link3(:,3);
link3_D = link3_U(3);

Ia_r = IA_link3 - link3_U/link3_D * link3_U';
pa = p_link3 + Ia_r * c_link3 + link3_U * link3_u/link3_D;
IaB = xm.link3_X_link2.mx' * Ia_r * xm.link3_X_link2.mx;    %% ctransform_Ia_revolute(Ia_r, xm.link3_X_link2.mx.ct, IaB);
IA_link2 = IA_link2 + IaB;
p_link2 = p_link2 + xm.link3_X_link2.mx' * pa;

% + Link link2
link2_u = tau(2) - p_link2(6);
link2_U = IA_link2(:,6);
link2_D = link2_U(6);

Ia_p = IA_link2 - link2_U/link2_D * link2_U';
pa = p_link2 + Ia_p * c_link2 + link2_U * link2_u/link2_D;
IaB = xm.link2_X_link1.mx' * Ia_p * xm.link2_X_link1.mx;    %% ctransform_Ia_prismatic(Ia_p, xm.link2_X_link1.mx, IaB);
IA_link1 = IA_link1 + IaB;
p_link1 = p_link1 + xm.link2_X_link1.mx' * pa;

% + Link link1
link1_u = tau(1) - p_link1(3);
link1_U = IA_link1(:,3);
link1_D = link1_U(3);



% ---------------------- THIRD PASS ---------------------- %
a_link1 = xm.link1_X_base0.mx(:,6) * g;
qdd(1) = (link1_u - dot(link1_U, a_link1)) / link1_D;
a_link1(3) = a_link1(3) + qdd(1);

a_link2 = xm.link2_X_link1.mx * a_link1 + c_link2;
qdd(2) = (link2_u - dot(link2_U, a_link2)) / link2_D;
a_link2(6) = a_link2(6) + qdd(2);

a_link3 = xm.link3_X_link2.mx * a_link2 + c_link3;
qdd(3) = (link3_u - dot(link3_U, a_link3)) / link3_D;
a_link3(3) = a_link3(3) + qdd(3);

a_link4 = xm.link4_X_link3.mx * a_link3 + c_link4;
qdd(4) = (link4_u - dot(link4_U, a_link4)) / link4_D;
a_link4(6) = a_link4(6) + qdd(4);

a_link5 = xm.link5_X_link4.mx * a_link4 + c_link5;
qdd(5) = (link5_u - dot(link5_U, a_link5)) / link5_D;
a_link5(3) = a_link5(3) + qdd(5);


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

