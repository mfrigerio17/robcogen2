
%%
%% Usage: [QD QDD J JD] = bodyToFBJointMotion(P, V, A, FBJ_SPECS)
%%
%% Transforms a spatial velocity and acceleration of a rigid body into the
%% velocity and acceleration of a virtual 6-Dof joint, which is assumed to
%% connect the fixed ground with the moving body.
%%
%% P is the position status of the virtual joint, ie 6 scalars. V and A are
%% respectively the spatial velocity and acceleration vector of the body.
%% FBJ_SPECS is the specification of the 6-dof joint,
%% as a cell array of six strings (optional parameter). The default value is
%% {'Px', 'Py', 'Pz', 'Rx', 'Ry', 'Rz'}, which corresponds to three translations
%% along x,y,z and three rotations about x,y,z (successive rotations); see the
%% documentation in Roy's spatial_v2.
%%
%% The function returns QD and QDD, the joint-space velocity and acceleration.
%% The third return value J is the Jacobian matrix that transforms the joint
%% velocity into the spatial velocity of the body, expressed in the body frame.
%% The fourth, JD, optional, is the time derivative of the Jacobian.
%%
%% This function depends on 'spatial_v2', namely the 'jcalc' function.
%%
%% Author: Marco Frigerio
%% Created in: 2013
%
%%% This file is part of the Octave test functions for RobCoGen.
%%% See the LICENSE file for more information.

function [qd qdd J Jd] = bodyToFBJointMotion(base_p, base_v, base_a, fbjoint_specs)

if nargin < 4
    fbjoint_specs = {'Px', 'Py', 'Pz', 'Rx', 'Ry', 'Rz'};
end

v{6} = base_v;
a{6} = base_a;

q = base_p;

J  = zeros(6,6);
Jp = zeros(6,6);
X  = eye(6);
Xp = eye(6);
dt = 0.00001;

for i = 6:-1:1
  [ childXj, S ] = jcalc( fbjoint_specs{i}, q(i) );
  X = X * childXj;
  J(:,i) = X * S;
end

Jinv  = inv(J);

% The joint space velocity:
qd  = Jinv * base_v;

% For the acceleration we need to estimate the time derivative of the Jacobian.
% The following finds the jacobian for the configuration q+dq
for i = 6:-1:1
  [ childXj, S ] = jcalc( fbjoint_specs{i}, q(i)+(qd(i)*dt) );
  Xp = Xp * childXj;
  Jprime(:,i) = Xp * S;
end

if nargout > 3
  Jd = (Jprime - J) / dt;
end

% Note that: J^{-1} Jdot qdot = Jdot^{-1} J qdot = Jdot^{-1} xdot
%
Jprimeinv = inv(Jprime);
qdd = (Jprimeinv-Jinv)/dt * base_v + Jinv*base_a;

