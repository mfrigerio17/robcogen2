function ret = compositeInertia(ip, xf, transformsType)

% Computes the spatial composite inertia of each link of the robot.
% Arguments:
% - ip : the structure with the inertia properties
% - xf : the structure with the spatial coordinate transformation matrices
% - transformsType : a string specifying which is the type of the given
%      coordinate transforms, either velocity ('motion') or force ('force').
%      Optional argument, default is 'force'.

if nargin < 3
    transformsType = 'force';
end

%
% Initialization of the composite-inertia matrices
%
ret = struct('Ic_link1',ip.link1.tensor6D,'Ic_link2',ip.link2.tensor6D,'Ic_link3',ip.link3.tensor6D,'Ic_link4',ip.link4.tensor6D,'Ic_link5',ip.link5.tensor6D);

%
% Leafs-to-root pass to update the composite inertia of
%     each link, for the current configuration:
%
if strcmp(transformsType, 'motion')  % we have transforms for motion vectors
% Contribution of link link5
ret.Ic_link4 = ret.Ic_link4 + xf.link5_X_link4.mx' * ret.Ic_link5 * xf.link5_X_link4.mx;

% Contribution of link link4
ret.Ic_link3 = ret.Ic_link3 + xf.link4_X_link3.mx' * ret.Ic_link4 * xf.link4_X_link3.mx;

% Contribution of link link3
ret.Ic_link2 = ret.Ic_link2 + xf.link3_X_link2.mx' * ret.Ic_link3 * xf.link3_X_link2.mx;

% Contribution of link link2
ret.Ic_link1 = ret.Ic_link1 + xf.link2_X_link1.mx' * ret.Ic_link2 * xf.link2_X_link1.mx;



else % we have transforms for force vectors

% Contribution of link link2
ret.Ic_link1 = ret.Ic_link1 + xf.link2_X_link1.mx * ret.Ic_link2 * xf.link2_X_link1.mx';

% Contribution of link link3
ret.Ic_link2 = ret.Ic_link2 + xf.link3_X_link2.mx * ret.Ic_link3 * xf.link3_X_link2.mx';

% Contribution of link link4
ret.Ic_link3 = ret.Ic_link3 + xf.link4_X_link3.mx * ret.Ic_link4 * xf.link4_X_link3.mx';

% Contribution of link link5
ret.Ic_link4 = ret.Ic_link4 + xf.link5_X_link4.mx * ret.Ic_link5 * xf.link5_X_link4.mx';


end
