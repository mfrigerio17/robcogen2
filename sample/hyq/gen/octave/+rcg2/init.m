function robot = init()

robot.mic = rcg2.ModelInertiaConstants();
robot.mip = rcg2.ModelInertiaParameters();
robot.mgc = rcg2.ModelGeometryConstants();
robot.mgp = rcg2.ModelGeometryParameters();

robot.ip  = rcg2.InertiaProperties(robot.mic, robot.mip);
robot.xm  = rcg2.MotionTransforms(robot.mgc, robot.mgp);

robot.jsim = rcg2.CompositeInertia(robot.ip, robot.xm);

% Some function handles for convenience:

robot.updateK = @(q) robot.xm.updateAll(q);

robot.ID = @(v_base, gravity, qd, qdd, fext) rcg2.inverseDynamics(robot.ip, robot.xm, v_base, gravity, qd, qdd, fext);
robot.FD = @(v_base, gravity, qd, tau, fext) rcg2.forwardDynamics(robot.ip, robot.xm, v_base, gravity, qd, tau, fext);


% addpath( <robcogen_root>/testing/src );
% addpath( genpath( <spatial_V2 root> ) );
% robot.roy = rcg2.RoyModel(robot.mgc, robot.mgp, robot.mic, robot.mip);
