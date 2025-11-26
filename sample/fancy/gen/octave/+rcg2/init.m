function robot = init()

robot.mic = rcg2.ModelInertiaConstants();
robot.mip = rcg2.ModelInertiaParameters();
robot.mgc = rcg2.ModelGeometryConstants();
robot.mgp = rcg2.ModelGeometryParameters();

robot.ip  = rcg2.InertiaProperties(robot.mic, robot.mip);
robot.xm  = rcg2.MotionTransforms(robot.mgc, robot.mgp);

robot.ID  = @(qd, qdd, fext) rcg2.inverseDynamics(robot.ip, robot.xm, qd, qdd, fext);

% addpath( <robcogen_root>/testing/src );
% addpath( genpath( <spatial_V2 root> ) );
% robot.roy = rcg2.FancyRoyModel(Fancy.mgc, Fancy.mgp, Fancy.mic, Fancy.mip);
