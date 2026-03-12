% Simple demonstration of how to invoke the generated solvers
%
addpath("../../gen/octave");

robot = rcg2.init();

dof = 5;  % depends on your model
q   = rand(dof,1);
qd  = rand(dof,1);
qdd = rand(dof,1);

robot.xm.updateAll(q);  % forward kinematics on the full tree

tau = rcg2.inverseDynamics(robot.ip, robot.xm, qd, qdd);

qdd2 = rcg2.forwardDynamics(robot.ip, robot.xm, qd, tau);

display(qdd-qdd2)
