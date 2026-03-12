% Simple demonstration of how to change the parametric inertial properties
%
addpath("../../gen/octave");

fancy = rcg2.init();

display( fancy.ip.link1.mass );          % the current value

params = rcg2.ModelInertiaParameters();  % a container for parameter values
params.m1 = 2;

fancy.ip.updateParameters(params);

display( fancy.ip.link1.mass );
