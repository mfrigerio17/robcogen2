%% This Octave script runs all the tests provided by RobCoGen, for the generated
%% Octave solvers.
%% For more information see the readme file in '<ROBCOGEN ROOT>/testing/'
%%
%% Required arguments:
%% - absolute path of the root of the "spatial_v2" software package
%% - absolute path of the RobCoGen testing sources, normally '<ROBCOGEN ROOT>/testing/src'
%%

args = argv();
if length(args) < 2
    error("Usage: SCRIPT <spatial_v2 path> <robcogen test src path>");
end

spatial_v2 = args{1};
abspath_testing_src = args{2};

addpath( abspath_testing_src );
addpath(genpath( spatial_v2 ) );

robot     = rcg2.init();
robot.roy = rcg2.RoyModel(robot.mgc, robot.mgp, robot.mic, robot.mip);

printf('\nTest of the generated Octave code for Inverse Dynamics\n');
[roy, me] = test_id_fb(robot.roy, robot);

printf('\nTest of the generated Octave code for the Joint Space Inertia Matrix\n');
[roy, me] = test_jsim_fb(robot.roy, robot);

printf('\nTest of the generated Octave code for Forward Dynamics\n');
[roy, me] = test_fd_fb(robot.roy, robot);

