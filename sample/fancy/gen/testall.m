%% This Octave script runs all the tests provided by RobCoGen, on the generated
%% code (both Octave and C++).
%% For more information see the readme file in '<ROBCOGEN ROOT>/testing/'
%%
%% Required arguments:
%% - absolute path of the root of the "spatial_v2" software package
%% - absolute path of the RobCoGen testing sources, normally '<ROBCOGEN ROOT>/testing/src'
%% - absolute path of the root of the generated Octave code
%% - absolute path of the build folder of the generated C++ code
%%

args = argv();
spatial_v2 = args{1};
abspath_testing_src = args{2};
abspath_gen_octave = args{3};
abspath_gen_cpp_build = args{4};

addpath( abspath_gen_octave );
addpath( abspath_testing_src );
addpath(genpath( spatial_v2 ) );

robot     = rcg2.init();
robot.roy = rcg2.RoyModel(robot.mgc, robot.mgp, robot.mic, robot.mip);

printf('\nTest of the generated Octave code for Inverse Dynamics\n');
[roy, me] = test_id(robot.roy, robot);

printf('\nTest of the generated Octave code for the JSIM\n');
[roy, me] = test_jsim(robot.roy, robot);

printf('\nTest of the generated Octave code for Forward Dynamics\n');
[roy, me] = test_fd(robot.roy, robot);


exe_id = [abspath_gen_cpp_build, '/test-id'];
exe_fd = [abspath_gen_cpp_build, '/test-fd'];
exe_im = [abspath_gen_cpp_build, '/test-jsim'];

printf('\nTest of the generated C++ code for Inverse Dynamics\n');
[roy, me] = testcpp_id(robot.roy, exe_id);

printf('\nTest of the generated C++ code for the Joint Space Inertia Matrix\n');
[roy, me] = testcpp_jsim(robot.roy, exe_im);

printf('\nTest of the generated C++ code for Forward Dynamics\n');
[roy, me] = testcpp_fd(robot.roy, exe_fd);
