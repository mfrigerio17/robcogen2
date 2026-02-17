
local template_shell = [[
if [ "$#" -ne 4 ]; then
  echo "Usage: $0 <PATH to RobCoGen root> <PATH to spatial_v2 root> <PATH to generated C++> <PATH to generated Octave>" >&2
  exit 1
fi
if ! [ -d "$1" ]; then
  echo "$1 not a directory" >&2
  exit 1
fi
if ! [ -d "$2" ]; then
  echo "$2 not a directory" >&2
  exit 1
fi
if ! [ -d "$3" ]; then
  echo "$3 not a directory" >&2
  exit 1
fi
if ! [ -d "$4" ]; then
  echo "$4 not a directory" >&2
  exit 1
fi

abspath_rcg_root=$(cd $1 && pwd -P)
abspath_spatial_v2=$(cd $2 && pwd -P)
abspath_gen_cpp=$(cd $3 && pwd -P)
abspath_gen_octave=$(cd $4 && pwd -P)

origin=$(cd -- "$(dirname -- "$0")" &> /dev/null && pwd)
dest=`mktemp --directory`

cd $dest && cmake "$abspath_gen_cpp" && make -j3
if [ $? != 0 ]
then
    echo "Failed to build the C++ code"
    exit 1
fi

cd $origin

echo "Running C++ consistency tests..."
$dest/test-cons

echo "Running tests for both C++ and Octave ..."
octave --quiet «self.octave_script.name» $abspath_spatial_v2 "$abspath_rcg_root/testing/src" $abspath_gen_octave $dest
]]

local template_octave = [[
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

robot     = «gen_octave.namespace»init();
robot.roy = «gen_octave.namespace»«gen_octave.roy_model_func»(robot.mgc, robot.mgp, robot.mic, robot.mip);

printf('\nTest of the generated Octave code for Inverse Dynamics\n');
[roy, me] = «tests.id»(robot.roy, robot);

printf('\nTest of the generated Octave code for the JSIM\n');
[roy, me] = «tests.H»(robot.roy, robot);

printf('\nTest of the generated Octave code for Forward Dynamics\n');
[roy, me] = «tests.fd»(robot.roy, robot);


exe_id = [abspath_gen_cpp_build, '/test-id'];
exe_fd = [abspath_gen_cpp_build, '/test-fd'];
exe_im = [abspath_gen_cpp_build, '/test-jsim'];

printf('\nTest of the generated C++ code for Inverse Dynamics\n');
[roy, me] = «testscpp.id»(robot.roy, exe_id);

printf('\nTest of the generated C++ code for the Joint Space Inertia Matrix\n');
[roy, me] = «testscpp.H»(robot.roy, exe_im);

printf('\nTest of the generated C++ code for Forward Dynamics\n');
[roy, me] = «testscpp.fd»(robot.roy, exe_fd);
]]


local function get_generators(robot, config)

    local genutils  = RCG.utils.templates
    local octavecfg = RCG.octave.text_cfg

    local namespace = table.concat(octavecfg.namespaces(robot), ".")
    if namespace ~= "" then
        namespace = namespace .. "."
    end

    local env = {
        self = config.meta,
        gen_octave = {
            roy_model_func = octavecfg.meta.func_roys_model.name(robot),
            namespace = namespace,
        },
        tests = {
            id = "test_id",
            H  = "test_jsim",
            fd = "test_fd",
        },
        testscpp = {
            id = "testcpp_id",
            H  = "testcpp_jsim",
            fd = "testcpp_fd",
        },
    }

    if robot.isFloatingBase then
        for k,v in pairs(env.tests) do
            env.tests[k] = v .. "_fb"
        end
        for k,v in pairs(env.testscpp) do
            env.testscpp[k] = v .. "_fb"
        end
    end

    local genshell = function()
        local ok,text = genutils.tpl_eval(template_shell, env)
        return ok,text
    end

    local genoctave = function()
        local ok,text = genutils.tpl_eval(template_octave, env)
        return ok,text
    end

    return {
        shell = genshell,
        octave = genoctave,
    }
end

return get_generators
