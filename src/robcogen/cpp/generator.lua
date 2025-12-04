
local function pyIterableToTable(pobject)
    local ret = {}
    for v in python.iter(pobject) do
        table.insert(ret, v)
    end
    return ret
end


-- Use a local alias for the expected global modules
local cpputils  = cpputils
local common    = cppcommon
local genutils  = RCG.utils.templates
local iterutils = RCG.utils.iters
local generators= generators -- this is the global tab where the various Lua files are storing the local generators


local function tpl_test_code(env)
    local tpl = [[
#include <iostream>

#include "«headers.main»"
#include "«headers.types»"
#include "«headers.constants»"
#include "«headers.transforms»"
#include "«headers.inertia»"
#include "«headers.fwd_dyn»"

using namespace std;
using namespace «ns.qualifier»;

int main()
{
    «classes.transforms»<double> tf;
    return 0;
}
]]
    return function()
        return genutils.tpl_eval(tpl, env)
    end
end


local function generator_tests(robot, configurator, env)
    return {
        test_id = function() return genutils.tpl_eval(RCG.cpp.templates.tests.id, env) end,
        test_consistency = function() return genutils.tpl_eval(RCG.cpp.templates.tests.consistency, env) end,
    }
end



local function allGenerators(robot, transforms, configurator)
    local config    = configurator.txtCfg
    local cfgdata   = configurator.data
    local ns        = cpputils.ns_utils( config.namespaces(robot) )
    local ns_iitrbd = cpputils.ns_utils( pyIterableToTable(cfgdata.iitrbd.namespace) )

    local header_file_name = configurator.headerFileName
    local impl_file_name   = configurator.implFileName

    local env = {
        robot = robot,
        vars = config.vars,
        mxops = config.mxops,
        ns = ns,
        ns_iit_rbd = ns_iitrbd,
        types   = config.types,
        classes = config.classes,
        opts    = config.opts,
        meta    = config.meta,
        common = common(robot, transforms, configurator),
        utils = genutils,
        iterutils = iterutils, -- TODO: remove - the iterators should be in this env
        templateAll  = config.opts.template_all,
        includeGuard = config.internal.includeGuard(robot),
        typesMacro   = config.internal.typesMacro,
        headers = {
            main      = header_file_name(configurator.files.h_main),
            types     = header_file_name(configurator.files.h_types),
            constants = header_file_name(configurator.files.h_constants),
            traits    = header_file_name(configurator.files.h_traits),
            transforms= header_file_name(configurator.files.h_transforms),
            inertia   = header_file_name(configurator.files.h_inertia),
            fwd_dyn   = header_file_name(configurator.files.h_fwd_dyn),
            inv_dyn   = header_file_name(configurator.files.h_inv_dyn),
            data_map  = header_file_name("data_map"),
        },
        impl_files = {
            constants = impl_file_name(configurator.files.h_constants),
            transforms= impl_file_name(configurator.files.h_transforms),
            inertia   = impl_file_name(configurator.files.h_inertia),
            inv_dyn   = impl_file_name(configurator.files.h_inv_dyn),
            fwd_dyn   = impl_file_name(configurator.files.h_fwd_dyn),
            tpl_test  = configurator.files.tpl_test .. '.cpp',
            test_cmdline_id = impl_file_name(configurator.files.test_cmdline_id),
            test_consistency = impl_file_name(configurator.files.test_consistency),
        },
        sorted_links = iterutils.sorted_links,
        sorted_links_reversed = iterutils.sorted_links_reversed,
        sorted_joints = iterutils.sorted_joints,
        pairs = genutils.poly_pairs,
        ipairs= genutils.poly_ipairs,
        RCG = RCG,
    }

    local ret = {
        common    = env.common, -- to let the caller inject something, possibly
        headers   = RCG.cpp.generators.core_headers(robot, configurator, env),
        constants = RCG.cpp.generators.constants(robot, configurator, env),
        inertia   = RCG.cpp.generators.inertia_properties(robot, configurator, env),
        --fd        = generators.fd.generators(robot, configurator, env),
        id        = generators.id.generators(robot, configurator, env),
        cmake     = generators.cmake(robot, configurator, env),
        --tpl_test  = tpl_test_code(env),
        tests     = generator_tests(robot, configurator, env),
    }

    return ret
end



return {
    generators = allGenerators
}
