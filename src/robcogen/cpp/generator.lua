
local function pyIterableToTable(pobject)
    local ret = {}
    for v in python.iter(pobject) do
        table.insert(ret, v)
    end
    return ret
end


-- Use a local alias for the expected global modules
local cpputils  = RCG.cpp.utils
local common    = RCG.cpp.common
local genutils  = RCG.utils.templates
local iterutils = RCG.utils.iters


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
            jsim      = header_file_name(configurator.files.h_jsim),
        },
        impl_files = {
            constants = impl_file_name(configurator.files.h_constants),
            transforms= impl_file_name(configurator.files.h_transforms),
            inertia   = impl_file_name(configurator.files.h_inertia),
            inv_dyn   = impl_file_name(configurator.files.h_inv_dyn),
            fwd_dyn   = impl_file_name(configurator.files.h_fwd_dyn),
            jsim      = impl_file_name(configurator.files.h_jsim),
            playground= configurator.files.playground .. '.cpp',
            test_cmdline_id = configurator.files.test_cmdline_id .. '.cpp',
            test_cmdline_jsim = configurator.files.test_cmdline_jsim .. '.cpp',
            test_cmdline_fd = configurator.files.test_cmdline_fd .. '.cpp',
            test_consistency = configurator.files.test_consistency .. '.cpp',
        },
        sorted_links           = iterutils.get_sorted_links_iter_factory(robot),
        sorted_links_reversed  = iterutils.get_sorted_links_iter_factory(robot, true),
        sorted_joints          = iterutils.get_sorted_joints_iter_factory(robot),
        sorted_joints_reversed = iterutils.get_sorted_joints_iter_factory(robot, true),
        pairs = genutils.poly_pairs,
        ipairs= genutils.poly_ipairs,
        RCG = RCG,
    }

    env.leafs = {}
    for _, link in env.sorted_links() do
        if robot.treeutils.isLeaf(link) then
            table.insert(env.leafs, link)
        end
    end

    local generators_constants = RCG.cpp.generators.constants(robot, configurator, env)
    local ret = {
        common    = env.common, -- to let the caller inject something, possibly
        headers   = RCG.cpp.generators.core_headers(robot, configurator, env),
        constants = generators_constants,
        inertia   = RCG.cpp.generators.inertia_properties(robot, configurator, env, generators_constants.readAccessExprForInertiaConstant),
        id        = RCG.cpp.generators.inverse_dynamics(robot, configurator, env),
        jsim      = RCG.cpp.generators.jsim(robot, configurator, env),
        fd        = RCG.cpp.generators.forward_dynamics(robot, configurator, env),
        cmake     = RCG.cpp.generators.cmake(robot, configurator, env),
        tests     = RCG.cpp.generators.tests(robot, configurator, env),
        playground= RCG.cpp.generators.playground(robot, configurator, env),
    }

    return ret
end



return {
    generators = allGenerators
}
