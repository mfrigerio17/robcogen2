local function load_lua_modules(path, path_ctgen)
    RCG.octave = {
        templates = {
            heading            = loadfile(path .. "/templates/heading.lua") (),
            inertia_constants  = loadfile(path .. "/templates/constants.lua") (),
            parameters         = loadfile(path .. "/templates/parameters.lua") (),
            inertia_properties = loadfile(path .. "/templates/inertia.lua") (),
            transforms_container = loadfile(path .. "/templates/transforms_container.lua") (),
            inverse_dynamics   = loadfile(path .. "/templates/inverse_dynamics.lua") (),
            jsim               = loadfile(path .. "/templates/jsim.lua") (),
            forward_dynamics   = loadfile(path .. "/templates/forward_dynamics.lua") (),
            roys_model         = loadfile(path .. "/templates/roy.lua") (),
            init_function      = loadfile(path .. "/templates/init_function.lua") (),
            test_script        = loadfile(path .. "/templates/test_script.lua") (),
        },
        commons = loadfile(path .. "/commons.lua")(),
        text_cfg = loadfile(path .. "/config.lua")(),
    }
    RCG.octave.text_cfg.ctgen = loadfile(path .. "/ctgen_config.lua") ()
    -- CtGen configuration must stay in its own file (it cannot go in config.lua)
    -- because CtGen can be configured only by giving a path to a config file
    -- (passing the runtime configuration table would not work, because of the
    --  different instances of the Lua runtime used by RobCoGen and CtGen)
end


return load_lua_modules
