local function load_lua_modules(path)
    RCG.cpp = {
        templates = {
            tests  = loadfile(path .. "/templates/tests.lua") (),
        },
        generators = {
            core_headers       = loadfile(path .. "/templates/core_headers.lua") (),
            constants          = loadfile(path .. "/templates/constants.lua") (),
            inertia_properties = loadfile(path .. "/templates/inertia_properties.lua") (),
            inverse_dynamics   = loadfile(path .. "/templates/inverse_dynamics.lua") (),
            cmake              = loadfile(path .. "/templates/cmake.lua") (),
        },
        common = loadfile(path .. "/common.lua") (),
    }
end


return load_lua_modules
