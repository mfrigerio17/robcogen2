local function load_lua_modules(path)
    RCG.cpp = {
        generators = {
            core_headers       = loadfile(path .. "/templates/core_headers.lua") (),
            constants          = loadfile(path .. "/templates/constants.lua") (),
            inertia_properties = loadfile(path .. "/templates/inertia_properties.lua") (),
            inverse_dynamics   = loadfile(path .. "/templates/inverse_dynamics.lua") (),
            jsim               = loadfile(path .. "/templates/jsim.lua") (),
            forward_dynamics   = loadfile(path .. "/templates/forward_dynamics.lua") (),
            aba_hinv           = loadfile(path .. "/templates/aba_hinv.lua") (),
            tests              = loadfile(path .. "/templates/tests.lua") (),
            cmake              = loadfile(path .. "/templates/cmake.lua") (),
            playground         = loadfile(path .. "/templates/playground.lua") (),
        },
        common = loadfile(path .. "/common.lua") (),
        utils  = loadfile(path .. "/utils.lua") (),
    }
end


return load_lua_modules
