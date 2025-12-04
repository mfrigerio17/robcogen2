local function load_templates_modules(path)
    RCG.cpp = {
        templates = {
            tests  = loadfile(path .. "/tests.lua") (),
        },
        generators = {
            core_headers       = loadfile(path .. "/core_headers.lua") (),
            constants          = loadfile(path .. "/constants.lua") (),
            inertia_properties = loadfile(path .. "/inertia_properties.lua") (),
        },
    }
end


return load_templates_modules
