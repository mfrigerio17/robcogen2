local function load_templates_modules(path)
    RCG.cpp = {
        templates = {
            traits = loadfile(path .. "/traits.lua") (),
            tests  = loadfile(path .. "/tests.lua") (),
        },
    }
end


return load_templates_modules
