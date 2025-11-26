-- Use a local alias for the expected global modules
local genutils  = RCG.utils.templates


local function defs_generators(env, constants)
    --for k,v in pairs(constants) do print(k,v) end
    local class = env.classes.constants
    local scalarTpl = env.common.scalarTpl( class )
    local decl = ''
    local def  = ''
    if env.opts.use_constexpr then
        decl = 'static constexpr «scalar» «name»{«value»};'
        def  = '«tpl»constexpr «scalar» «ns»::«classq»::«name»;'
    else
        decl = 'static const «scalar» «name»;'
        def  = '«tpl»const «scalar» «ns»::«classq»::«name»{«value»};'
    end

    local lenv = {
        tpl = '',
        ns  = env.ns.qualifier,
        class = class,
        classq= class,
        scalar= env.types.scalar,
        ipairs= genutils.poly_ipairs,
    }
    if env.templateAll then
        lenv.classq= scalarTpl.class.in_qualifier
        lenv.tpl   = scalarTpl.heading .. '\n'
    end

    local function code(whichtpl, constant)
        lenv.name  = constant.name
        lenv.value = constant.value
        local ok, code = genutils.tpl_eval(whichtpl, lenv)
        if not ok then error(code) end
        return code
    end

    local class_def_template = [[
«tpl»struct «class» {
@ for _, constant in ipairs(constants) do
    «declare(constant)»
@ end
};
]]
    local definitions_template = [[
    @ for _, constant in ipairs(constants) do
«define(constant)»
    @ end
]]

    lenv.declare = function(constant) return code(decl, constant) end
    lenv.define  = function(constant) return code(def , constant) end
    lenv.constants = constants

    local function class_def()
        local ok, code = genutils.tpl_eval(class_def_template, lenv, {returnTable=true})
        if not ok then error(code) end
        return code
    end
    local function constants_defs()
        local ok, code = genutils.tpl_eval(definitions_template, lenv, {returnTable=true})
        if not ok then error(code) end
        return code
    end
    local function reference(constant, ns_qualified)
        lenv.name = constant.name
        local tpl = '«classq»::«name»'
        if ns_qualified then tpl = '«ns»::' .. tpl end
        local ok, code = genutils.tpl_eval(tpl, lenv)
        if not ok then error(code) end
        return code
    end
    return {
        class_def = class_def,
        constants_defs= constants_defs,
        reference = reference
    }
end



local header_template = [[
#ifndef «include_guard»
#define «include_guard»

#include "«headers.types»"

${ns.open}

${ccgen.class_def()}

${ns.close}

@if templateAll then
#include "«impl_files.constants»"
@end

#endif
]]

local impl_template = [[
@if not templateAll then
#include "«headers.constants»"

using «ns.qualifier»::«types.scalar»;
@end

${ccgen.constants_defs()}

]]

local function allGenerators(robot, configurator, env)
    local allConstants = {}
    for i,c in genutils.poly_ipairs(robot.allConstantsIter()) do
        allConstants[i] = c
    end
    local defs_gens = defs_generators(env, allConstants)
    env.ccgen = defs_gens
    local function gen_header()
        env.include_guard = env.includeGuard(configurator.files.h_constants)
        return genutils.tpl_eval(header_template, env)
    end

    local function gen_implementation()
        return genutils.tpl_eval(impl_template, env)
    end

    return { header= gen_header, impl= gen_implementation, readAccessExpr=defs_gens.reference  }
end

generators.constants = allGenerators
