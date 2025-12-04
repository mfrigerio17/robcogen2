local header_template = [[
#ifndef «include_guard»
#define «include_guard»

#include "«headers.types»"

${ns.open}

«tpl»struct «class» {
@ for _, constant in ipairs(constants) do
    $<constant_declaration>
@ end
};

${ns.close}

@if templateAll then
#include "«impl_files.constants»"
@end

#endif
]]

local impl_template = [[
@ -- when using constexpr, the source file is unnecessary, we leave it empty
@if not meta.constants.use_constexpr then
@   if not templateAll then
#include "«headers.constants»"

using «ns.qualifier»::«types.scalar»;
@   end

@   for _, constant in ipairs(constants) do
$<constant_definition>
@   end
@end
]]

local function generators_constants(robot, configurator, given_env)
    local genutils  = RCG.utils.templates

    -- shallow copy the evaluation environment, and adds required entries
    local env = {}
    for k,v in pairs(given_env) do
        env[k] = v
    end
    env.class = env.meta.constants.class
    env.classqualifier = env.meta.constants.class
    env.tpl = ''
    if env.templateAll then
        local scalarTpl = env.common.scalarTpl( env.meta.constants.class )
        env.classqualifier = scalarTpl.class.in_qualifier
        env.tpl = scalarTpl.heading .. '\n'
    end

    -- Store the constants in a table, because the templates possibly iterate
    -- over the list more than once, thus passing the python iterator will not work
    env.constants = {}
    for i,c in genutils.poly_ipairs(robot.allConstantsIter()) do
        env.constants[i] = c
    end

    -- The sub-templates for the declaration/definition of a constant
    local declaration = 'static constexpr «types.scalar» «constant.name»{«constant.value»};'
    local definition  = ''
    if not env.meta.constants.use_constexpr then
        declaration = 'static const «types.scalar» «constant.name»;'
        definition  = '«tpl»const «types.scalar» «ns.qualifier»::«classqualifier»::«constant.name»{«constant.value»};'
    end


    local function gen_header()
        env.include_guard = env.includeGuard(configurator.files.h_constants)
        return genutils.tpl_eval(header_template, env, {}, {constant_declaration=declaration})
    end

    local function gen_implementation()
        return genutils.tpl_eval(impl_template, env, {}, {constant_definition=definition})
    end

    local function gen_reference(constant, ns_qualified)
        local qualifier = env.classqualifier
        if ns_qualified then qualifier = env.ns.qualifier ..'::'..qualifier end
        return qualifier .. constant.name
    end

    return { header= gen_header, source= gen_implementation, readAccessExpr=gen_reference }
end

return generators_constants
