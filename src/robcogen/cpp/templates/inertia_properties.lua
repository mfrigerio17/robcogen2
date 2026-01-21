local header = [[
#ifndef «include_guard»
#define «include_guard»

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "«headers.main»"
#include "«headers.constants»"

${ns.open}


/**
 * A container for the runtime parameters of the inertia of the robot.
 *
 * Inertia parameters are non-constant inertia-properties, symbolically
 * defined in the robot model.
 * As the value of the parameters must be resolved at runtime, we refer
 * to them as "runtime parameters", "runtime dynamics parameters",
 * "runtime inertia parameters", etc.
 *
 * Unfortunately, the literature commonly refers to the inertia-properties
 * as "inertia parameters". Do not confuse them. In RobCoGen, the parameters
 * are the non-constant values of the properties.
 */
@ local NAMES = meta.inertia_parameters
@if templateAll then
template<typename «types.scalar»>
@end
struct «NAMES.class»
{
@ for i,p in ipairs(parameters) do
    «types.scalar» «NAMES.members.pvalue(p)»;
@ end
    «NAMES.class»() {
        defaults();
    }
    void defaults() {
@ for i,p in ipairs(parameters) do
@   if p.defaultValue ~= nil then
        «NAMES.members.pvalue(p)» = «p.defaultValue»;
@   else
        #error "You MUST change this code to provide a valid default value for your inertia parameters"
        «NAMES.members.pvalue(p)» = 0.0; // TODO change the value!!
@   end
@ end
    }
};


/**
 * A container for the inertial properties of the links of the robot
 */
@local NAMES = meta.inertia_properties
@local CLASS = NAMES.class
«tpl_help.heading»
class «CLASS»
{
@if templateAll then
«typesMacro»
@end
public:
    «CLASS»();
    ~«CLASS»() {};

@for name,link in sorted_links(robot.isFloatingBase) do
    const InertiaMatrix& «NAMES.members.tensorGetter(link)»() const {
        return tensor_«name»;
    }
@ end
@for name,link in sorted_links(robot.isFloatingBase) do
    «types.scalar» «NAMES.members.massGetter(link)»() const {
        return tensor_«name».getMass();
    }
@ end
@for name,link in sorted_links(robot.isFloatingBase) do
    const «types.vec3»& «NAMES.members.comGetter(link)»() const {
        return com_«name»;
    }
@ end
    «types.scalar» getTotalMass() const {
        return
@ for _,fcall,plus in utils.i_iterator_with_separator(utils.i_iterator_decorator(function() return sorted_links(robot.isFloatingBase) end, NAMES.members.massGetter), "+") do
            «fcall»()«plus»
@ end
        ;
    }


    /*!
     * Set new inertia parameters.
     * A change in the parameters triggers the update of the inertia
     * properties modeled by this instance.
     */
    void «NAMES.members.paramsUpdate»(const «meta.inertia_parameters.class»«tpl_help.suffix»&);

private:
    «meta.inertia_parameters.class»«tpl_help.suffix» «NAMES.members.parameters»;

@for name,link in sorted_links(robot.isFloatingBase) do
    InertiaMatrix tensor_«name»;
@ end
@for name,link in sorted_links(robot.isFloatingBase) do
    «types.vec3» com_«name»;
@ end
};

${ns.close}

@if templateAll then
#include "«impl_files.inertia»"
@end

#endif
]]


local source = [[
@if not templateAll then
#include "«headers.inertia»"
@end

@ local NAMES = meta.inertia_properties
@ local CLASS = NAMES.class
@ local qualifier = ns.qualifier .. '::' .. tpl_help.class.in_qualifier
«tpl_help.heading»
«qualifier»::«CLASS»()
{
@for name,link in sorted_links(robot.isFloatingBase) do
@   local ip = inertial_data.byLink(link)
    com_«name» = «types.vec3»(«field_value(ip.com.x)», «field_value(ip.com.y)», «field_value(ip.com.z)»);
    tensor_«name».fill(
        «field_value(ip.mass)»,
        com_«name»,
        «tensor_expression(ip)»
    );
@ end
}

«tpl_help.heading»
void «qualifier»::«NAMES.members.paramsUpdate»(const «meta.inertia_parameters.class»«tpl_help.suffix»& fresh)
{
    «NAMES.members.parameters» = fresh;  // trivial bit-copy is fine
@for link, flags in pairs(robot.inertia.parametric_flags) do
@   local ip = inertial_data.byLink(link)
@   if flags.allParametric() then
    com_«link.name» = «types.vec3»(«field_value(ip.com.x)», «field_value(ip.com.y)», «field_value(ip.com.z)»);
    tensor_«link.name».fill(
        «field_value(ip.mass)»,
        com_«link.name»,
        «tensor_expression(ip)»
    );
@else
@ if flags.parametricMass() then
    tensor_«link.name».changeMass(«field_value(ip.mass)»);
@ end
@ if flags.parametricCoM() then
    com_«link.name» = «types.vec3»(«field_value(ip.com.x)», «field_value(ip.com.y)», «field_value(ip.com.z)»);
    tensor_«link.name».changeCOM(com_«link.name»);
@ end
@ if flags.parametricTensor() then
    tensor_«link.name».changeRotationalInertia(
        «tensor_expression(ip)»
    );
@ end
@ end
@ end
}

]]


local function generators_inertia_properties(robot, configurator, given_env)
    -- shallow copy the template environment
    local env = {}
    for k,v in pairs(given_env) do
        env[k] = v
    end

    local field_value = function( expr )
        if type(expr) == 'number' then
            return expr
        end
        if robot.inertia.isParameter(expr.arg) then
            local pvalue = env.meta.inertia_properties.members.parameters..'.'..
                           env.meta.inertia_parameters.members.pvalue(expr.arg)
            local replacements = {
                [expr.arg.symbol] = pvalue
            }
            return configurator.symbolicExpressionToCode(expr.expr, replacements )
        end
        local replacements = {
            [expr.arg.symbol] = env.common.constantValueAccess(expr.arg)
        }
        return configurator.symbolicExpressionToCode(expr.expr, replacements )
    end

    local tensor_expression = function (ip)
        local ixx = field_value(ip.moments.ixx)
        local ixy = field_value(ip.moments.ixy)
        local ixz = field_value(ip.moments.ixz)
        local iyy = field_value(ip.moments.iyy)
        local iyz = field_value(ip.moments.iyz)
        local izz = field_value(ip.moments.izz)
        return env.ns_iit_rbd.qualifier..'::Utils::buildInertiaTensor<'..
              env.types.scalar..
        '>('..ixx..','..iyy..','..izz..','..ixy..','..ixz..','..iyz..')'
    end

    -- additional fields for the evaluation environment
    env.parameters = {}
    for p,v in python.iter(robot.inertia.parameters) do
        table.insert(env.parameters, p)
    end
    env.inertial_data = robot.inertia.actual_data
    env.include_guard = env.includeGuard(configurator.files.h_inertia)
    env.tpl_help = env.common.scalarTpl( env.meta.inertia_properties.class )
    env.field_value = field_value
    env.tensor_expression = tensor_expression

    return {
        header = function() return RCG.utils.templates.tpl_eval(header, env) end,
        source = function() return RCG.utils.templates.tpl_eval(source, env) end,
    }

end


return generators_inertia_properties


