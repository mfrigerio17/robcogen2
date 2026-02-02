local id = [[
#include <iit/robcogen/test/cmdline_id.h>
#include <«headers.inertia»>
#include <«headers.transforms»>
#include <«headers.inv_dyn»> // TODO add the installation path
#include <«headers.traits»>  // TODO add the installation path

/**
 * This program calls the generated implementation of Inverse Dynamics, and
 * prints the result (i.e. the joint forces) on stdout.
 *
 * It requires all inputs to be given as command line arguments; there are «3*robot.DOFs»
 * arguments, for the position, velocity and acceleration of each joint of
 * the robot. Group the arguments by type, not by joint.
 */
int main(int argc, char** argv)
{
    using namespace «ns.qualifier»;

@if robot.hasParametricGeometry then
    «meta.transforms_container.class»«tplscalar» xt{ModelParameters«tplscalar»()};
@else
    «meta.transforms_container.class»«tplscalar» xt{};
@end
    «meta.inertia_properties.class»«tplscalar» ip;
    «meta.inverse_dynamics.class»«tplscalar» id(ip, xt);

@if robot.isFloatingBase then
    iit::robcogen::test::cmdline_id_fb< Traits«tplscalar» >(argc, argv, id);
@else
    iit::robcogen::test::cmdline_id< Traits«tplscalar» >(argc, argv, id);
@end
    return 0;
}
]]

local jsim = [[
#include <iit/robcogen/test/cmdline_jsim.h>
#include <«headers.jsim»> // TODO add the installation path
#include <«headers.traits»>  // TODO add the installation path

/**
 * This program calls the generated implementation of the algorithm to calculate
 * the Joint Space Inertia Matrix, and prints it on stdout.
 *
 * It requires all inputs to be given as command line arguments; there are
 * «robot.DOFs» arguments, for the position status of each joint of
 * the robot.
 */
int main(int argc, char** argv)
{
    using namespace «ns.qualifier»;

@if robot.hasParametricGeometry then
    «meta.transforms_container.class»«tplscalar» xt{ModelParameters«tplscalar»()};
@else
    «meta.transforms_container.class»«tplscalar» xt{};
@end
    «meta.inertia_properties.class»«tplscalar» ip;
    «meta.jsim.class»«tplscalar» jsim(ip, xt);

    iit::robcogen::test::cmdline_jsim< Traits«tplscalar» >(argc, argv, jsim);
    return 0;
}
]]

local consistency = [[
#include <iit/robcogen/test/dynamics_consistency.h>
#include <«headers.inertia»>
#include <«headers.transforms»>
#include <«headers.inv_dyn»>
#include <«headers.jsim»>
#include <«headers.traits»>  // TODO add the installation path

/**
 * This program calls the dynamics-consistency-test implemented in
 * iit::robcogen::test.
 *
 * No arguments are required, all the relevant joint-space quantities are
 * randomly generated.
 *
 * The test prints some output on stdout. All the numerical values should be
 * zero; if that is not the case, there is some inconsistency among the generated
 * dynamics algorithms.
 */
int main(int argc, char** argv)
{
    using namespace «ns.qualifier»;
@if robot.hasParametricGeometry then
    «meta.transforms_container.class»«tplscalar» xt{ModelParameters«tplscalar»()};
@else
    «meta.transforms_container.class»«tplscalar» xt{};
@end
    «meta.inertia_properties.class»«tplscalar» ip;
    «meta.inverse_dynamics.class»«tplscalar» id(ip, xt);
    «meta.jsim.class»«tplscalar» jsim(ip, xt);


@if robot.isFloatingBase then
    iit::robcogen::test::floatingBaseID< Traits«tplscalar» >(id);
    iit::robcogen::test::floatingBaseJSIM< Traits«tplscalar» >(id, jsim);
@else
    iit::robcogen::test::fixedBaseID< Traits«tplscalar» >(id);
    iit::robcogen::test::fixedBaseJSIM< Traits«tplscalar» >(id, jsim);
@end

    return 0;
}
]]



local function generator_tests(robot, configurator, env)
    env.tplscalar = ''
    if configurator.templateAll() then
        env.tplscalar = '<double>'
    end
    return {
        test_id = function() return RCG.utils.templates.tpl_eval(id, env) end,
        test_jsim = function() return RCG.utils.templates.tpl_eval(jsim, env) end,
        test_consistency = function() return RCG.utils.templates.tpl_eval(consistency, env) end,
    }
end



return generator_tests
