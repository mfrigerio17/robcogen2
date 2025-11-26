local id = [[
#include <iit/robcogen/test/cmdline_id.h>
#include <«headers.transforms»>  // TODO add the installation path
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
    Transforms xt{ModelParameters()};
@else
    Transforms xt{};
@end
    Traits::InertiaProperties ip;
    Traits::InvDynEngine      id(ip, xt);

@if robot.isFloatingBase then
    iit::robcogen::test::cmdline_id_fb< Traits >(argc, argv, id);
@else
    iit::robcogen::test::cmdline_id< Traits >(argc, argv, id);
@end
    return 0;
}
]]

local consistency = [[
#include <iit/robcogen/test/dynamics_consistency.h>
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
    Transforms xt{ModelParameters()};
@else
    Transforms xt{};
@end
    Traits::InertiaProperties ip;
    Traits::InvDynEngine      id(ip, xt);

@if robot.isFloatingBase then
    iit::robcogen::test::floatingBaseID< Traits >(id);
@else
    iit::robcogen::test::fixedBaseID<Traits>(id);
@end

    return 0;
}
]]

return {
    id = id,
    consistency = consistency,
}
