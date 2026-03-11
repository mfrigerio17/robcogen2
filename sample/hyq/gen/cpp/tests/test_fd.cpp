#include <iit/robcogen/test/cmdline_fd.h>
#include <inertia_properties.h>
#include <transforms.h>
#include <forward_dynamics.h>
#include <traits.h>

/**
 * This program calls the generated implementation of Forward Dynamics, and
 * prints the result (i.e. the joint forces) on stdout.
 *
 * It requires all inputs to be given as command line arguments; there are 54
 * arguments, for the position, velocity and force of each joint of
 * the robot. Group the arguments by type, not by joint.
 */
int main(int argc, char** argv)
{
    using namespace hyq::rcg2;

    Transforms xt{};
    InertiaProperties ip;
    ForwardDynamics solver(ip, xt);

    iit::robcogen::test::cmdline_fd_fb< Traits >(argc, argv, solver);
    return 0;
}
