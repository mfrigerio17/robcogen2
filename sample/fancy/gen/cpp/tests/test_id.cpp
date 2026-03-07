#include <iit/robcogen/test/cmdline_id.h>
#include <inertia_properties.h>
#include <transforms.h>
#include <inverse_dynamics.h>
#include <traits.h>

/**
 * This program calls the generated implementation of Inverse Dynamics, and
 * prints the result (i.e. the joint forces) on stdout.
 *
 * It requires all inputs to be given as command line arguments; there are 15
 * arguments, for the position, velocity and acceleration of each joint of
 * the robot. Group the arguments by type, not by joint.
 */
int main(int argc, char** argv)
{
    using namespace fancy::rcg2;

    Transforms xt{ModelParameters()};
    InertiaProperties ip;
    InverseDynamics id(ip, xt);

    iit::robcogen::test::cmdline_id< Traits >(argc, argv, id);
    return 0;
}
