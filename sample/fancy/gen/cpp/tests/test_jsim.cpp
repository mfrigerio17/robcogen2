#include <iit/robcogen/test/cmdline_jsim.h>
#include <jsim.h>
#include <traits.h>

/**
 * This program calls the generated implementation of the algorithm to calculate
 * the Joint Space Inertia Matrix, and prints it on stdout.
 *
 * It requires all inputs to be given as command line arguments; there are
 * 5 arguments, for the position status of each joint of
 * the robot.
 */
int main(int argc, char** argv)
{
    using namespace fancy::rcg2;

    Transforms xt{ModelParameters()};
    InertiaProperties ip;
    JSIM jsim(ip, xt);

    iit::robcogen::test::cmdline_jsim< Traits >(argc, argv, jsim);
    return 0;
}
