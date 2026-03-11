#include <iit/robcogen/test/dynamics_consistency.h>
#include <inertia_properties.h>
#include <transforms.h>
#include <inverse_dynamics.h>
#include <jsim.h>
#include <forward_dynamics.h>
#include <traits.h>

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
    using namespace hyq::rcg2;
    Transforms xt{};
    InertiaProperties ip;
    InverseDynamics id(ip, xt);
    JSIM jsim(ip, xt);
    ForwardDynamics fd(ip, xt);

    iit::robcogen::test::floatingBaseID< Traits >(id);
    iit::robcogen::test::floatingBaseJSIM< Traits >(id, jsim);
    iit::robcogen::test::floatingBaseFD< Traits >(fd, id);

    return 0;
}
