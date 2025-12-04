local template = [[
#include <iostream>

#include "«headers.main»"
#include "«headers.types»"
#include "«headers.constants»"
#include "«headers.transforms»"
#include "«headers.inertia»"
#include "«headers.traits»"

using namespace std;
using namespace «ns.qualifier»;

// Use this file as an example on how to use the generated code.
// You can also modify it directly, but beware that it will be overwritten the
//  next time you generate the code.

int main()
{
@if templateAll then
    «classes.transforms»<double> xt;
    «meta.inertia_properties.class»<double> ip;
    Traits<double>::«types.jointState» q;
@else
@--TODO: fix next line, it should be meta.transforms.class or so, but we
@-- havent harmonized configuration of CtGen yet...
    «classes.transforms» xt;
    «meta.inertia_properties.class» ip;
@end
    return 0;
}
]]


local function generator_playground(robot, configurator, env)
    return {
        source = function() return RCG.utils.templates.tpl_eval(template, env) end
    }
end


return generator_playground
