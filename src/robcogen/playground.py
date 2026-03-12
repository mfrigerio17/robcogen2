import robcogen.luabridge as lua

def playground(core_generator):
    '''
    A function for manual testing, experiments and debugging, with access to the
    core generator object created after loading the input model.
    Execute this function by passing the '--secret' switch at the command line.
    '''

    # more often than not you will want to investigate the robot model
    robot = core_generator.robot

    # you can run Python code but also see what happens with Lua
    code = '''
local iters = RCG.utils.iters

local function playground(robot)
    -- this is an example
    sorted_links = iters.get_sorted_links_iter_factory(robot)

    for name,link in sorted_links() do
        print(name, link)
    end

end

return playground
'''
    f = lua.load_code(code)
    f(core_generator.robot)
