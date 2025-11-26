import pathlib
import lupa

import robcogen.utils.files as fileutils
from kgprim.ct.repr.mxrepr import MatrixRepresentation
from robmodel.connectivity import JointKind

lua_runtime = lupa.LuaRuntime(unpack_returned_tuples=True)


def load_code_from_file(full_path):
    with fileutils.open_utf8_reading(full_path) as code:
        ret = lua_runtime.execute(code.read())
        return ret

def load_code(code_text):
    ret = lua_runtime.execute(code_text)
    return ret


# prepare the global symbol table that will be used by the generators
dosetup = lua_runtime.eval('''
function(basepath)
    RCG = {
        enums = {
            MatrixRepresentation = python.eval('MatrixRepresentation'),
            JointKind = python.eval('JointKind'),
        },
        utils = {
            templates = loadfile(basepath .. "/utils/templates.lua")(),
            iters     = loadfile(basepath .. "/utils/iters.lua")(),
        },
    }
end
''')

pathhere = pathlib.Path(__file__).resolve().parent
dosetup( str(pathhere) )
