'''
This is the package of the "Octave" backend of RobCoGen2
'''

import robcogen.luabridge   as lua
import robcogen.utils.files as fileutils

import ctgen_backends.octave as ctgenoctave

path_here = fileutils.base_path(__file__)
load_lua_modules = lua.load_code_from_file(path_here / '__init__.lua')

path_ctgen = fileutils.base_path(ctgenoctave.__file__)

load_lua_modules(str(path_here), str(path_ctgen))
