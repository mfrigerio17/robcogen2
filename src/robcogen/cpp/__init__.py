import robcogen.luabridge   as lua
import robcogen.utils.files as fileutils

path_here = fileutils.base_path(__file__)
lua_load  = lua.load_code_from_file(path_here / '__init__.lua')
lua_load( str(path_here) )
