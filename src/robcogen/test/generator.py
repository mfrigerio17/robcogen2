import pathlib, logging
import robcogen.utils.files as fileutils

logger = logging.getLogger(__name__)

class Generator:
    def __init__(self, robot, lua_runtime):
        path_here = pathlib.Path(__file__).parent

        config = None
        with fileutils.open_utf8_reading(path_here / "config.lua") as code:
            self.txtcfg = lua_runtime.execute(code.read(), name="test/config.lua")

        with fileutils.open_utf8_reading(path_here / "generator.lua") as code:
            f = lua_runtime.execute(code.read(), name="test/generator.lua")
            self.generators = f( robot, self.txtcfg )


    def generate(self, fileWriter):
        def genFile(ok, text, filename):
            if not ok :
                logger.error("Template evaluation failed (for file '{dest}'): {err}".format(dest=filename, err=text) )
            else :
                fileWriter.genFile( filename, text )

        ok, text = self.generators.shell()
        genFile(ok, text, self.txtcfg.meta.shell_script.name)

        ok, text = self.generators.octave()
        genFile(ok, text, self.txtcfg.meta.octave_script.name)
