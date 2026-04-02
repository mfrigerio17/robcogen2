import os, logging, itertools
from types import SimpleNamespace as DictDot

from kgprim.ct.repr.mxrepr import MatrixRepresentation
from kgprim.ct.repr.mxrepr import MatrixReprMetadata
import kgprim.ct.repr.mxrepr as mxrepr

import ctgen
import ctgen.common
import ctgen_backends.cpp_iitrbd.generator as ctgencpp

import robcogen.luabridge as lua
import robcogen.core
import robcogen.vpc
import robcogen.utils.files as fileutils
import robcogen.cpp.config
import robcogen.cpp.templates

logger = logging.getLogger(__name__)


_path_here = fileutils.base_path(__file__)


class Generator:
    def __init__(self, robot, transforms, configurator, fileWriter):
        self.robot = robot
        self.transforms = transforms
        self.configurator = configurator
        self.fileWriter = fileWriter

        # The order of loading modules matter (sometimes), because some modules
        # define global symbols which are used by the other ones
        lua.lua_runtime.execute('generators = {}') # prepare a global
        self._loadLuaModule('utils.lua')
        self._loadLuaModule('common.lua')
        #self._loadLuaModule('tpl_fwd_dyn.lua')
        luat = self._loadLuaModule('generator.lua')

        self.generators = luat.generators(self.robot, transforms, configurator)

    def _loadLuaModule(self, filename):
        luaCodeSrc = open( os.path.join( _path_here, filename), "r")
        ret = lua.lua_runtime.execute(luaCodeSrc.read())
        luaCodeSrc.close()
        return ret

    def _genFile(self, ok, text, filename):
        if not ok :
            logger.error("Template evaluation failed (for file '{dest}'): {err}".format(dest=filename, err=text) )
        else :
            self.fileWriter.genFile( filename, text )

    def coreFiles(self):
        basename = self.configurator.files.h_types
        ok, text = self.generators.headers.types()
        self._genFile(ok, text, self.configurator.headerFileName(basename) )

        basename = self.configurator.files.h_main
        ok, text = self.generators.headers.main()
        self._genFile(ok, text, self.configurator.headerFileName(basename))

        basename = self.configurator.files.h_traits
        ok, text = self.generators.headers.traits()
        self._genFile(ok, text, self.configurator.headerFileName(basename))

        basename = self.configurator.files.h_constants
        ok, text = self.generators.constants.header()
        self._genFile(ok, text, self.configurator.headerFileName(basename) )
        ok, text = self.generators.constants.source()
        self._genFile(ok, text, self.configurator.implFileName(basename) )

        ok, text = self.generators.playground.source()
        self._genFile(ok, text, self.configurator.files.playground + '.cpp')


    def coordinateTransforms(self, ctModelMeta):
        aux = self.generators.constants.readAccessExprForGeometricConstant
        ctgenConfigurator = robcogen.cpp.config.CTGenConfigurator(
             self.robot.kinematics, self.configurator, ctModelMeta.ctModel, aux)

        ctgenerator = ctgencpp.Generator( ctgenConfigurator )

        matricesMetadata = self.transforms.allMatricesMetadata(MatrixRepresentation.homogeneous)

        (okh,header), (oks,source) = ctgenerator.generate_code(ctModelMeta,
                          {MatrixRepresentation.homogeneous : matricesMetadata})
        basename = self.configurator.files.h_transforms
        self._genFile(okh, header, self.configurator.headerFileName(basename) )
        self._genFile(oks, source, self.configurator.implFileName(basename) )


    def inertiaProperties(self):
        basename = self.configurator.files.h_inertia
        ok, text = self.generators.inertia.header()
        self._genFile(ok, text, self.configurator.headerFileName(basename))
        ok, text = self.generators.inertia.source()
        self._genFile(ok, text, self.configurator.implFileName(basename))

    def dynamics(self):
        basename = self.configurator.files.h_fwd_dyn
        ok, text = self.generators.fd.header()
        self._genFile(ok, text, self.configurator.headerFileName(basename))
        ok, text = self.generators.fd.source()
        self._genFile(ok, text, self.configurator.implFileName(basename))

        basename = self.configurator.files.h_inv_dyn
        ok, text = self.generators.id.header()
        self._genFile(ok, text, self.configurator.headerFileName(basename))
        ok, text = self.generators.id.source()
        self._genFile(ok, text, self.configurator.implFileName(basename))

        basename = self.configurator.files.h_jsim
        ok, text = self.generators.jsim.header()
        self._genFile(ok, text, self.configurator.headerFileName(basename))
        ok, text = self.generators.jsim.source()
        self._genFile(ok, text, self.configurator.implFileName(basename))

        basename = self.configurator.files.h_aba_hinv
        ok, text = self.generators.aba_hinv.header()
        self._genFile(ok, text, self.configurator.headerFileName(basename))
        ok, text = self.generators.aba_hinv.source()
        self._genFile(ok, text, self.configurator.implFileName(basename))

    def cmakefile(self):
        ok, text = self.generators.cmake.main()
        self._genFile(ok, text, 'CMakeLists.txt')

    def tests(self):
        originalWriter = self.fileWriter
        self.fileWriter = self.fileWriter.subDir('tests')

        ok, text = self.generators.tests.test_id()
        self._genFile(ok, text, self.configurator.files.test_cmdline_id+'.cpp')

        ok, text = self.generators.tests.test_jsim()
        self._genFile(ok, text, self.configurator.files.test_cmdline_jsim+'.cpp')

        ok, text = self.generators.tests.test_fd()
        self._genFile(ok, text, self.configurator.files.test_cmdline_fd+'.cpp')

        ok, text = self.generators.tests.test_consistency()
        self._genFile(ok, text, self.configurator.files.test_consistency+'.cpp')

        self.fileWriter = originalWriter

    def tests_for_installed_files(self):
        originalWriter = self.fileWriter
        self.fileWriter = self.fileWriter.subDir('tests_installation')

        ok, text = self.generators.tests.test_for_installed_files__consistency()
        self._genFile(ok, text, self.configurator.files.test_consistency+'.cpp')

        ok, text = self.generators.cmake.test_for_installed_files()
        self._genFile(ok, text, 'CMakeLists.txt')

        self.fileWriter = originalWriter

