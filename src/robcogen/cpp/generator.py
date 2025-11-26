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
import robcogen.constants
import robcogen.vpc
import robcogen.utils.files as fileutils
import robcogen.cpp.config
import robcogen.cpp.templates

logger = logging.getLogger(__name__)
homrepr = MatrixRepresentation.homogeneous




_path_here = fileutils.base_path(__file__)


class Generator:
    def __init__(self, robot, transforms, configurator, fileWriter):
        self.robot = robot
        self.configurator = configurator
        self.fileWriter = fileWriter

        # The order of loading modules matter (sometimes), because some modules
        # define global symbols which are used by the other ones
        lua.lua_runtime.execute('generators = {}') # prepare a global
        self._loadLuaModule('utils.lua')
        self._loadLuaModule('common.lua')
        self._loadLuaModule('tpl_core_headers.lua')
        self._loadLuaModule('tpl_constants.lua') # also defines the function to generate the reference to a constant
        self._loadLuaModule('tpl_inertia.lua')
        #self._loadLuaModule('tpl_fwd_dyn.lua')
        self._loadLuaModule('tpl_inv_dyn.lua')
        self._loadLuaModule('tpl_cmake.lua')
        luat = self._loadLuaModule('generator.lua')

        self.generators = luat.generators(self.robot, transforms, configurator)

        self.constValueExprGenerator = self._getConstantValueExprGenerator()
        self.generators.common.constantValueAccess = self.constValueExprGenerator


    def _getConstantValueExprGenerator(self):
        mc = self.configurator.txtCfg.classes.constants
        scalar_t = self.configurator.txtCfg.types.scalar

        getter = None
        if self.configurator.templateAll() :
            getter = lambda constant: ('{mc}<{scalar}>::{name}'.format(mc=mc, scalar=scalar_t, name=constant.name))
        else :
            getter = lambda constant : (mc + '::' + constant.name)
            # ignore the scalar argument
        aux = DictDot(
            valueExpression = getter,
            piExpression    = lambda ___ : 'std::M_PI'
        )
        ca = robcogen.constants.ConstantsAccess(aux, self.configurator.constantFolding)
        return ca.valueExpression


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
        ok, text = self.generators.constants.impl()
        self._genFile(ok, text, self.configurator.implFileName(basename) )

        if self.configurator.templateAll() :
            ok, text = self.generators.tpl_test()
            self._genFile(ok, text, self.configurator.files.tpl_test + '.cpp')

        fname = "data_map.h"
        path = _path_here / "static" / fname
        with fileutils.open_utf8_reading(path) as content:
            self.fileWriter.genFile( fname, content.read() )
        #path.copy( self.fileWriter.basePath / fname ) # requires python 3.14

    def coordinateTransforms(self, ctModelMeta):
        ctgenConfigurator = robcogen.cpp.config.CTGenConfigurator(self.robot.kinematics, self.configurator, ctModelMeta.ctModel)
        luacfg = ctgenConfigurator.getTextGeneratorsConfiguration()
        luacfg.constants.value_expression = self.constValueExprGenerator
        ctgenerator = ctgencpp.Generator( ctgenConfigurator )

        allMxMeta = {}
        for ctMeta in ctModelMeta.transformsMetadata :
            MX     = mxrepr.hCoordinatesSymbolic(ctMeta.ct)
            mxMeta = MatrixReprMetadata(ctMeta, MX, MatrixRepresentation.homogeneous)
            allMxMeta[ctMeta.name] = mxMeta

        (okh,header), (oks,source) = ctgenerator.generate_code(ctModelMeta, {homrepr : allMxMeta})
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
        # basename = self.configurator.files.h_fwd_dyn
        # ok, text = self.generators.fd.header()
        # self._genFile(ok, text, self.configurator.headerFileName(basename))
        # ok, text = self.generators.fd.impl()
        # self._genFile(ok, text, self.configurator.implFileName(basename))

        basename = self.configurator.files.h_inv_dyn
        ok, text = self.generators.id.header()
        self._genFile(ok, text, self.configurator.headerFileName(basename))
        ok, text = self.generators.id.source()
        self._genFile(ok, text, self.configurator.implFileName(basename))

    def cmakefile(self):
        ok, text = self.generators.cmake()
        self._genFile(ok, text, 'CMakeLists.txt')

    def tests(self):
        originalWriter = self.fileWriter
        self.fileWriter = self.fileWriter.subDir('tests')

        ok, text = self.generators.tests.test_id()
        self._genFile(ok, text, self.configurator.implFileName(self.configurator.files.test_cmdline_id))

        ok, text = self.generators.tests.test_consistency()
        self._genFile(ok, text, self.configurator.implFileName(self.configurator.files.test_consistency))


        self.fileWriter = originalWriter
