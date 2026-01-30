import os
import sympy
from types import SimpleNamespace as DictDot

import ctgen_backends.cpp_iitrbd as ctcppgen
import ctgen_backends.cpp_iitrbd.generator
import ctgen_backends.cpp_iitrbd.config

from robmodel.connectivity import JointKind as JointKind

import robcogen.luabridge as lua
import robcogen.vpc
import robcogen.utils.files as fileutils

def add_cmdline_opts(args):
    args.add_argument('--template', dest='template_all',
                      action='store_const', const=True, default=None,
                      help='template everything on the scalar type')
    args.add_argument('--no-constexpr', dest='no_constexpr',
                      action='store_const', const=True, default=None,
                      help='do NOT use constexpr for the scalar constants')
    args.add_argument('--use-constexpr', dest='no_constexpr',
                      action='store_const', const=False, default=None,
                      help='do use constexpr for the scalar constants')

_lua_config_file_path = os.path.join( os.path.dirname(__file__), 'config.lua')

class Configurator:
    def __init__(self, cmdline_args, robot):
        self.robot  = robot
        self.txtCfg = lua.load_code_from_file(_lua_config_file_path)

        template_all = self.txtCfg.opts.template_all or False
        no_constepxr = self.txtCfg.meta.constants.avoid_constexpr or False
        if cmdline_args is not None:
            template_all = cmdline_args.template_all or template_all
            if cmdline_args.no_constexpr is not None:
                no_constepxr = cmdline_args.no_constexpr

        # re-write the entry in the config dictionary, just in case, to make
        # sure a value is there, possibly the command line override
        self.txtCfg.opts.template_all = template_all
        self.txtCfg.meta.constants.avoid_constexpr = no_constepxr

        self.do_template_all = template_all
        self.avoid_constexpr = no_constepxr


        def spatialVectorIndex(joint):
            if joint.kind == JointKind.prismatic :
                return 'LZ'
            elif joint.kind == JointKind.revolute :
                return 'AZ'
            else :
                raise RuntimeError("Unknow joint type " + joint.kind)

        iitrbd = DictDot(
            namespace = ['iit', 'rbd'],
            spatialVectorIndex = spatialVectorIndex
        )

        files = DictDot(
            h_main  = 'declarations',
            h_types = 'rbd_types',
            h_traits= 'traits',
            h_transforms = 'transforms',
            h_constants = 'constants',
            h_inertia = 'inertia_properties',
            h_fwd_dyn = 'forward_dynamics',
            h_inv_dyn = 'inverse_dynamics',
            h_jsim = 'jsim',
            playground = 'playground',
            test_cmdline_id = 'test_id',
            test_cmdline_jsim = 'test_jsim',
            test_cmdline_fd = 'test_fd',
            test_consistency = 'test_consistency',
        )

        self.data = DictDot(
            outDir = 'cpp',
            files = files,
            iitrbd = iitrbd,
            constantFolding = False
        )

    @property
    def files(self):
        return self.data.files

    @property
    def constantFolding(self):
        return self.data.constantFolding

    def symbolicExpressionToCode(self, symb_expr, replacements):
        return robcogen.vpc.symbolicExpressionToCode(symb_expr, replacements, sympy.printing.cxxcode)

    def templateAll(self): return self.do_template_all

    def avoidConstexpr(self): return self.avoid_constexpr

    def headerFileName(self, base_name) :
        return base_name + '.h'

    def implFileName(self, base_name) :
        if self.templateAll() :
            return base_name + '.cpp.h'
        else :
            return base_name + '.cpp'



class CTGenConfigurator(ctcppgen.config.Configurator):
    def __init__(self, robotKinematics, mainConfigurator, ctModel, readAccessExprForGeometricConstant):
        super().__init__(ctModel, None, None, lua.lua_runtime)

        self.robotKinematics = robotKinematics
        self.mainConfigurator = mainConfigurator

        # Customize the configuration of the Lua generators in the
        # ct-gen package.
        # First, get the default configuration
        ctgen_lua_cfg = super().getTextGeneratorsConfiguration()

        # We will use the local Lua configuration to override some
        # of the defaults of the ct-gen configuration.
        local_lua_cfg = mainConfigurator.txtCfg

        # About the model constants
        ctgen_lua_cfg.constants.value_expression   = readAccessExprForGeometricConstant
        ctgen_lua_cfg.constants.use_constexpr      = not local_lua_cfg.meta.constants.avoid_constexpr
        ctgen_lua_cfg.constants.generate_local_defs= False

        # About the joint state variable
        jointState_t = local_lua_cfg.types.jointState
        if mainConfigurator.templateAll() :
            ctgen_lua_cfg.tpl.template_all = True
        # Constrain the scalar type used internally by the transforms code.
        # Normally, this would not be necessary, as this is a type defined
        # within the transforms class for internal use. However, there is a
        # corner case. We want to configure the variables-status type to be the
        # joint-status type. In the case of templates, this type must have the
        # suffix `<Scalar>`, where 'Scalar' must be the typename valid within
        # the context of the transforms code. By forcing it to be a string that
        # we know, we can also construct the correct string for the joint status
        # type.
            ctgen_lua_cfg.internal.scalar_t = local_lua_cfg.types.scalar
            ctgen_scalar_t = ctgen_lua_cfg.internal.scalar_t
            jointState_t = jointState_t + '<' + ctgen_scalar_t + '>'

        def jointStateAccess(variable):
            joint = robotKinematics.symVarToJoint[variable]
            idx   = mainConfigurator.robot.jointNum(joint)
            return local_lua_cfg.ids.jointStateFormalParameter + "(" + str(idx-1) + ")"

        ctgen_lua_cfg.variables.status_formal_parameter    = local_lua_cfg.ids.jointStateFormalParameter
        ctgen_lua_cfg.variables.value_expression           = jointStateAccess
        ctgen_lua_cfg.variables.status_type                = jointState_t
        ctgen_lua_cfg.variables.generate_local_status_type = False

        ctgen_lua_cfg.scalar_traits.generate_local_def = False
        ctgen_lua_cfg.scalar_traits.use_default        = False
        ctgen_lua_cfg.scalar_traits.type_name = 'iit::rbd::ScalarTraits'

        desired = local_lua_cfg.meta.transforms_container
        current = ctgen_lua_cfg.container_class
        current.generate_it = True
        current.class_name = lambda _ : desired["class"]
        current.members.transform    = desired.members.transform
        current.members.update_params= desired.members.update_params
        current.members.update       = desired.members.update
        current.members.parameters   = desired.members.parameters

        # The namespace configuration for the generators in the
        # `ctgen_backends.cpp_iitrbd` package, is a function taking the
        # transforms model object. In this case, we want to force the
        # locally configured namespaces, regardless of the transforms
        # model. To ensure compatibility, use a lambda that takes a
        #  dummy argument, so that the ctgen generator will not complain.
        ns = local_lua_cfg.namespaces(mainConfigurator.robot)
        ctgen_lua_cfg.namespaces = lambda _ : ns

        files = mainConfigurator.files
        include_files = [files.h_types, files.h_main, files.h_constants]
        includes = ['"{name}.h"'.format(name=file) for file in include_files]
        ctgen_lua_cfg.external.includes = lua.lua_runtime.table_from(includes)

        ctgen_lua_cfg.files.include_dirs = lambda _ : lua.lua_runtime.table_from([])

        self.ctgen_text_generators_configuration = ctgen_lua_cfg

    # @override
    def getTextGeneratorsConfiguration(self):
        return self.ctgen_text_generators_configuration

    # @override
    def generateTemplates(self):
        return self.mainConfigurator.templateAll()

    # @override
    def getOutputFileNames(self):
        basename = self.mainConfigurator.files.h_transforms

        return DictDot(
            header_file = self.mainConfigurator.headerFileName(basename),
            implementation_file = self.mainConfigurator.implFileName(basename),
            include_path = ''
        )



