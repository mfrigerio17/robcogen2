import logging

from kgprim.ct.repr.mxrepr import MatrixRepresentation
from kgprim.ct.repr.mxrepr import MatrixReprMetadata
import kgprim.ct.repr.mxrepr as mxrepr

import ctgen_backends.octave as ctgenoctave
import ctgen_backends.octave.backend
import ctgen_backends.octave.generator

import robcogen.luabridge
import robcogen.utils.files as fileutils

import robcogen.octave.templates as templates
import robcogen.octave.config as this_backend_config
import robcogen.octave.roy as roy


logger = logging.getLogger(__name__)
path_here = fileutils.base_path(__file__)

class Generator:
    def __init__(self, robot, transforms, configurator, fileWriter):
        self.robot = robot
        self.transforms = transforms
        self.configurator = configurator
        self.fileWriter = fileWriter.subDir(
                            fileutils.compose_paths(
                                ["+"+d for d in configurator.namespaces ] ))

        self.txtcfg = self.configurator.getTextTemplatesConfig()

        lua_module = robcogen.luabridge.load_code_from_file( path_here.joinpath('generator.lua') )
        self.generators = lua_module.getGenerators(self.robot, transforms, configurator)

    def coreFiles(self):
        ok, text = self.generators.inertia_constants()
        self._genFile(ok, text, self.txtcfg.meta.class_inertia_constants.name+".m" )

        ok, text = self.generators.geometry_parameters()
        self._genFile(ok, text, self.txtcfg.meta.class_geom_parameters.name+".m" )

        ok, text = self.generators.inertia_parameters()
        self._genFile(ok, text, self.txtcfg.meta.class_inertia_parameters.name+".m" )

        ok, text = self.generators.inertia_properties()
        self._genFile(ok, text, self.txtcfg.meta.class_inertia_properties.name+".m" )

        # copy the static files to the destination directory
        path = path_here / "static/RigidBodyInertia.m"
        with fileutils.open_utf8_reading(path) as content:
            self._genFile(True, content.read(), "RigidBodyInertia.m" )

    def coordinateTransformFiles(self):
        lua_config = ctgenoctave.backend.get_codegen_configuration(path_here / "ctgen_config.lua")
        # patch the entry about the variables state
        def variableValueExpression(var):
            joint = self.robot.kinematics.symVarToJoint[var]
            num   = self.robot.tree.jointNum(joint)
            q     = lua_config.variables.status_formal_parameter
            return "{}({})".format(q, num)
        lua_config.variables.value_expression = variableValueExpression
        actualGenerator = ctgenoctave.generator.Generator(lua_config)

        ctModelMeta = self.transforms.modelMetadata
        typeMotion = MatrixRepresentation.spatial_motion
        matricesMetadata = self.transforms.allMatricesMetadata(typeMotion)

        actualGenerator.generate(ctModelMeta, {typeMotion : matricesMetadata}, self.fileWriter.basePath)
        #TODO: call code generation but not _files generation_
        # generate the files here with the local file writer

        # now generate the container class
        ok, text = self.generators.transforms_container(matricesMetadata)
        self._genFile(ok, text, self.txtcfg.meta.class_transforms_container.name+".m" )

    def inverseDynamics(self):
        typeMotion = MatrixRepresentation.spatial_motion
        matricesMetadata = self.transforms.allMatricesMetadata(typeMotion)

        ok, text = self.generators.inverse_dynamics(matricesMetadata)
        self._genFile(ok, text, self.txtcfg.meta.func_inverse_dynamics.name+".m" )

    def jsim(self):
        ok, text = self.generators.jsim()
        self._genFile(ok, text, self.txtcfg.meta.class_jsim.name+".m" )

    def forward_dynamics(self):
        ok, text = self.generators.forward_dynamics()
        self._genFile(ok, text, self.txtcfg.meta.func_forward_dynamics.name+".m" )

    def RoysModel(self):
        xtree_data = roy.buildXTreeData(self.robot)
        ok, text = self.generators.roys_model(xtree_data)
        self._genFile(ok, text, self.txtcfg.meta.func_roys_model.name(self.robot)+".m" )

    def initFunction(self):
        ok, text = self.generators.init_function()
        self._genFile(ok, text, self.txtcfg.meta.func_init.name+".m" )

    def _genFile(self, ok, text, filename):
        if not ok :
            logger.error("Template evaluation failed (for file '{dest}'): {err}".format(dest=filename, err=text) )
        else :
            self.fileWriter.genFile( filename, text )
