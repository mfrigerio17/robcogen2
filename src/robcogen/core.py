import robcogen.models
import robcogen.cpp as cpp
import robcogen.cpp.config, robcogen.cpp.generator

import robcogen.octave as octave
import robcogen.octave.generator
import robcogen.octave.config

class Generator:
    def __init__(self, configurator, robotGeometry, robotInertia, userDesiredTransforms):
        self.robot      = robcogen.models.RobotModel(robotGeometry, robotInertia, configurator.floatingBase)
        self.transforms = robcogen.models.TransformsModelWrapper(self.robot, userDesiredTransforms)
        self.config     = configurator
        self.fileWriter = configurator.getFileGenerator(robotGeometry.connectivityModel)

    def generateCPP(self):
        cppconfig  = cpp.config.Configurator(self.config.cmdline_args, self.robot.tree)
        cppgen     = cpp.generator.Generator(self.robot, self.transforms, cppconfig, self.fileWriter)

        cppgen.coordinateTransforms(self.transforms.ctModelMeta)
        cppgen.coreFiles()
        cppgen.inertiaProperties()
        cppgen.dynamics()
        cppgen.cmakefile()
        cppgen.tests()

    def generateOctave(self):
        octconfig = octave.config.Configurator(self.config.cmdline_args, self.robot.tree)
        octgen    = octave.generator.Generator(self.robot, self.transforms, octconfig, self.fileWriter)

        octgen.coreFiles()
        octgen.coordinateTransformFiles()
        octgen.inverseDynamics()
        octgen.RoysModel()
        octgen.initFunction()
