import logging, argparse

import rmt
import rmt.rmt as rmtool
import kgprim.ct
import ctgen
import ctgen_backends.octave
import ctgen_backends.cpp_iitrbd

import robcogen.core
import robcogen.config
import robcogen.cpp.config
import robmodel

logger = logging.getLogger(__package__)

def cfgLogging(level):
    formatter = logging.Formatter('%(levelname)s (%(name)s) : %(message)s')
    handler = logging.StreamHandler()
    handler.setFormatter(formatter)

    loggers = [logger,
        logging.getLogger(kgprim.ct.__package__),
        logging.getLogger(rmtool.__name__),
        logging.getLogger(robmodel.__package__),
        logging.getLogger(ctgen.__package__),
        logging.getLogger(ctgen_backends.octave.__package__),
        logging.getLogger(ctgen_backends.cpp_iitrbd.__package__)
    ]

    for log in loggers :
        log.setLevel(level)
        log.addHandler(handler)

def main():

    class VersionOptionAction(argparse.Action):
        def __init__(self, option_strings, dest, **kwargs):
            return super().__init__(option_strings, dest, nargs=0, default=argparse.SUPPRESS, **kwargs)
        def __call__(self, parser, namespace, values, option_string, **kwargs):
            print("RobCoGen2 version " + robcogen.__version__)
            parser.exit() # terminates the program too

    argparser = argparse.ArgumentParser(prog='rcg', description='The Robotics Code Generator')
    argparser.add_argument('-v', '--version', action=VersionOptionAction, help='print version information')
    argparser.add_argument('-d', '--verbose', dest='verbose', action='store_true', help='lower the logging level to DEBUG (default is WARN)')
    argparser.add_argument('-o', '--output', dest='output', metavar='DIR', help='base output path (defaults to {defa})'.format(defa=robcogen.config.default_config["outdir"]))

    argparser.add_argument('-b', '--floating-base', dest='floating', action='store_true', help='consider the robot as having a floating underactuated base')

    argparser.add_argument('--cpp',    dest='cpp',   action='store_true', help='generate C++ code')
    argparser.add_argument('--octave', dest='octave',action='store_true', help='generate octave code')
    argparser.add_argument('--tests',  dest='tests', action='store_true', help='generate helper test scripts')

    rmtool.setRobotArgs(argparser)

    cppgroup = argparser.add_argument_group('C++')
    robcogen.cpp.config.add_cmdline_opts(cppgroup)

    args = argparser.parse_args()

    cfgLogging(logging.DEBUG if args.verbose else logging.WARNING)

    loadopts = {
        'dropFixedJoints' : True # for URDF only
    }
    connectivity, tree, frames, geometry, inertia, params = rmtool.getmodels(args.robot, args.params, **loadopts)[0:6]
    configurator = robcogen.config.Configurator(args)
    core = robcogen.core.Generator(configurator, geometry, inertia, None)

    if args.cpp:
        core.generateCPP()
    if args.octave:
        core.generateOctave()
    if args.tests:
        core.generateTestScripts()

