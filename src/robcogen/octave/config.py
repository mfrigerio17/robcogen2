import sympy.printing
import robcogen.luabridge
import robcogen.vpc
from robcogen.utils.format import FloatsFormatter


class Configurator:
    def __init__(self, cmdline_args, robot_connectivity_model):
        self.robot = robot_connectivity_model
        self.formatter = FloatsFormatter()
        self.text_cfg = robcogen.luabridge.load_code("return RCG.octave.text_cfg")

        namespaces = []
        for (i,name) in self.text_cfg.namespaces(self.robot).items() :
            namespaces.append(name)
        self.namespaces_ = namespaces

    def getTextTemplatesConfig(self):
        return self.text_cfg

    @property
    def floatsFormatter(self):
        return self.formatter

    @property
    def namespaces(self):
        return self.namespaces_

    def symbolicExpressionToCode(self, symb_expr, replacements):
        return robcogen.vpc.symbolicExpressionToCode(symb_expr, replacements, sympy.printing.octave_code)

