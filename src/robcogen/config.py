import pathlib
from robcogen.utils.files import FileWriter

default_config = {
    'outdir' : '/tmp/rcgen2'
}

class Configurator:
    '''
    Robcogen core-configuration interface.
    Independent of language backends.
    '''
    def __init__(self, user_overrides):
        self.data = {}

        # command-line overrides
        # TODO: complete
        outdir = user_overrides.output or default_config['outdir']
        self.data['outdir'] = pathlib.Path(outdir)

        self.data['floating_base'] = user_overrides.floating or False
        self.cmdline_args = user_overrides

    def getFileGenerator(self, robot):
        return FileWriter(self.getOutputPath(robot))

    def getOutputPath(self, robot):
        return self.data['outdir']

    @property
    def floatingBase(self):
        return self.data['floating_base']


