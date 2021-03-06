from distutils.core import setup, Extension
import subprocess as _sp

with open('VERSION','r') as _f:
    __version__ = _f.read().strip() + ':' + _sp.getoutput('git log --format=%h -1')

setup(  name            = 'PRUserial485',
        version         = __version__,
        description     = """Interface Serial de Alta Performance/Velocidade""",
        author          = "Patricia Nallin",
        author_email    = "patricia.nallin@lnls.br",
        url             = "https://github.com/lnls-sirius/pru-serial485",
        license         = "BSD",
        platforms       = ["Debian Beaglebone"],
        ext_modules     =[Extension('PRUserial485',
                                sources=['libPRUserial485.c'],
                                define_macros=[('VERSIONHASH', '"{}"'.format(__version__))],
                                include_dirs = ['/usr/include'],
                                libraries = ['PRUserial485','prussdrv','python3'])])
