from distutils.core import setup, Extension

with open('VERSION-HASH','r') as _f:
    __version__ = _f.read().strip()

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
                                define_macros=[('VERSION-HASH', '"{}"'.format(__version__))],
                                include_dirs = ['/usr/include'],
                                libraries = ['PRUserial485','prussdrv','python3'])])
