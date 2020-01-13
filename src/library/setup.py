from distutils.core import setup, Extension

with open('VERSION-HASH','r') as _f:
    __version__ = _f.read().strip()

setup(  name            = 'PRUserial485',
        version         = __version__,
        ext_modules=[Extension('PRUserial485',
                                sources=['libPRUserial485.c'],
                                define_macros=[('VERSION-HASH', '"{}"'.format(__version__))],
                                include_dirs = ['/usr/include'],
                                libraries = ['PRUserial485','prussdrv','python3'])])
