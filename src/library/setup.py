from distutils.core import setup, Extension
import os as _os

_os.system("(cat VERSION && echo ':' && git log --format=%h -1) | tr -d '\n' > VERSION-HASH")

with open('VERSION-HASH','r') as _f:
    __version__ = _f.read().strip()


setup(  name            = 'PRUserial485',
        version         = __version__,
        ext_modules=[Extension('PRUserial485',
                                sources=['libPRUserial485.c'],
                                include_dirs = ['/usr/include'],
                                libraries = ['PRUserial485','prussdrv','python3'])])
