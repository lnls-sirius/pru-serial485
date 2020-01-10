from distutils.core import setup, Extension
setup(name='prucon', version='1.0',  \
      ext_modules=[Extension('prucon', sources=['libpc.c'], include_dirs = ['/usr/include'],
                    libraries = ['PRUserial485','prussdrv','python3'])])
