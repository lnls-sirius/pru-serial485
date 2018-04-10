#!/usr/bin/env python-sirius

"""
setup.py file for PRUserial485
"""


from setuptools import setup


with open('VERSION','r') as _f:
    __version__ = _f.read().strip()


dist = setup (name = 'PRUserial485',
              version = __version__,
              description  = """Interface Serial de Alta Performance/Velocidade""",
              author       = "Patricia Nallin",
              author_email = "patricia.nallin@lnls.br",
              url          = "https://github.com/lnls-sirius/pru-serial485.git",
              packages     = ["PRUserial485"],
              package_data = {'PRUserial485': ['VERSION']},
              license      = "BSD",
              platforms    = ["Debian Beaglebone"],
)
