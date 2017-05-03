#!/usr/bin/env python

"""
setup.py file for PRUserial485
"""


from setuptools import setup


dist = setup (name = 'PRUserial485',
              version = '1.0',
              description = """Interface Serial de Alta Performance/Velocidade""",
              author      = "Patricia Nallin",
              author_email= "patricia.nallin@lnls.br",
              url         = "https://git.cnpem.br/patricia.nallin/PRUserial485.git",
              packages    = ["PRUserial485"],
              license     = "BSD",
              platforms   = ["Debian Beaglebone"],
)

