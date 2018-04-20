import os as _os
import inspect as _inspect
import subprocess as _sp

from .implementation import *


def _get_last_commit_hash():
    """Get commit Hash of the repository of the calling file."""
    fname = _os.path.realpath(_inspect.stack()[1][1])
    path = fname.rpartition(_os.path.sep)[0]
    pwd = _os.path.abspath('.')
    return _sp.getoutput('cd ' + path +
                         '; git log --format=%h -1; cd ' + pwd)


with open(_os.path.join(__path__[0], 'VERSION'), 'r') as _f:
     __version__ = _f.read().strip() + ':' + _get_last_commit_hash()


del implementation  # clean package namespace
