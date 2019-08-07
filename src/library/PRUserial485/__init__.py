import os as _os
import inspect as _inspect
import subprocess as _sp
import sys
from .implementation import *

PYTHON3 = "3"
PRUserial485_files_path = "/root/pru-serial485"

if(sys.version[0] == PYTHON3):
  # Needed only for Python3
  def _get_last_commit_hash():
      """Get commit Hash of the repository of the calling file."""
      fname = _os.path.realpath(_inspect.stack()[1][1])
      #path = fname.rpartition(_os.path.sep)[0]
      path = PRUserial485_files_path
      pwd = _os.path.abspath('.')
      return _sp.getoutput('cd ' + path +
                           '; git log --format=%h -1; cd ' + pwd)


  with open(_os.path.join(__path__[0], 'VERSION'), 'r') as _f:
       __version__ = _f.read().strip() + ':' + _get_last_commit_hash()


  del implementation  # clean package namespace
