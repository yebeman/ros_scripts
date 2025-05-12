from pynq import Overlay
from pynq.lib.iic import AxiIIC
import time
import threading

import time
from bno055 import *
import cffi

from ctypes import c_ubyte, POINTER, cast
from IPython.display import clear_output
import sys