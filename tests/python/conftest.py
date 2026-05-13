import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_PARENT = os.path.normpath(os.path.join(HERE, "..", "..", "python"))
if PKG_PARENT not in sys.path:
    sys.path.insert(0, PKG_PARENT)
