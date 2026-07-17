import os
import sys

# Prefer an installed justblend; fall back to the in-tree package, which dev
# builds populate with _core*.so via CMake's LIBRARY_OUTPUT_DIRECTORY.
try:
    import justblend  # noqa: F401
except ImportError:
    HERE = os.path.dirname(os.path.abspath(__file__))
    PKG_PARENT = os.path.normpath(os.path.join(HERE, "..", "..", "python"))
    if PKG_PARENT not in sys.path:
        sys.path.insert(0, PKG_PARENT)
