"""KLCserial - Python wrapper to control TL KLC controllers"""

import pkg_resources

from .core import KLC

__version__ = pkg_resources.require("KLCserial")[0].version
__author__ = "iancynk <ian.cynk@posteo.eu>"
__all__ = ['KLC']