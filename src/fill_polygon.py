# This module will be deleted. It is for backwards compatibility. DO NOT EDIT.
from flyvr.fill_polygon import *
import warnings
import os

if int(os.environ.get('FLYVR_THROW_DEPRECATION','0')):
    raise RuntimeError('you are importing a deprecated module')

warnings.warn("You are importing module 'fill_polygon'. Please update this to module 'flyvr.fill_polygon'")
