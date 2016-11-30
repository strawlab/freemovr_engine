# This module will be deleted. It is for backwards compatibility. DO NOT EDIT.
from freemoovr.plot_utils import *
import warnings
import os

if int(os.environ.get('FREEMOOVR_THROW_DEPRECATION','0')):
    raise RuntimeError('you are importing a deprecated module')

warnings.warn("You are importing module 'plot_utils'. Please update this to module 'freemoovr.plot_utils'")
