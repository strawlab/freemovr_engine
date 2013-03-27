# This module will be deleted. It is for backwards compatibility. DO NOT EDIT.
from flyvr.display_client import *
import warnings
import os

if int(os.environ.get('FLYVR_THROW_DEPRECATION','0')):
    raise RuntimeError('you are importing a deprecated module')

warnings.warn("You are importing module 'display_client'. Please update this to module 'flyvr.display_client'")
