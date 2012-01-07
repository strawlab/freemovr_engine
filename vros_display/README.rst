************
vros_display
************

vros_display nodes
==================

display_server - the VROS display server
----------------------------------------

The VROS display server node runs locally on the computer(s) connected
to the physical display. During a typical experiment, it will be
running an experiment plugin. Each experiment plugin updates the
graphics engine on the basis of the fly's current position. Given the
scenegraph and the calibrated screen layout, the node will compute the
images shown on the projectors.

viewport_definer.py - VROS viewport definer
-------------------------------------------

Runs a GUI program that allows the user to interactively define the
viewports for all connected projectors.
