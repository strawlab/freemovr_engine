.. image:: https://strawlab.org/assets/freemovr/freemovr-principle.png
    :alt: FreemoVR
    :width: 446
    :height: 184

*********************************
FreemoVR - virtual reality engine
*********************************

FreemoVR is a virtual reality engine built on `ROS <http://ros.org>`_ and
`OpenSceneGraph <http://www.openscenegraph.org>`_. It manages
multi-computer realtime tracking and display with the goal of being
useful for scientific studies of vision and behavior.

Discussion
==========

For questions or discussion, please use `the "freemovr" Google
Group <https://groups.google.com/forum/#!forum/freemovr>`_.

Installation
============

For installation, we recommend using
`our Ansible playbooks <https://github.com/strawlab/strawlab-ansible-roles.git>`_.
In particular, the "ros-kinetic-freemovr-engine" role or the "ros-kinetic-freemovr"
role install either the FreemoVR engine alone or a
`full FreemoVR system, including flydra <https://strawlab.org/freemovr>`_ (on
Ubuntu 16.04 with ROS Kinetic).

CUDA support
============

Stimulus plugins that make use of CUDA can be used if the osgcompute package is
present at compilation time.

License
=======

With the exception of third party software (see "other parts" in LICENSE.txt),
the software, documentation and other resouces are licensed under either of

* Apache License, Version 2.0,
  (./LICENSE-APACHE or http://www.apache.org/licenses/LICENSE-2.0)
* MIT license (./LICENSE-MIT or http://opensource.org/licenses/MIT)
  at your option.

Contribution
============

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual licensed as above, without any additional terms or
conditions.

Code of conduct
===============

Anyone who interacts with freemovr_engine in any space including but not limited
to this GitHub repository is expected to follow our [code of
conduct](https://github.com/strawlab/freemovr_engine/blob/master/code_of_conduct.md).


.. contents::

* `installation and getting started <docs/getting_started.rst>`_
* `using the joystick for input <docs/joystick.rst>`_

Theory of operation
===================

A moving observer has a pose within a global coordinate frame. Objects
within the global frame may also move or be updated (e.g. a moving
grating). Six camera views with a fixed relationship to the observer
are used to build a cube map, showing the scene surrounding the
observer without regard to the projection surface.

This cube map is then projected onto a 3D shape model of the display
surface. From there, this image is warped to the physical display
output.

freemovr_engine nodes
=====================

display_server - the FreemoVR display server
--------------------------------------------

The FreemoVR display server node runs locally on the computer(s) connected
to the physical display. During a typical experiment, it will be
running an experiment plugin. Each experiment plugin updates the
graphics engine on the basis of the fly's current position. Given the
scenegraph and the calibrated screen layout, the node will compute the
images shown on the projectors.

viewport_definer.py - FreemoVR viewport definer
-----------------------------------------------

Runs a GUI program that allows the user to interactively define the
viewports for all connected projectors.

Glossary
========

**Display Coordinates** - the native pixel indices on a physical
display. These are 2D.

**World Coordinates** - the 3D coordinates in lab space of physical
(or simulated) points. (May also be represented as a `4D homogeneous
vector <http://en.wikipedia.org/wiki/Homogeneous_coordinates>`_
*x,y,z,w* with nonzero *w*.)

**Physical Display** - a physical device capable of emitting a large,
rectangluar block of pixels. It has display coordinates - the 2D
locations of each pixel. (A physical display does not have world
coordinates used for the VR mathematics. On the other hand, A virtual
display does have world coordinates.)

**Virtual Display** - a model of a physical display which relates
world coordinates to display coordinates. The model consists of a
linear pinhole projection model, a non-linear warping model for lens
distortions, viewport used to clip valid display coordinates, 3D
display surface shape in world coordinates, and luminance
masking/blending. Note that a physical display can have multiple
virtual displays, for example, if a projector shines onto mirrors that
effectively create multiple projections.

**Viewport** - vertices of polygon defining projection region in
display coordinates (x0,y0,x1,y1,...). It is used to limit the region
of the physical display used to illuminate a surface. (The FreemoVR
Viewport corresponds to a 2D polygon onto which the image of the
projection screen is shown.)

**Display Surface** - a physical, 2D manifold in 3D space which is
illuminated by a physical display (either by projection or direct
illumination like an LCD screen).

Developing
==========

When developing a stimulus, you can launch the display_server
with that stimulus loaded like the following

``./bin/display_server --stimulus lib/libStimulusLatencyTimestamp.so``

