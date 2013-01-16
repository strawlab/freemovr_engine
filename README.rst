****
VROS
****

VROS (Virtual reality for the Robot Operating System) manages
multi-computer realtime tracking and display.

.. contents::

Installation
************

(These instructions are modeled after the `ROS kinect package
<http://www.ros.org/wiki/kinect>`_.) These instructions were tested on
Ubuntu Lucid (10.04).

 1. Get the rosinstall tool::

      sudo apt-get install python-stdeb python-yaml
      pypi-install vcstools
      pypi-install rosinstall

 2. `Install ROS Electric Desktop-Full from source
    <http://www.ros.org/wiki/electric/Installation/Ubuntu/Source>`_ to
    ``~/ros``.  (This will install the required stacks, such as
    image_pipeline and executive_smach.) Although installing from
    source will work directly frollowing these instructions, step 4
    below will fail unless you also enable the ROS .deb
    repository. (Specifically, you'll get errors such as ``E: Couldn't
    find package libeigen3-dev``.) Do steps 1.2 and 1.3 as described
    `here
    <http://www.ros.org/wiki/electric/Installation/Ubuntu>`_. Then
    run::

      apt-get update

 3. Get the vros.rosinstall file from github and save to a local
    directory. Manually visit https://github.com/strawlab/vros and
    download the ``vros.rosinstall`` file.

 4. Download VROS into ``~/vros-devel``::

      rosinstall ~/vros-devel ~/ros vros.rosinstall

 5. Build VROS::

      . ~/vros-devel/setup.sh
      rosmake vros --rosdep-install

Note, the above requires installation of package libeigen3-dev on
Ubuntu, which is not available through normal Ubuntu channels. Binaries are available at:
 * http://packages.ros.org/ros/ubuntu/pool/main/e/eigen3/libeigen3-dev_3.0.1-1+ros4~lucid_amd64.deb
 * http://packages.ros.org/ros/ubuntu/pool/main/e/eigen3/libeigen3-dev_3.0.1-1+ros4~lucid_i386.deb

VROS architecture overview
**************************

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
of the physical display used to illuminate a surface. (The VROS
Viewport corresponds to a 2D polygon onto which the image of the
projection screen is shown.)

**Display Surface** - a physical, 2D manifold in 3D space which is
illuminated by a physical display (either by projection or direct
illumination like an LCD screen).
