Getting Started
***************

FreemoVR is developed and tested on Ubuntu 16.04 on the amd64 architecture using NVIDIA graphics.

Getting Started
===============

Testing the basic installation
------------------------------

Once FreemoVR is installed, you should be able to run a quick demo by typing::

    # setup ROS environment variables

    # launch the demo
    roslaunch freemovr_engine demo_display_server.launch

If that opens a graphics window showing a 3D scene, FreemoVR is installed and running properly.

Configuring a new display
-------------------------

The most important part of FreemoVR is the Display Server. This is the program that draws on a single
display. If you need multiple physical displays, you will run multiple display servers. (A single display
server can drive multiple virtual displays, as explained in the glossary.) We need to tell the Display
Server about your display.

FreemoVR uses `ROS <http://ros.org>`_ to handle configuration. To bootstrap a new system, begin by
copying a default configuration file into a new location:

    roscd freemovr_engine/config
    cp rosparamconfig.yaml my_display.yaml

Edit this new ``my_display.yaml`` to reflect your display. Much of this `YAML <http://en.wikipedia.org/wiki/YAML>`_
file should be self-explanatory. On initial setup, the most critical information is the contents of the
``display:`` key are the X windows parameters ``displayNum`` and ``screenNum`` and the window geometry parameters
``x``, ``y``, ``width``, ``height``, and ``windowDecoration``. FreemoVR does not switch your graphics mode, so set
these values such that the display server will completely utilize your display.

You can test your new configuration by creating a new ROS launch file which will load these parameters.

    roscd freemovr_engine/launch
    cp demo_display_server.launch my_test.launch

Edit this new ``my_test.launch`` file and change the name of the configuration .yaml file to refer to the file you
created above. Now, run this new launch file:

    roslaunch freemovr_engine my_test.launch

The displayed window should now have the properties you specified in ``my_display.yaml``.

Running the pinhole calibration wizard
--------------------------------------

(To be continued...)
