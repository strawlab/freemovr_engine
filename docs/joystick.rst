Using the joystick for input
****************************

Several FreemooVR programs use a joystick for input (e.g. ``joypose``,
``joystick_cursor``, ``spacenav_pose``, ``pinhole_wizard.py``). Specifically,
they listen to the ROS ``/joy`` topic. To ensure that your joystick is running,
you can do this from the command line:

    rostopic echo /joy

Using a web browser instead of a physical joystick
==================================================

If you don't have a real joystick, you can run an emulated one:

    rosrun browser_joystick web_control.py

This starts a webserver running on the local machine and prints the
URL. Open this URL with a modern browser and the webserver should now
emit messages on the ROS ``/joy`` topic.

Note: this requres the installation of python-tornado (> 2.4.x) and
python-sockjs-tornado packages.

Using a physical joystick
=========================

If your joystick is device ``/dev/input/js0``, use ROS to emit
``/joy`` messages like this:

    rosrun joy joy_node /dev/input/js0

Using a PS3 joystick
--------------------

A PS3 joystick can be run like a physical joystick, but there are a
couple of tricks to get it connected. The ROS ``ps3joy`` package
facilitates this. The required steps are:

For initial setup, perform bluetooth pairing with the joystick (Use
the ``sixpair`` program.)

Then, for daily use, run the bluetooth listener which write the output
into the linux device system. Run ``python ps3joy.py``.
