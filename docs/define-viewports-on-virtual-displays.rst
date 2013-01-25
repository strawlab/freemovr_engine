***************************************
Defining viewports for virtual displays
***************************************

Run the ``viewport_definer.py`` command. For example, on display
server ``ds`` and viewport ``center``, do the following.

::

    rosrun flyvr viewport_definer.py --display-server /ds --viewport center

This will set the ROS parameters in the roscore server with the values
you set with the GUI. To save these parameters to a file ``ds.yaml``.

::

    rosparam dump ds.yaml /ds
