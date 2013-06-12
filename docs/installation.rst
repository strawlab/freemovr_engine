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

 3. Get the flyvr.rosinstall file from github and save to a local
    directory. Manually visit https://github.com/strawlab/flyvr and
    download the ``flyvr.rosinstall`` file.

 4. Download FlyVR into ``~/flyvr-devel``::

      rosinstall ~/flyvr-devel ~/ros flyvr.rosinstall

 5. Build FlyVR::

      . ~/flyvr-devel/setup.sh
      rosmake flyvr --rosdep-install

Note, the above requires installation of package libeigen3-dev on
Ubuntu, which is not available through normal Ubuntu channels. Binaries are available at:
 * http://packages.ros.org/ros/ubuntu/pool/main/e/eigen3/libeigen3-dev_3.0.1-1+ros4~lucid_amd64.deb
 * http://packages.ros.org/ros/ubuntu/pool/main/e/eigen3/libeigen3-dev_3.0.1-1+ros4~lucid_i386.deb
