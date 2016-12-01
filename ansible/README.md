# FreemooVR Ansible Role

These are ansible roles to install freemoovr and have been tested on
Ubuntu xenial (16.04) amd64 and use ROS kinetic.

## Install the prequisites and the installer using ansible

Create an Ansible playbook file like this:

```yml
# playbook.yml
---
- hosts: localhost
  roles:
    - ros-kinetic-freemoovr
```

This will install ROS kinetic into /opt/ros and will also create a
script in /opt/ros/user-install-ros-kinetic-freemoovr.sh which any
user can run to install a ROS workspace in `~/ros/freemoovr-kinetic/`.

## Install freemoovr

After installing the prerequisites and the installer (see above), run
the following as a normal user:

```bash
/opt/ros/user-install-ros-kinetic-freemoovr.sh
```

## Usage

After running the user-install script to install the ROS workspace in
`~/ros/freemoovr-kinetic/` (see above), you can start a demo VR
display server with:

```bash
source ~/ros/freemoovr-kinetic/devel/setup.bash
roslaunch freemoovr demo_display_server.launch
```

## Further instructions

freemoovr is built as a ROS component. Read more about ROS at
http://www.ros.org
