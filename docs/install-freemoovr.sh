#!/bin/bash -x
set -e

# ----- Check that we have the supported version of Ubuntu ----------
RELEASE=`lsb_release --release | cut -f2`
if [ ${RELEASE} == "12.04" ]; then
    echo "OK: using Ubuntu 12.04"
else
    echo "ERROR: release ${RELEASE} not supported"
    exit 1
fi

ARCH=`uname -i`
if [ ${ARCH} == "x86_64" ]; then
    echo "OK: using adm64 architecture"
else
    echo "ERROR: architecture ${ARCH} not supported"
    exit 1
fi

# ----- Check that we have root permissions ----

if [ "$UID" -ne "0" ]; then
    echo "ERROR: you need to have superuser permissions. Re-run with 'sudo'."
    exit 1
fi

# ----- Update our packages ------------

apt-get update --yes
apt-get install --yes python-software-properties

# -------------- add our repository -----------
#     add key id=F8DB323D
wget -qO - http://debs.strawlab.org/astraw-archive-keyring.gpg | sudo apt-key add -
add-apt-repository 'deb http://debs.strawlab.org/ precise/'

# -------------- add ROS repository -----------
#     add key id=B01FA116
wget -qO - http://packages.ros.org/ros.key | sudo apt-key add -
add-apt-repository 'deb http://packages.ros.org/ros/ubuntu precise main'

# -------------- add multiverse repository (needed for nvidia-cg-toolkit) ---------------
add-apt-repository 'deb http://archive.ubuntu.com/ubuntu/ precise multiverse'
add-apt-repository 'deb http://archive.ubuntu.com/ubuntu/ precise-updates multiverse'
add-apt-repository 'deb http://archive.ubuntu.com/ubuntu/ precise-security multiverse'


# -------------- update local package index ----
apt-get update --yes
apt-get upgrade --yes

# ---- install ROS

DEBIAN_FRONTEND=noninteractive apt-get install --yes python-rosinstall ros-hydro-desktop-full make

# ------ install our overlay ------------
ROSINSTALL_SPEC_PATH="/tmp/freemoovr.rosinstall"

# ---- create a .rosinstall spec file for this git revision -------------
cat > ${ROSINSTALL_SPEC_PATH} <<EOF
- git: {local-name: freemoovr, uri: 'https://github.com/strawlab/freemoovr.git', version: freemoovr-hydro}
- git: {local-name: rosgobject, uri: 'https://github.com/strawlab/rosgobject.git', version: freemoovr-hydro}
EOF

# ---- script to ensure a line in a file ----
ENSURE_LINE_PATH="/tmp/ensure_line.py"

cat > ${ENSURE_LINE_PATH} <<EOF
#!/usr/bin/env python
import argparse
import os

def ensure_line_in_file( line, filename, create_ok=True ):
    if not os.path.exists(filename):
        if not create_ok:
            raise RuntimeError('the file %r does not exist, but will not create it.')
        else:
            fd = open(filename,mode='a+') # do not delete if just created
            fd.close()

    line_in_file = False
    with open(filename, mode='r+') as fd:
        for this_line in fd.readlines():
            this_line_rstrip = this_line.rstrip()
            if line==this_line_rstrip:
                line_in_file = True
                break

        if not line_in_file:
            fd.write(line + '\n')

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('line', help='the line to be included')
    parser.add_argument('filename', help='the file in which to include it')

    args = parser.parse_args()

    ensure_line_in_file( args.line, args.filename )

EOF

# ---- install our ROS stuff
export FREEMOOVR_TARGET="/opt/ros/ros-freemoovr.hydro"
rosinstall ${FREEMOOVR_TARGET} /opt/ros/hydro/.rosinstall ${ROSINSTALL_SPEC_PATH}

source ${FREEMOOVR_TARGET}/setup.bash

if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  echo "rosdep already initialized"
else
  sudo rosdep init
fi

chmod a+x ${ENSURE_LINE_PATH}
${ENSURE_LINE_PATH} "yaml https://raw.github.com/strawlab/rosdistro/hydro/rosdep.yaml" "/etc/ros/rosdep/sources.list.d/20-default.list"
chmod -R a+rX /etc/ros

rosdep update

rosdep install freemoovr --default-yes
rosmake freemoovr

chmod -R a+rX ${FREEMOOVR_TARGET}
