#!/bin/bash
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
    echo "ERROR: you need to have superuser permissions"
    exit 1
fi

# ----- Update our packages ------------

apt-get update --yes
apt-get install --yes openssh-server git-core python-software-properties

# -------------- add multiverse repository (needed for nvidia-cg-toolkit) ---------------
add-apt-repository 'deb http://archive.ubuntu.com/ubuntu/ precise multiverse'
add-apt-repository 'deb http://archive.ubuntu.com/ubuntu/ precise-updates multiverse'
add-apt-repository 'deb http://archive.ubuntu.com/ubuntu/ precise-security multiverse'

# -------------- add our repository -----------
#     add key id=F8DB323D
wget -qO - http://debs.strawlab.org/astraw-archive-keyring.gpg | sudo apt-key add -
add-apt-repository 'deb http://debs.strawlab.org/ precise/'

apt-get update --yes
apt-get install --yes python-sh python-grin htop byobu

# ---- install ROS

export ROS_TARGET="/opt/ros/ros.electric.boost1.46"
export FLYVR_TARGET="/opt/ros/ros-flyvr.electric.boost1.46"

wget https://raw.github.com/strawlab/rosinstall/master/scripts/electric_check_ros.bash -O /tmp/electric_check_ros.bash
chmod a+x /tmp/electric_check_ros.bash
/tmp/electric_check_ros.bash


# ---- install our ROS stuff

wget https://raw.github.com/strawlab/rosinstall/master/scripts/electric_check_flyvr.bash -O /tmp/electric_check_flyvr.bash
chmod a+x /tmp/electric_check_flyvr.bash
/tmp/electric_check_flyvr.bash
