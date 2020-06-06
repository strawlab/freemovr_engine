FROM ubuntu:20.04

RUN apt-get -y update && apt-get -y dist-upgrade
RUN apt-get install -y cython python-dev libopenscenegraph-dev
RUN apt-get install -y python-setuptools
RUN apt-get install -y python-numpy
RUN apt-get install -y build-essential
RUN apt-get install -y libjansson-dev

# For python3:
RUN apt-get install -y python3-dev python3-setuptools python3-numpy
RUN apt-get install -y cython3

ADD . src
WORKDIR src

RUN python setup.py build
RUN python setup.py install

# For python3:
RUN python3 setup.py build
RUN python3 setup.py install
