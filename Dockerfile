FROM ubuntu:20.04

RUN apt-get -y update && apt-get -y dist-upgrade
RUN apt-get install -y python3-dev libopenscenegraph-dev python3-setuptools python3-numpy build-essential libjansson-dev cython3

ADD . src
WORKDIR src

RUN python setup.py build
RUN python setup.py install

# For python3:
RUN python3 setup.py build
RUN python3 setup.py install
