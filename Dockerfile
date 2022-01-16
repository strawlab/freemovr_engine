FROM ubuntu:20.04

RUN apt-get -y update && apt-get -y dist-upgrade
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y python3-dev libopenscenegraph-dev python3-setuptools python3-numpy build-essential libjansson-dev cython3

ADD . src
WORKDIR src

RUN python3 setup.py build
RUN python3 setup.py install
