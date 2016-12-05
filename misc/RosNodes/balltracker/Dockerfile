FROM osrf/ros:indigo-desktop-full

ENV http_proxy http://proxy-chain.intel.com:911
ENV https_proxy http://proxy-chain.intel.com:911
ENV no_proxy localhost,intel.com,/var/run/docker.sock,127.0.0.1,10.0.0.0/8

RUN apt-get update \
    && apt-get install -y nano python-pip \
    && rm -rf /var/lib/apt/lists/ \
    && pip install imutils

COPY balltracker.py ./balltracker.py
