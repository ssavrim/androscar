FROM osrf/ros:indigo-desktop-full

RUN apt-get update \
    && apt-get install -y nano python-pip \
    && rm -rf /var/lib/apt/lists/ \
    && pip install imutils

COPY balltracker.py ./balltracker.py
