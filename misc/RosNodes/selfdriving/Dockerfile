FROM osrf/ros:indigo-desktop-full

COPY requirements.txt ./requirements.txt
COPY selfdriving.py ./selfdriving.py


RUN apt-get update \
    && apt-get install -y nano python-pip \
    && rm -rf /var/lib/apt/lists/ \
    && pip install -U  -r requirements.txt


