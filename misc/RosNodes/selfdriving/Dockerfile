FROM osrf/ros:indigo-desktop-full

ENV http_proxy http://proxy-chain.intel.com:911
ENV https_proxy http://proxy-chain.intel.com:911
ENV no_proxy localhost,intel.com,/var/run/docker.sock,127.0.0.1,10.0.0.0/8

RUN apt-get update \
    && apt-get install -y wget build-essential python-dev \
    && rm -rf /var/lib/apt/lists/

COPY requirements.txt ./requirements.txt
RUN wget https://bootstrap.pypa.io/get-pip.py \
    && python get-pip.py \
    && pip install --upgrade pip \
    && pip install -r requirements.txt

COPY selfdriving.py ./selfdriving.py
