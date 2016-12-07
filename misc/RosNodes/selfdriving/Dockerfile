FROM osrf/ros:indigo-desktop-full

RUN apt-get update \
    && apt-get install -y wget build-essential python-dev \
    && rm -rf /var/lib/apt/lists/

COPY requirements.txt ./requirements.txt
RUN wget https://bootstrap.pypa.io/get-pip.py \
    && python get-pip.py \
    && pip install --upgrade pip \
    && pip install -r requirements.txt

COPY scripts ./scripts
