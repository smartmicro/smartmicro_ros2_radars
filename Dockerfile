FROM ros:foxy

RUN apt-get update && apt-get install -y \
    iputils-ping \
    python3 \
    python3-pip \
    ros-foxy-point-cloud-msg-wrapper \
    tcpdump \
    wget
RUN pip3 install dpkt
WORKDIR /code
