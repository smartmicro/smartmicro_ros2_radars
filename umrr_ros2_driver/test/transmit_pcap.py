# Copyright (c) 2021, s.m.s, smart microwave sensors GmbH, Brunswick, Germany
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import socket
import time

import dpkt

UDP_TARGET_IP = "172.22.10.100"
UDP_TARGET_PORT = 55555

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
file_path = os.path.dirname(os.path.abspath(__file__))
path_pcap = os.path.join(file_path, "..", "pcap", "Radar_data_recording.pcapng")

while True:
    with open(path_pcap, "rb") as f:
        pcap = dpkt.pcap.Reader(f)
        for ts, buf in pcap:
            eth = dpkt.ethernet.Ethernet(buf)
            radardata = eth.data.data.data
            sock.sendto(radardata, (UDP_TARGET_IP, UDP_TARGET_PORT))
            time.sleep(0.5)
