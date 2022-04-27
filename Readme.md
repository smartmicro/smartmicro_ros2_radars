# ROS2 smartmicro radar driver

[![Build and test](https://github.com/smartmicro/smartmicro_ros2_radars/actions/workflows/dockerbuild.yml/badge.svg)](https://github.com/smartmicro/smartmicro_ros2_radars/actions/workflows/dockerbuild.yml)

## Purpose / Use cases
There is a need for a node that will interface with a smartmicro radar driver and publish the data
acquired by the sensor through the ROS2 pipeline. This package implements such a node.

## Get the smartaccess release
```bash
./smart_extract.sh
```

## How to launch this node
```
ros2 launch umrr_ros2_driver radar.launch.py
```

## Prerequisites

### Supported ROS distributions:
- ROS2 foxy

### UMRR-96 radar and Smart Access API version
A [smartmicro](https://www.smartmicro.com/automotive-radar) UMRR96 radar, UMRR11 radar or both are 
required to run this node. This code is bundled with a version of Smart Access API. Please make
sure the version used to publish the data is compatible with this version:

- Date of release: `March 25, 2022`
- Smart Access Library version: `v4.3.0`
- User interface version: `Smartaccess UMRR96 Type 153 AUTOMOTIVE v1.2.1`
- User interface version: `Smartaccess UMRR11 Type 132 AUTOMOTIVE v1.1.1`

### Point cloud message wrapper library
To add targets to the point cloud in a safe and quick fashion a
[`point_cloud_msg_wrapper`](https://gitlab.com/ApexAI/point_cloud_msg_wrapper) library is used within
this project's node. This project can be installed either through `rosdep` or manually by executing:
```
sudo apt install ros-foxy-point-cloud-msg-wrapper
```

## Inputs / Outputs / Configuration

### The inputs:
The inputs are coming as network packages generated in either of the following two ways:
- Through directly interfacing with the sensor
- Through a provided pcap file

These inputs are processed through the Smart Access C++ API and trigger a callback. Every time this
callback is triggered a new point cloud message is created and published.

### The outputs:
The driver publishes `sensor_msgs::msg::PointCloud2` messages with the radar targets on the topic
`umrr/targets` which can be remapped through the parameters.

### Configuration:
The node is configured through the parameters. Here is a short recap of the most important parts.
For more details, see the [`radar.template.yaml`](param/radar.template.yaml) file.
- `client_id`: the id of the client, must be a unique integer
- `ip`: the IP of the used sensor or the source
- `port`: port to be used to receive the packets
- `iface_name`: name of the used network interface
- `frame_id`: name of the frame in which the messages will be published
- `history_size`: size of history for the message publisher
- `model`: the model of the sensor being used 

## Configuration of the sensors
In order to use multiple sensors (maximum of up to ten sensors) with the node the sensors should be configured separately.
The IP addresses of the sensors could be assigned using:
- The smartmicro tool `DriveRecorder`.
- Using the `Smart Access C++ API`

Each sensor has to be assigned a unique IP address!

## Development
The dockerfile can be used to build and test the ros driver.

Accept the agreement and get the smartaccess release
```bash
./smart_extract.sh
````

Building docker container
```bash
docker build . -t umrr-ros:latest
```

Building the driver with the docker container
```bash
docker run --rm -v`pwd`:/code umrr-ros colcon build --packages-select umrr_ros2_driver
```

Running the unit and integration tests via the docker compose
```bash
docker-compose up
```

Getting the test coverage via the docker container
```bash
docker run --rm -v`pwd`:/code umrr-ros colcon test-result --all --verbose
```

Stop and remove docker containers and networks
```bash
docker-compose down
```

## Contribution
This project is a joint effort between [smartmicro](https://www.smartmicro.com/) and [Apex.AI](https://www.apex.ai/). The initial version of the code was developed by Igor Bogoslavskyi of Apex.AI (@niosus) and was thereafter adapted and extended by smartmicro.

## License
Licensed under the [Apache 2.0 License](LICENSE).
