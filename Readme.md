# ROS2 smartmicro radar driver

[![Build and test](https://github.com/smartmicro/smartmicro_ros2_radars/actions/workflows/dockerbuild.yml/badge.svg)](https://github.com/smartmicro/smartmicro_ros2_radars/actions/workflows/dockerbuild.yml)

## Purpose / Use cases
There is a need for a node that will interface with a smartmicro radar driver and publish the data
acquired by the sensor through the ROS2 pipeline. This package implements such a node.

## Get the Smart Access release
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

### UMRR radars and Smart Access API version
A [smartmicro](https://www.smartmicro.com/automotive-radar) UMRR96 radar, UMRR11 radar or UMRR9F radar are 
required to run this node. This code is bundled with a version of Smart Access API. Please make
sure the version used to publish the data is compatible with this version:

- Date of release: `September 22, 2022`
- Smart Access Automotive version: `v2.0.0`
- User interface version: `UMRR96 Type 153 AUTOMOTIVE v1.2.1`
- User interface version: `UMRR11 Type 132 AUTOMOTIVE v1.1.1`
- User interface version: `UMRR9F Type 169 AUTOMOTIVE v1.1.1`

### Sensor Firmwares
This ROS2 driver release is compatible with the following sensor firmwares:
- UMRR11 Type 132: V5.1.4
- UMRR96 Type 153: V5.2.4

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
- Through using the sensor simulators

These inputs are processed through the Smart Access C++ API and trigger a callback. Every time this
callback is triggered a new point cloud message is created and published.

### The outputs:
The driver publishes `sensor_msgs::msg::PointCloud2` messages with the radar targets on the topic
`umrr/targets` which can be remapped through the parameters.

### Configuration:
The node is configured through the parameters. Here is a short recap of the most important parts.
For more details, see the [`radar.template.yaml`](umrr_ros2_driver/param/radar.template.yaml) file.
- `client_id`: the id of the client, must be a unique integer
- `ip`: the IP of the used sensor or the source
- `port`: port to be used to receive the packets
- `iface_name`: name of the used network interface
- `frame_id`: name of the frame in which the messages will be published
- `history_size`: size of history for the message publisher
- `model`: the model of the sensor being used 

## Mode of operations of the sensors
The smartmicro radars come equipped with numerous features and modes of operation. Using the ros2 services provided one
may access these modes. A list of available sensor modes is given in the [`sensor_params.json`](umrr_ros2_driver/config/sensor_params.json).

A ros2 `SetMode` service should be called to implement these mode changes. There are three inputs to a ros2 service call:
- `param`: name of the mode instruction (specific to the sensor)
- `value`: the mode of operation (specific to sensor where the modes are same) 
- `sensor_id`: the id of the sensor to which the service call should be sent.

For instance, changing the `Index of Transmit Antenna (tx_antenna_idx)` of a UMRR-11 sensor to `AEB (2)` mode would require the following call:
`ros2 service call /smartmicro_radar_node/set_radar_mode umrr_ros2_msgs/srv/SetMode "{param: "tx_antenna_idx", value: 2, sensor_id: 100}"`

## Configuration of the sensors
In order to use multiple sensors (maximum of up to eight sensors) with the node the sensors should be configured separately.
The IP addresses of the sensors could be assigned using:
- The smartmicro tool `DriveRecorder`.
- Using the `Smart Access C++ API`
- Using `Sensor Services` provided by the node

Each sensor has to be assigned a unique IP address!

To use the ros2 `SetIp`service we require two inputs:
- `value_ip`: the value of the ip address in decimal. For instance to set the IP to `192.168.11.64` its corresponding
value in decimal `3232238400` should be used.
- `sensor_id`: the sensor whose ip address is to be changed.

The call for such a service would be as follows:
`ros2 service call /smartmicro_radar_node/set_ip_address umrr_ros2_msgs/srv/SetIp "{value_ip: 3232238400, sensor_id: 100}"`

Note: For successfull execution of this call it is important that the sensor is restarted, the ip address in the
[`radar.template.yaml`](umrr_ros2_driver/param/radar.template.yaml) is updated and the driver is build again.

## Sensor Service Responses

The sensor services respond with certain value codes. The following is a lookup table for the possible responses:

**Value**   |   **Description**
--- | ---
0   |    No instruction Response
1   |    Instruction Response was processed successfully
2   |    General error
6   |    Invalid protection
7   |    Value out of minimal bounds
8   |    Value out of maximal bounds

## Development
The dockerfile can be used to build and test the ros driver.

### Prerequisites

- Docker version >= 20.10.14
- Docker compose version >= 1.29.2

## Building and Testing
Accept the agreement and get the smartaccess release
```bash
./smart_extract.sh
```

Building docker container
```bash
docker build . -t umrr-ros:latest
```

Building the driver with the docker container
```bash
docker run --rm -v`pwd`:/code umrr-ros colcon build
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
## ARMv8 Support
The Smart Access release which will be downloaded using the script also offers platform support for armv8. In order to build the driver on an armv8 machine, the [`CMakeLists.txt`](umrr_ros2_driver/CMakeLists.txt) should be adopted.
Instead of using the default `lib-linux-x86_64_gcc_9` the user should plugin the `lib-linux-armv8-gcc_9` for armv8.
 
## Contribution
This project is a joint effort between [smartmicro](https://www.smartmicro.com/) and [Apex.AI](https://www.apex.ai/). The initial version of the code was developed by Igor Bogoslavskyi of Apex.AI (@niosus) and was thereafter adapted and extended by smartmicro.

## License
Licensed under the [Apache 2.0 License](LICENSE).
