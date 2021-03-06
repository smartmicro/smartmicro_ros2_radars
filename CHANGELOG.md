# Change Log
All notable changes to this project will be documented in this file. This project adheres to [Semantic Versioning](http://semver.org/).
 
## v1.0.0 - 2021-11-01

This is the first official versioned release of the `smartmicro_ros2_radars`. It provides a ros2 node that is wrapped around `Smart Access C++ API v3.3.15`, interfacing with smartmicro automotive radars and publishing the incoming data as point cloud.

## v2.0.0 - 2022-05-05

This major release of the driver includes multi-user interfaces. The driver now supports and publishes data from UMRR96 and UMRR11.
A new test approach has been implemented which simulates the sensors and interfaces with the node hence making the tests more robust.

## v2.1.0 - 2022-06-02

This minor release of the driver offer ros2 services to communicate with the sensor. The driver now supports mode changes for UMRR96 and UMRR11.
This release also offers the possibilty of configuring the ip addresses of the sensor using ros2 services. The tests are further extended to inlcude ros2 services check.