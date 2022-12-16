# Change Log
All notable changes to this project will be documented in this file. This project adheres to [Semantic Versioning](http://semver.org/).
 
## v1.0.0 - 2021-11-01

This is the first official versioned release of the `smartmicro_ros2_radars`. It provides a ros2 node that is wrapped around `Smart Access C++ API v3.3.15`, interfacing with smartmicro automotive radars and publishing the incoming data as point cloud.

## v2.0.0 - 2022-05-05

This major release of the driver includes multi-user interfaces. The driver now supports and publishes data from UMRR96 and UMRR11.
A new test approach has been implemented which simulates the sensors and interfaces with the node hence making the tests more robust.
Requires new sensor firmware, if using UMRR11-T132: V5.1.4 and if using UMRR96-T153: V5.2.4.

## v2.1.0 - 2022-06-02

This minor release of the driver offer ros2 services to communicate with the sensor. The driver now supports mode changes for UMRR96 and UMRR11.
This release also offers the possibilty of configuring the ip addresses of the sensor using ros2 services. The tests are further extended to inlcude ros2 services check. Requires new sensor firmware, if using UMRR11-T132: V5.1.4 and if using UMRR96-T153: V5.2.4.

## v3.0.0 - 2022-09-23

Major release includes the new smartmicro sensor DRVEGRD 169. The driver offers mode changes and configuration of the DRVEGRD 169 along with publishing the radar targets as pointcloud data. The callbacks for datastream now require a clientID. 

## v3.1.0 - 2022-10-19

This release includes the new smartmicro sensor DRVEGRD 152. The driver offers mode changes and configuration of the DRVEGRD 152 along with publishing the radar targets as pointcloud data. The callbacks for datastream now require a clientID.

## v3.2.0 - 2022-11-11

This minor release introduces signal-to-noise field in the pointclouds and also fixes the max number of sensors that could be connected at once.

## v3.3.0 - 2022-12-16

This release fixes the offset which causes anomaly in pointclouds from DRVEGRD 152 sensor. It also fixes the timestamp calculation bug which causes rviz to crash and updates the simulator source files.
