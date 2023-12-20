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
This release also offers the possibility of configuring the ip addresses of the sensor using ros2 services. The tests are further extended to include ros2 services check. Requires new sensor firmware, if using UMRR11-T132: V5.1.4 and if using UMRR96-T153: V5.2.4.

## v3.0.0 - 2022-09-23

Major release includes the new smartmicro sensor DRVEGRD 169. The driver offers mode changes and configuration of the DRVEGRD 169 along with publishing the radar targets as point cloud data. The callbacks for datastream now require a clientID. 

## v3.1.0 - 2022-10-19

This release includes the new smartmicro sensor DRVEGRD 152. The driver offers mode changes and configuration of the DRVEGRD 152 along with publishing the radar targets as point cloud data. The callbacks for datastream now require a clientID.

## v3.2.0 - 2022-11-11

This minor release introduces signal-to-noise field in the point clouds and also fixes the max number of sensors that could be connected at once.

## v3.2.1 - 2022-12-16

This release fixes the offset which causes anomaly in point clouds from DRVEGRD 152 sensor. It also fixes the timestamp calculation bug which causes rviz to crash and updates the simulator source files.

## v4.0.0 - 2023-02-06

This release includes the new DRVEGRD 169 user-interface. The release includes new modes for DRVEGRD 169 and also introduces an additional ros2 service to save configurations. Previously used model 'UMRR9F' in radar parameters has been divided into two versions, the 'UMRR9F_V1_1_1' and 'UMRR9F_V2_0_0'.

## v4.1.0 - 2023-08-21

This release includes the new DRVEGRD 171 and DRVEGRD 152 user-interface. The release includes a complete list of all params and commands for all sensors which are accessed using the ros2 services.

## v5.0.0 - 2023-09-22

This release includes CAN communication for all the provided sensor types and sensors interfaces. The params are now extended to include the params/setting for the connected adapters along with the sensor params.

## v6.0.0 - 2023-12-20

This release includes features for downloading sensor firmware on to the sensors. It also provides custom RVIZ plugins to log the target list data, record and save it. It has a plugin to send instructions to the sensors and also it is possible to dowload the firmware too. This release also includes a python GUI to send custom CAN messages. The radar param templates have now been merged into one param file. The pointcloud has been extended to also include the polar coordinates. Additionally, the release also includes new user interface for sensor A4 T171.