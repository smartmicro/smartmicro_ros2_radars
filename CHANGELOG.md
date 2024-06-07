# Change Log
All notable changes to this project will be documented in this file. This project adheres to [Semantic Versioning](http://semver.org/).
 
## v1.0.0 - 2021-11-01

### Initial Release
- **smartmicro_ros2_radars**: First official versioned release.
- **ROS 2 Node**: Wrapped around `Smart Access C++ API v3.3.15`.
- **Functionality**: Interfaces with smartmicro automotive radars and publishes incoming data as point cloud.

## v2.0.0 - 2022-05-05

### Major Update
- **Multi-User Interface**: Supports and publishes data from UMRR96 and UMRR11.
- **Testing Enhancements**: Implemented a new test approach simulating sensors and interfacing with the node.
- **New Sensor Firmware Requirement For**: 
  - UMRR11-T132: V5.1.4
  - UMRR96-T153: V5.2.4

# v2.1.0 - 2022-06-02

### Minor Update
- **ROS 2 Services**: Added services to communicate with the sensor.
- **Mode Changes**: Supports mode changes for UMRR96 and UMRR11.
- **IP Configuration**: Allows configuring sensor IP addresses via ROS 2 services.
- **Extended Testing**: Includes ROS 2 services check.
- **New Sensor Firmware Requirement For**: 
  - UMRR11-T132: V5.1.4
  - UMRR96-T153: V5.2.4

## v3.0.0 - 2022-09-23

### Major Update
- **New Sensor Support**: Added support for smartmicro sensor DRVEGRD 169.
- **Functionality**: Mode changes and configuration for DRVEGRD 169, publishes radar targets as point cloud data.
- **Callback Changes**: Data stream callbacks now require a clientID.

## v3.1.0 - 2022-10-19

### Minor Update
- **New Sensor Support**: Added support for smartmicro sensor DRVEGRD 152.
- **Functionality**: Mode changes and configuration for DRVEGRD 152, publishes radar targets as point cloud data.
- **Callback Changes**: Data stream callbacks now require a clientID.

## v3.2.0 - 2022-11-11

### Minor Update
- **Point Cloud Enhancement**: Introduced signal-to-noise field.
- **Bug Fixes**: Fixed the max number of sensors that could be connected simultaneously.

## v3.2.1 - 2022-12-16

### Patch Update
- **Bug Fixes**:
  - Fixed offset causing anomalies in point clouds from DRVEGRD 152 sensor.
  - Fixed timestamp calculation bug causing RViz to crash.
- **Updates**: Updated simulator source files.

## v4.0.0 - 2023-02-06

### Major Update
- **New UI**: Added user-interface for DRVEGRD 169.
- **New Modes**: Introduced new modes for DRVEGRD 169.
- **New Service**: Added ROS 2 service to save configurations.
- **Model Update**: 'UMRR9F' radar parameter model split into 'UMRR9F_V1_1_1' and 'UMRR9F_V2_0_0'.

## v4.1.0 - 2023-08-21

### Minor Update
- **New UI**: Added user-interfaces for DRVEGRD 171 and DRVEGRD 152.
- **Parameter List**: Complete list of all parameters and commands for all sensors accessible via ROS 2 services.

## v5.0.0 - 2023-09-22

### Major Update
- **CAN Communication**: Enabled for all provided sensor types and interfaces.
- **Parameter Expansion**: Extended parameters to include settings for connected adapters along with sensor parameters.

## v6.0.0 - 2023-12-20

### Major Update
- **Firmware Download**: Added features for downloading sensor firmware onto the sensors.
- **RViz Plugins**:
  - Log target list data, record, and save it.
  - Send instructions to the sensors.
- **Python GUI**: Added a GUI to send custom CAN messages.
- **Parameter Templates**: Merged radar parameter templates into one file.
- **Point Cloud Enhancement**: Added polar coordinates.
- **New UI**: Added user-interface for sensor A4 T171.

## v6.1.0 - 2024-01-26

### Minor Update
- **New UIs**: Added user interfaces for DRVEGRD 169 and DRVEGRD 152.
- **Bug Fixes**: 
  - Fixed issues with the smart record plugin.
  - Added azimuth and elevation angles in degrees.

## v7.0.0 - 2024-06-07

### New Features
- **User Interface for DRVEGRD 169**: Introduced a new UI with integrated object tracking capabilities.
- **RViz Plugins**: Added plugins to RViz for decoding targets, objects, and header information.
- **Enhanced Command Configurator**: Improved the command configurator for better usability and functionality.
- **New ROS 2 Parameter - `pub_type`**: Added the `pub_type` parameter to manage the desired publication type.
- **New message definitions**:
  - CanObjectHeader.msg includes object status for sensors connected over CAN.
  - CanTargetHeader.msg includes target status for sensors connected over CAN.
  - PortObjectHeader.msg includes object status for sensors connected over ethernet.
  - PortTargetHeader.msg includes target status for sensors connected over ethernet.
