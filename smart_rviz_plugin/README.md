# smart_rviz_plugin

ROS2 RViz panel plugin suite for Smartmicro radar operations.

## What this package provides

This package provides RViz panels to support day-to-day radar workflows:
1. Smart Recorder (CSV target/object recording)
2. Smart Command Configurator (services: command/param/status operations)
3. Smart Firmware Download (firmware transfer workflow)
4. Smart Status (target/object header monitoring)
5. Smart Fault Reports (fault report monitoring)

For detailed architecture and full plugin design, see [DESIGN_DOCUMENT.md](DESIGN_DOCUMENT.md).

## Build

From workspace root:

```bash
colcon build --packages-select smart_rviz_plugin
source install/setup.bash
```

## Load plugins in RViz2

1. Start RViz2.
2. Open Panels menu.
3. Add the desired Smart plugins from the smart_rviz_plugin library.

## Plugin summary

### Smart Recorder
- Captures target/object data and exports CSV.

### Smart Command Configurator
- Sends commands and mode/config related service calls.

### Smart Firmware Download
- Sends firmware download request to selected sensor.

### Smart Status
- Displays live target/object header status topics.

### Smart Fault Reports
- Displays live fault report topics per sensor.

## Troubleshooting

1. If plugin is not visible in RViz, verify package is built and sourced.
2. If service-based panels do not respond, verify corresponding service servers are running.
