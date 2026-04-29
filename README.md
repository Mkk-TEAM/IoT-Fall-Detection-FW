# IoT Fall Detection Firmware

PlatformIO firmware project for the NodeMCU-32S board.

## Environment

```ini
[env:nodemcu-32s]
platform = espressif32
board = nodemcu-32s
framework = arduino
monitor_speed = 115200
```

## Dependency

```ini
lib_deps =
    https://github.com/DFRobot/DFRobot_WT61PC.git
```

## Common Commands

```bash
pio run
pio run --target upload
pio device monitor
```
