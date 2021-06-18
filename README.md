# Swing Path Tracking
@JasonChuangTW @chi1027 @jerry871002

This is a repo of the final project of the embedded system course. (2021 spring)

## Motivation
- We are in the baseball team of NYCU
- Batting is a important part in baseball
- We want to track the path of bat while swinging
- Improve training efficiency
- Help player to adjust batting pose

## Idea
- Use IMU (GY-801) to track the path
- Plot and visualize the path
- Show the result via Line Notify

## Hardware
- Raspberry Pi: MQTT Broker and client, data processing and visualization, send result via Line
- ESP32-S: MQTT Client, collect raw data from the IMU
- GY-801: IMU
- Battery (18650)

## Software
- Python: Used on Raspberry Pi
- Micropython: Used on ESP32-S
- MQTT (Message Queuing Telemetry Transport)
- Line Notify API: Used to send message

## How to use

### With ESP32
1. Connect your GY801 to a ESP32, make sure the ESP32 and Raspberry Pi are in the same network.
2. Run `main_esp32.py` on ESP32.
3. Run `mqtt_subscribe.py` on Pi to make sure if MQTT is working.
4. Run `main_pi.py` on Pi to start measuring!

You may need to adjust the setting in `line_notifier.py` to send the message correctly

### Without ESP32
1. Connect GY801 directly to Pi
2. Run `main_without_esp32.py` on Pi to start measuring!

## Where to calibrate

### With ESP32

Checkout `main_without_esp32.py`

### Without ESP32

Checkout `imu.py`

## References

- [Exploring Data Acquisition and Trajectory Tracking with Android Devices and Python](https://medium.com/analytics-vidhya/exploring-data-acquisition-and-trajectory-tracking-with-android-devices-and-python-9fdef38f25ee)
- [AHRS](https://pypi.org/project/AHRS/)
- [gborgonovo/gy801](https://github.com/gborgonovo/gy801)