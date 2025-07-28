# Olive North Detector

This example shows a north detector with the magnometer of the IMU in ROS 2. 

## Dependencies
- `ROS 2 Humble`: The Humble Distro of ROS 2

## Configuration

1. Go to the Olive System Manager, by connecting to `192.168.7.100` in your browser.
2. Go to `Sensor Settings`
3. Change **AI Fusion Mode** from `AHRS` to `IMU`
4. Press `Change mode`, accept the system reboot prompt

## Quickstart
```commandline
python src/north.py
```


Note: High-accuracy directional north functionality requires calibration.

