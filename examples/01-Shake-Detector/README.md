# OLIVE Shake Detector IMU
This simple example shows a ROS 2 shake detector with the **Olive Robotics IMU olixSense IMU/AHRS E1**.

## Dependencies
- `ROS 2 Humble`: The Humble Distro of ROS 2

## Getting started
1. Plugin the **Olive Robotics IMU olixSense IMU/AHRS E1** IMU or another Olive IMU (adjust the topic in the code)
2. Check if the IMU topics are available
3. To start the shake detection run following command
```bash
python3 python/shake_detector.py
```