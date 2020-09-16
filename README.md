# Data Capture for Arduino Nano33 BLE Sense

This provides a [PlatformIO](https://platformio.org/) project to flash the Arduino Nano33 BLE Sense with firmware suitable for data capture with the [SensiML Data Capture Lab](https://sensiml.com/products/data-capture-lab/).

This code is meant as a reference code for setting up data capture with any Arduino Device. This firmware collects data from the LSM9DS1 9-axis accelerometer only. Other sensors can be added by extending this firmware.

## Requirements for building

- PlatformIO IDE (or plugin for your favorite IDE, command line interface)
- Arduino Nano33 BLE Sense
- Windows Computer for Data Capture Lab

## Known Issues

- Currently only works over serial port. ArduinoBLE library consistently crashes when attempting to connect via Windows BLE libraries
- Magnetometer will be collected at the higher sample rate of Accel/Gyro, should it be used.


