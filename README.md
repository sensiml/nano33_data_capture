# Data Capture for Arduino Nano33 BLE Sense

This provides a [PlatformIO](https://platformio.org/) project to flash the Arduino Nano33 BLE Sense with firmware suitable for data capture with the [SensiML Data Capture Lab](https://sensiml.com/products/data-capture-lab/).

This code is meant as a reference code for setting up data capture with any Arduino Device. This firmware collects data from the LSM9DS1 9-axis accelerometer only. Other sensors can be added by extending this firmware.

Documentation for using this firmware is provided at [SensiML's documentation site](https://sensiml.com/documentation/firmware/arduino-nano33/arduino-nano33.html)

## Requirements for building

- PlatformIO IDE (or plugin for your favorite IDE, command line interface)
- Arduino Nano33 BLE Sense
- Windows Computer for Data Capture Lab

## Data Collection Application

This project has two output options - BLE or Serial

See sensor_config.h for options to turn on either Audio or the IMU Sensor

To turn on serial debug options set

    #define SERIAL_DEBUG 1

Note: When SERIAL_DEBUG is turned on for the BLE application, it will wait to connect to the Serial port before starting
