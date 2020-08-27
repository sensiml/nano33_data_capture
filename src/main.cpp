
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Arduino_APDS9960.h> //Include library for colour, proximity and gesture recognition
#include <Arduino_HTS221.h> //Include library to read Temperature and Humidity
#include <Arduino_LPS22HB.h> //Include library to read Pressure
#include <Arduino_LSM9DS1.h> //Include the library for 9-axis IMU

#include "collection_config.h"

#define MAX_NUMBER_OF_COLUMNS 10

#if USE_BLE
#include <ArduinoBLE.h>

// BLE Service
BLEService sensorService(uuidOfService);

// RX / TX Characteristics
BLECharacteristic configChar(uuidOfConfigChar, BLERead, WRITE_BUFFER_SIZE, WRITE_BUFFER_FIXED_LENGTH);
BLEDescriptor configNameDescriptor("2901", "Sensor Configuration");
BLECharacteristic sensorDataChar(uuidOfDataChar, BLERead | BLENotify, 20, WRITE_BUFFER_FIXED_LENGTH);
BLEDescriptor sensorDataDescriptor("2901", "Sensor Data TX");
#endif

static accel_gyro_odr_t accel_gyro_speed = ACCEL_GYRO_DEFAULT_ODR;
static float actual_odr;
static mag_odr_t mag_speed = MAG_DEFAULT_ODR;
static bool config_received = false;

static int16_t sensorRawData[MAX_NUMBER_OF_COLUMNS];
static int sensorRawIndex = 0;

DynamicJsonDocument config_message(256);

static int column_index = 0;

static void sendJsonConfig()
{
#if USE_BLE
    serializeJson(config_message, ble_output_buffer, WRITE_BUFFER_SIZE);
    configChar.writeValue(ble_output_buffer, WRITE_BUFFER_SIZE);
    Serial.println(ble_output_buffer);
#else
    serializeJson(config_message, Serial);
#endif //USE_BLE
}

static void setup_imu()
{
    if (!IMU.begin()) //Initialize IMU sensor
    {
        Serial.println("Failed to initialize IMU!");
        while (1)
            ;
    }
    //Set units.
    IMU.accelUnit = METERPERSECOND2;
    IMU.gyroUnit = DEGREEPERSECOND;
    IMU.magnetUnit = MICROTESLA;

#if ENABLE_ACCEL && (ENABLE_GYRO == 0)
    IMU.setAccelODR(accel_gyro_speed);
    IMU.setGyroODR(ACCEL_GYRO_ODR_OFF);
    config_message["sample_rate"] = (int)IMU.getAccelODR();
    config_message["Columns"]["AccelerometerX"] = column_index++;
    config_message["Columns"]["AccelerometerY"] = column_index++;
    config_message["Columns"]["AccelerometerZ"] = column_index++;

#elif (ENABLE_ACCEL && ENABLE_GYRO)
    IMU.setAccelODR(accel_gyro_speed);
    IMU.setGyroODR(accel_gyro_speed);
    config_message["sample_rate"] = (int)IMU.getAccelODR();
    config_message["Columns"]["AccelerometerX"] = column_index++;
    config_message["Columns"]["AccelerometerY"] = column_index++;
    config_message["Columns"]["AccelerometerZ"] = column_index++;
    config_message["Columns"]["GyroscopeX"] = column_index++;
    config_message["Columns"]["GyroscopeY"] = column_index++;
    config_message["Columns"]["GyroscopeZ"] = column_index++;
#else //gyro only
    IMU.setAccelODR(ACCEL_GYRO_ODR_OFF);
    IMU.setGyroODR(accel_gyro_speed);
#endif

#if ENABLE_MAG
    IMU.setMagnetODR(mag_speed);
    config_message["Columns"]["MagnetometerX"] = column_index++;
    config_message["Columns"]["MagnetometerY"] = column_index++;
    config_message["Columns"]["MagnetometerZ"] = column_index++;
#else
    IMU.setMagnetODR(0);
#endif
    IMU.setContinuousMode();
    actual_odr = IMU.getAccelODR();
}

#if USE_BLE
/*
 * LEDS
 */
void connectedLight()
{
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);
}

void disconnectedLight()
{
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW);
}

void onBLEConnected(BLEDevice central)
{
    Serial.print("Connected event, central: ");
    Serial.println(central.address());
    connectedLight();

}

void onBLEDisconnected(BLEDevice central)
{
    Serial.print("Disconnected event, central: ");
    Serial.println(central.address());
    disconnectedLight();
}

void onDataCharSubscribed(BLEDevice central, BLECharacteristic ch)
{
    if (ch.uuid() == sensorDataChar.uuid()) {
        config_received = true;
    }
}

static void setup_ble()
{
    if (!BLE.begin()) {
        Serial.println("starting BLE failed!");
        while (1)
            ;
    }

    BLE.setLocalName(nameOfPeripheral);
    BLE.setAdvertisedService(sensorService);
    BLE.setConnectionInterval(0x0001, 0x0002); //1.25 to 2.5ms
    BLE.noDebug();

    configChar.addDescriptor(configNameDescriptor);
    sensorDataChar.addDescriptor(sensorDataDescriptor);
    sensorService.addCharacteristic(configChar);
    sensorService.addCharacteristic(sensorDataChar);
    sendJsonConfig();
    delay(1000);
    BLE.addService(sensorService);

    // Bluetooth LE connection handlers.
    BLE.setEventHandler(BLEConnected, onBLEConnected);
    BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);

    sensorDataChar.setEventHandler(BLESubscribed, onDataCharSubscribed);

    BLE.advertise();
    BLE.setConnectable(true);

    Serial.println("Bluetooth device active, waiting for connections...");
}
#endif //#if USE_BLE

void setup()
{

#if USE_BLE
    setup_ble();
#endif
    Serial.begin(SERIAL_BAUD_RATE); //Serial monitor to display all sensor values
    setup_imu();
}

static void update_imu()
{
    //Accelerometer values
    if (IMU.accelerationAvailable() && ENABLE_ACCEL) {
        IMU.readRawAccelInt16(sensorRawData[sensorRawIndex++], sensorRawData[sensorRawIndex++], sensorRawData[sensorRawIndex++]);
    }

    //Gyroscope values
    if (IMU.gyroscopeAvailable() && ENABLE_GYRO) {
        IMU.readRawGyroInt16(sensorRawData[sensorRawIndex++], sensorRawData[sensorRawIndex++], sensorRawData[sensorRawIndex++]);
    }

    //Magnetometer values
    if (IMU.magneticFieldAvailable() && ENABLE_MAG) {
        IMU.readRawMagnetInt16(sensorRawData[sensorRawIndex++], sensorRawData[sensorRawIndex++], sensorRawData[sensorRawIndex++]);
    }
}

void loop()
{
#if USE_BLE

    delay(1000);
    BLEDevice central = BLE.central();
    if (central) {
        if (central.connected()) {
            connectedLight();
            if (!config_received) {

                return;
            }
        }
    } else {
        disconnectedLight();
    }
#else
    if (!config_received) {
        sendJsonConfig();
        delay(1000);
        int x = Serial.available();
        if (x > 0) {
            String rx = Serial.readString();
            Serial.println(rx);
            if (rx.equals("sensiml")) {
                config_received = true;
            }
            config_message.clear();
        }
        return;
    }
#endif
    update_imu();

#if USE_BLE
    sensorDataChar.writeValue((void*)sensorRawData, sensorRawIndex * sizeof(int16_t));
#else
    Serial.write((uint8_t*)sensorRawData, sensorRawIndex * sizeof(int16_t));
#endif //USE_BLE
    sensorRawIndex = 0;
    memset(sensorRawData, 0, MAX_NUMBER_OF_COLUMNS * sizeof(int16_t));
    delay(1000 / (long)actual_odr);
}
