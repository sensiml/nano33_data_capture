
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Arduino_APDS9960.h>  //Include library for colour, proximity and gesture recognition
#include <Arduino_HTS221.h>    //Include library to read Temperature and Humidity
#include <Arduino_LPS22HB.h>   //Include library to read Pressure
#include <Arduino_LSM9DS1.h>   //Include the library for 9-axis IMU

#include "sensor_config.h"


#if USE_BLE
#define MAX_SAMPLES_PER_PACKET 1
#include <ArduinoBLE.h>

// BLE Service
BLEService sensorService(uuidOfService);

// RX / TX Characteristics
BLECharacteristic configChar(uuidOfConfigChar,
                             BLERead,
                             WRITE_BUFFER_SIZE,
                             WRITE_BUFFER_FIXED_LENGTH);
BLEDescriptor     configNameDescriptor("2901", "Sensor Configuration");
BLECharacteristic sensorDataChar(uuidOfDataChar,
                                 BLERead | BLENotify,
                                 20,
                                 WRITE_BUFFER_FIXED_LENGTH);
BLEDescriptor     sensorDataDescriptor("2901", "Sensor Data TX");
#else
#endif  // USE_BLE


static char ble_output_buffer[WRITE_BUFFER_SIZE];

static int actual_odr;

static bool          config_received = false;
static unsigned long currentMs, previousMs;
static long          interval = 0;
extern volatile int  samplesRead;


DynamicJsonDocument config_message(256);

int column_index = 0;

static void sendJsonConfig()
{
#if USE_BLE
    serializeJson(config_message, ble_output_buffer, WRITE_BUFFER_SIZE);
    configChar.writeValue(ble_output_buffer, WRITE_BUFFER_SIZE);
#else
    serializeJson(config_message, ble_output_buffer, WRITE_BUFFER_SIZE);
    Serial.println(ble_output_buffer);
    Serial.flush();
#endif  // USE_BLE
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
    BLE.setConnectable(true);
}

void onDataCharSubscribed(BLEDevice central, BLECharacteristic ch)
{
    if (ch.uuid() == sensorDataChar.uuid())
    {
        config_received = true;
    }
}

static void setup_ble()
{
    if (!BLE.begin())
    {
        Serial.println("starting BLE failed!");
        while (1)
            ;
    }

    BLE.setLocalName(nameOfPeripheral);
    BLE.setAdvertisedService(sensorService);
    BLE.setConnectionInterval(0x0006, 0x0007);  // 1.25 to 2.5ms
    BLE.noDebug();

    configChar.addDescriptor(configNameDescriptor);
    sensorDataChar.addDescriptor(sensorDataDescriptor);
    sensorService.addCharacteristic(configChar);
    sensorService.addCharacteristic(sensorDataChar);

    delay(1000);
    BLE.addService(sensorService);

    // Bluetooth LE connection handlers.
    BLE.setEventHandler(BLEConnected, onBLEConnected);
    BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);

    sensorDataChar.setEventHandler(BLESubscribed, onDataCharSubscribed);

    BLE.advertise();

    Serial.println("Bluetooth device active, waiting for connections...");
}
#endif  //#if USE_BLE

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println("Setting up...");

#if USE_BLE
    setup_ble();
#endif  // USE_BLE

#if ENABLE_AUDIO
    column_index += setup_audio(config_message, column_index);
    interval = 0;
#endif
#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG
    column_index += setup_imu(config_message, column_index);
    interval = (1000 / (long) actual_odr);
#endif

    delay(1000);
    sendJsonConfig();
}


static int packetNum = 0;
void       loop()
{
    currentMs = millis();
#if USE_BLE

    delay(1000);
    BLEDevice central = BLE.central();
    if (central)
    {
        if (central.connected())
        {
            connectedLight();
            if (!config_received)
            {
                return;
            }
        }
    }
    else
    {
        disconnectedLight();
    }
#else
    if (!config_received)
    {
        sendJsonConfig();
        delay(1000);
        if (Serial.available() > 0)
        {
            String rx = Serial.readString();
            if (rx.equals("connect"))
            {
                config_received = true;
            }
        }
    }

    else
    {

#endif
    if (currentMs - previousMs >= interval)
    {
        // save the last time you blinked the LED
        previousMs = currentMs;
#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG
        update_imu();
        packetNum++;
#if USE_BLE
        sensorDataChar.writeValue((void*) sensorRawData, sensorRawIndex * sizeof(int16_t));
        sensorRawIndex = 0;
        memset(sensorRawData, 0, MAX_NUMBER_OF_COLUMNS * MAX_SAMPLES_PER_PACKET * sizeof(int16_t));
#else
        if (packetNum == MAX_SAMPLES_PER_PACKET)
        {
            Serial.write((uint8_t*) sensorRawData, sensorRawIndex * sizeof(int16_t));
            Serial.flush();
            sensorRawIndex = 0;
            memset(
                sensorRawData, 0, MAX_NUMBER_OF_COLUMNS * MAX_SAMPLES_PER_PACKET * sizeof(int16_t));
            packetNum = 0;
        }
#endif  //#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG
#endif  // USE_BLE
#if ENABLE_AUDIO
        if (samplesRead)
        {
            Serial.write(getSampleBuffer(), samplesRead * 2);
            Serial.flush();
            samplesRead = 0;
        }
#endif  // ENABLE_AUDIO
    }
}
}
