
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Arduino_LSM9DS1.h>   //Include the library for 9-axis IMU
#include "sensor_config.h"

#if USE_BLE
#include <ArduinoBLE.h>
const char* nameOfPeripheral = "Nano 33 DCL";
const char* uuidOfService    = "16480000-0525-4ad5-b4fb-6dd83f49546b";
const char* uuidOfConfigChar = "16480001-0525-4ad5-b4fb-6dd83f49546b";
const char* uuidOfDataChar   = "16480002-0525-4ad5-b4fb-6dd83f49546b";

bool WRITE_BUFFER_FIXED_LENGTH = false;

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


static int8_t ble_output_buffer[WRITE_BUFFER_SIZE];

extern int actual_odr;

static bool          config_received = false;
static unsigned long currentMs, previousMs;
static long          interval = 0;
extern volatile int  samplesRead;

DynamicJsonDocument config_message(256);
#if USE_SECOND_SERIAL_PORT_FOR_OUTPUT
auto& dataOutSerial = Serial1;
#else
auto& dataOutSerial = Serial;
#endif //USE_SECOND_SERIAL_PORT_FOR_OUTPUT

int column_index = 0;

static void sendJsonConfig()
{
#if USE_BLE
    serializeJson(config_message, ble_output_buffer, WRITE_BUFFER_SIZE);
    configChar.writeValue(ble_output_buffer, WRITE_BUFFER_SIZE);
#else
    serializeJson(config_message, (void *)ble_output_buffer, WRITE_BUFFER_SIZE);
    dataOutSerial.println((char*)ble_output_buffer);
    dataOutSerial.flush();
#endif  // USE_BLE
}

#if USE_BLE
/*
 * LEDS
 */
void connectedLight()
{
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW);
}

void disconnectedLight()
{
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);

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

    BLE.advertise();

    Serial.println("Bluetooth device active, waiting for connections...");
}
#endif  //#if USE_BLE

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    delay(2000);
    Serial.println("Setting up...");
#if USE_SECOND_SERIAL_PORT_FOR_OUTPUT
    Serial1.begin(SERIAL_BAUD_RATE);
#endif //USE_SECOND_SERIAL_PORT_FOR_OUTPUT

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
    config_message["samples_per_packet"] = MAX_SAMPLES_PER_PACKET;

    delay(1000);
    sendJsonConfig();
}


static int packetNum      = 0;
static int sensorRawIndex = 0;
void       loop()
{
    currentMs = millis();
#if USE_BLE

    BLEDevice central = BLE.central();
    if (central)
    {
        if (central.connected())
        {
            connectedLight();
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
        if (dataOutSerial.available() > 0)
        {
            String rx = dataOutSerial.readString();
            Serial.println(rx);
            if (rx.equals("connect") || rx.equals("cnnect"))
            {
                #if USE_SECOND_SERIAL_PORT_FOR_OUTPUT
                Serial.println("Got Connect message");

                #endif
                config_received = true;
            }
        }
    }

    else
    {
        if (dataOutSerial.available() > 0)
        {
            String rx = dataOutSerial.readString();
            if( rx.equals("disconnect"))
            {
                config_received = false;
            }
        }
#endif
        if (currentMs - previousMs >= interval)
        {
            // save the last time you blinked the LED
            previousMs = currentMs;
    #if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG
            sensorRawIndex = update_imu(sensorRawIndex);
            packetNum++;
            int16_t* pData = get_imu_pointer();
    #if USE_BLE
            sensorDataChar.writeValue((void*) pData, sensorRawIndex * sizeof(int16_t));
            sensorRawIndex = 0;
            memset(pData, 0, MAX_NUMBER_OF_COLUMNS * MAX_SAMPLES_PER_PACKET * sizeof(int16_t));
    #else
            if (packetNum == MAX_SAMPLES_PER_PACKET)
            {
                dataOutSerial.write((uint8_t*) pData, sensorRawIndex * sizeof(int16_t));
                dataOutSerial.flush();
                sensorRawIndex = 0;
                memset(pData, 0, MAX_NUMBER_OF_COLUMNS * MAX_SAMPLES_PER_PACKET * sizeof(int16_t));
                packetNum = 0;
            }
    #endif  //#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG
    #endif  // USE_BLE

    #if ENABLE_AUDIO
            if (samplesRead)
            {
                dataOutSerial.write(getSampleBuffer(), samplesRead * 2);
                dataOutSerial.flush();
                samplesRead = 0;
            }
    #endif  // ENABLE_AUDIO
        }
    }
#if USE_BLE==0
} //loop()
#endif
