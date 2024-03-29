#include <sensor_config.h>
#include <ArduinoJson.h>
#include <Arduino_LSM9DS1.h>

#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG

int actual_odr;
int16_t sensorRawData[MAX_SAMPLES_PER_PACKET*MAX_NUMBER_OF_COLUMNS];


static int get_acc_gyro_odr()
{
    switch (ACCEL_GYRO_DEFAULT_ODR)
    {
        case ACCEL_GYRO_ODR_OFF:
            return 0;
        case ACCEL_GYRO_ODR_10HZ:
            return 10;
        case ACCEL_GYRO_ODR_50HZ:
            return 50;
        case ACCEL_GYRO_ODR_119HZ:
            return 119;
        case ACCEL_GYRO_ODR_238HZ:
            return 238;
        case ACCEL_GYRO_ODR_476HZ:
            return 476;
    }
}

int update_imu(int startIndex)
{
    int sensorRawIndex = startIndex;
    if (ENABLE_ACCEL)
    {
        IMU.readRawAccelInt16(sensorRawData[sensorRawIndex++],
                              sensorRawData[sensorRawIndex++],
                              sensorRawData[sensorRawIndex++]);
    }

    if (ENABLE_GYRO)
    {
        IMU.readRawGyroInt16(sensorRawData[sensorRawIndex++],
                             sensorRawData[sensorRawIndex++],
                             sensorRawData[sensorRawIndex++]);
    }

    if (ENABLE_MAG)
    {
        IMU.readRawMagnetInt16(sensorRawData[sensorRawIndex++],
                               sensorRawData[sensorRawIndex++],
                               sensorRawData[sensorRawIndex++]);
    }
    return sensorRawIndex;
}

int16_t* get_imu_pointer()
{
    return &sensorRawData[0];
}

int setup_imu(JsonDocument& config_message, int column_start)
{

    int column_index = column_start;
#if SERIAL_DEBUG    
    Serial.println("Setting up IMU");
#endif    
if (!IMU.begin())  // Initialize IMU sensor
    {  
        while(1){
#if SERIAL_DEBUG
            Serial.println("IMU failed to Initlialize! Aborting..");
#endif
            delay(1000);
        }
    }
#if SERIAL_DEBUG
    Serial.println("Setting up IMU ODR values");
#endif
    // Set units.
    IMU.accelUnit  = METERPERSECOND2;
    IMU.gyroUnit   = DEGREEPERSECOND;
    IMU.magnetUnit = MICROTESLA;

#if ENABLE_ACCEL && (ENABLE_GYRO == 0)
    IMU.setAccelODR(ACCEL_GYRO_DEFAULT_ODR);
    IMU.setGyroODR(ACCEL_GYRO_ODR_OFF);

    config_message["column_location"]["AccelerometerX"] = column_index++;
    config_message["column_location"]["AccelerometerY"] = column_index++;
    config_message["column_location"]["AccelerometerZ"] = column_index++;
    actual_odr                                          = get_acc_gyro_odr();
    config_message["sample_rate"]                       = actual_odr;

#elif (ENABLE_ACCEL && ENABLE_GYRO)
    IMU.setAccelODR(ACCEL_GYRO_DEFAULT_ODR);
    IMU.setGyroODR(ACCEL_GYRO_DEFAULT_ODR);
    actual_odr                                          = get_acc_gyro_odr();
    config_message["sample_rate"]                       = actual_odr;
    config_message["column_location"]["AccelerometerX"] = column_index++;
    config_message["column_location"]["AccelerometerY"] = column_index++;
    config_message["column_location"]["AccelerometerZ"] = column_index++;
    config_message["column_location"]["GyroscopeX"]     = column_index++;
    config_message["column_location"]["GyroscopeY"]     = column_index++;
    config_message["column_location"]["GyroscopeZ"]     = column_index++;
    actual_odr                                          = get_acc_gyro_odr();
#else  // gyro only
    IMU.setAccelODR(ACCEL_GYRO_ODR_OFF);
    IMU.setGyroODR(ACCEL_GYRO_DEFAULT_ODR);
    actual_odr                                      = get_acc_gyro_odr();
    config_message["sample_rate"]                   = actual_odr;
    config_message["column_location"]["GyroscopeX"] = column_index++;
    config_message["column_location"]["GyroscopeY"] = column_index++;
    config_message["column_location"]["GyroscopeZ"] = column_index++;
#endif

#if ENABLE_MAG
    IMU.setMagnetODR(MAG_DEFAULT_ODR);
    config_message["column_location"]["MagnetometerX"] = column_index++;
    config_message["column_location"]["MagnetometerY"] = column_index++;
    config_message["column_location"]["MagnetometerZ"] = column_index++;
#else
    IMU.setMagnetODR(0);
#endif
    IMU.setContinuousMode();
    return column_index;
}

#endif  //#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG
