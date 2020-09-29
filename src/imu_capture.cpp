#include <sensor_config.h>
#include <ArduinoJson.h>

#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG

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

void update_imu()
{
    // Accelerometer values IMU.accelerationAvailable() &&
    if (ENABLE_ACCEL)
    {
        IMU.readRawAccelInt16(sensorRawData[sensorRawIndex++],
                              sensorRawData[sensorRawIndex++],
                              sensorRawData[sensorRawIndex++]);
    }

    // Gyroscope values IMU.gyroscopeAvailable() &&
    if (ENABLE_GYRO)
    {
        IMU.readRawGyroInt16(sensorRawData[sensorRawIndex++],
                             sensorRawData[sensorRawIndex++],
                             sensorRawData[sensorRawIndex++]);
    }

    // Magnetometer values IMU.magneticFieldAvailable() &&
    if (ENABLE_MAG)
    {
        IMU.readRawMagnetInt16(sensorRawData[sensorRawIndex++],
                               sensorRawData[sensorRawIndex++],
                               sensorRawData[sensorRawIndex++]);
    }
}

int setup_imu(DynamicJsonDocument& config_message, int column_start)
{
    int column_index = column_start;
    if (!IMU.begin())  // Initialize IMU sensor
    {
        Serial.println("Failed to initialize IMU!");
        while (1)
            ;
    }
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
    actual_odr                                            = get_acc_gyro_odr();
    config_message["sample_rate"]                         = actual_odr;
    v config_message["column_location"]["AccelerometerX"] = column_index++;
    config_message["column_location"]["AccelerometerY"]   = column_index++;
    config_message["column_location"]["AccelerometerZ"]   = column_index++;
    config_message["column_location"]["GyroscopeX"]       = column_index++;
    config_message["column_location"]["GyroscopeY"]       = column_index++;
    config_message["column_location"]["GyroscopeZ"]       = column_index++;
    actual_odr                                            = get_acc_gyro_odr();
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
}

#endif  //#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG
