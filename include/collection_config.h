#ifndef __COLLECTION_CONFIG_H__
#define __COLLECTION_CONFIG_H__

/**
 *
 * BLE Settings
 *
 */
#define USE_BLE 0

const char* nameOfPeripheral = "Nano 33 DCL";
const char* uuidOfService = "16480000-0525-4ad5-b4fb-6dd83f49546b";
const char* uuidOfConfigChar = "16480001-0525-4ad5-b4fb-6dd83f49546b";
const char* uuidOfDataChar = "16480002-0525-4ad5-b4fb-6dd83f49546b";


const int WRITE_BUFFER_SIZE = 256;
bool WRITE_BUFFER_FIXED_LENGTH = false;


/**
 *
 * Serial Port Settings
 *
 */
#define SERIAL_BAUD_RATE 115200

/**
 *
 * IMU Settings
 *
 */

// IMU Sensors Enable/Disable
#define ENABLE_ACCEL 1
#define ENABLE_GYRO 1
#define ENABLE_MAG 0

//IMU ODR settings. Note: Gyroscope and Accel are linked.
typedef enum {
    //Sample Rate 0:off, 1:10Hz, 2:50Hz, 3:119Hz, 4:238Hz, 5:476Hz, 6:952Hz
    ACCEL_GYRO_ODR_OFF = 0,
    ACCEL_GYRO_ODR_10HZ = 1,
    ACCEL_GYRO_ODR_50HZ = 2,
    ACCEL_GYRO_ODR_119HZ = 3,
    ACCEL_GYRO_ODR_238HZ = 4,
    ACCEL_GYRO_ODR_476HZ = 5,
   // ACCEL_GYRO_ODR_952HZ = 6
} accel_gyro_odr_t;

//Mag ODR settings.
typedef enum {
    // range (0..8) = {0.625,1.25,2.5,5,10,20,40,80,400}Hz
    MAG_ODR_0_625HZ = 0,
    MAG_ODR_1_25HZ,
    MAG_ODR_2_5HZ,
    MAG_ODR_5HZ,
    MAG_ODR_10HZ,
    MAG_ODR_20HZ,
    MAG_ODR_40HZ,
    MAG_ODR_80HZ,
    MAG_ODR_400HZ
} mag_odr_t;

//Default sample rates. Note: Mag will be read at a higher rate.
#define ACCEL_GYRO_DEFAULT_ODR ACCEL_GYRO_ODR_119HZ
#define MAG_DEFAULT_ODR MAG_ODR_20HZ



#endif //__COLLECTION_CONFIG_H__
