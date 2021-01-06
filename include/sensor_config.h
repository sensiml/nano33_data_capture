#ifndef __SENSOR_CONFIG_H__
#define __SENSOR_CONFIG_H__
#include <ArduinoJson.h>


/**
 * BLE Settings
 */
#define USE_BLE 1


#define ENABLE_AUDIO 0

// IMU Sensors Enable/Disable
#define ENABLE_ACCEL 1
#define ENABLE_GYRO 1
#define ENABLE_MAG 0

#if USE_BLE
#define MAX_NUMBER_OF_COLUMNS 10
#define MAX_SAMPLES_PER_PACKET 1
#else
#define MAX_NUMBER_OF_COLUMNS 20
<<<<<<< HEAD
#define MAX_SAMPLES_PER_PACKET 10

#endif //USE_BLE
=======
#define MAX_SAMPLES_PER_PACKET 6
#endif  // USE_BLE
>>>>>>> e29a476... ble implementation

/**
 * Serial Port Settings
 */
<<<<<<< HEAD
#define USE_SECOND_SERIAL_PORT_FOR_OUTPUT 1

#if USE_SECOND_SERIAL_PORT_FOR_OUTPUT
#define SERIAL_BAUD_RATE 115200 *4
#else
#define SERIAL_BAUD_RATE 115200 * 8
#endif //USE_SECOND_SERIAL_PORT_FOR_OUTPUT

=======
#define SERIAL_BAUD_RATE 115200 * 8
>>>>>>> e29a476... ble implementation
const int WRITE_BUFFER_SIZE = 256;

/**
 * IMU Settings
 */


#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG
int      setup_imu(JsonDocument& config_message, int column_start);
int16_t* get_imu_pointer();
int      update_imu(int startIndex);
// IMU ODR settings. Note: Gyroscope and Accel are linked.
typedef enum
{
    // Sample Rate 0:off, 1:10Hz, 2:50Hz, 3:119Hz, 4:238Hz, 5:476Hz, 6:952Hz
    ACCEL_GYRO_ODR_OFF   = 0,
    ACCEL_GYRO_ODR_10HZ  = 1,
    ACCEL_GYRO_ODR_50HZ  = 2,
    ACCEL_GYRO_ODR_119HZ = 3,
    ACCEL_GYRO_ODR_238HZ = 4,
    ACCEL_GYRO_ODR_476HZ = 5,
    // ACCEL_GYRO_ODR_952HZ = 6
} accel_gyro_odr_t;

// Mag ODR settings.
typedef enum
{
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

<<<<<<< HEAD
//Default sample rates. Note: Mag will be read at a higher rate.
#define ACCEL_GYRO_DEFAULT_ODR ACCEL_GYRO_ODR_238HZ
=======
// Default sample rates. Note: Mag will be read at a higher rate.
#define ACCEL_GYRO_DEFAULT_ODR ACCEL_GYRO_ODR_119HZ
>>>>>>> e29a476... ble implementation
#define MAG_DEFAULT_ODR MAG_ODR_20HZ

#endif  //#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG

#define ENABLE_AUDIO 0	#endif  //__SENSOR_CONFIG_H__
#if ENABLE_AUDIO	
#define AUDIO_SAMPLE_RATE 16000	
int setup_audio(JsonDocument& config_message, int column_start);	
uint8_t* getSampleBuffer();	
#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG	
#warning “Audio and IMU are enabled. only audio will be used”	
#undef ENABLE_ACCEL	
#undef ENABLE_GYRO	
#undef ENABLE_MAG	
#define ENABLE_ACCEL 0	
#define ENABLE_GYRO  0	
#define ENABLE_MAG   0	
#endif //#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG	
#endif //ENABLE_AUDIO

<<<<<<< HEAD
#define ENABLE_AUDIO 1
#if ENABLE_AUDIO
#define AUDIO_SAMPLE_RATE 16000
int setup_audio(JsonDocument& config_message, int column_start);
uint8_t* getSampleBuffer();
#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG
#warning "Audio and IMU are enabled. only audio will be used"
#undef ENABLE_ACCEL
#undef ENABLE_GYRO
#undef ENABLE_MAG
#define ENABLE_ACCEL 0
#define ENABLE_GYRO  0
#define ENABLE_MAG   0

#undef MAX_SAMPLES_PER_PACKET
#define MAX_SAMPLES_PER_PACKET 128
#endif //#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG

#endif //ENABLE_AUDIO

#endif //__SENSOR_CONFIG_H__
=======
#endif  //__SENSOR_CONFIG_H__
>>>>>>> e29a476... ble implementation
