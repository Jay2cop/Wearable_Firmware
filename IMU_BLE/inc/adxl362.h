#ifndef ADXL362_H_
#define ADXL362_H_

#include "adxl362_hal.h"
//#include <stdint.h>

#define UPLINK_BUFFER_MAX 300

// Error codes
#define ADXL_READ_SUCCESS      0
#define ADXL_READ_ERROR       -1

// ADXL362 Commands
#define ADXL_WRITE             0x0A
#define ADXL_READ              0x0B
#define ADXL_READ_FIFO         0x0D

// ADXL362 Register Addresses
#define ADXL_DEVID_AD           0x00
#define ADXL_STATUS             0x0B
#define ADXL_SELF_TEST          0x2E
#define ADXL362_REG_SOFT_RESET  0x52
#define ADXL362_SOFT_RESET_KEY  0x1F
#define ADXL362_REG_INTMAP1     0x2A
#define ADXL362_REG_FILTER_CTL  0x2C
#define ADXL362_REG_POWER_CTL   0x2D
#define ADXL362_MEASURE_ON	    0x02
#define ADXL362_MEASURE_STANDBY 0x00

//Self_test bits
#define ADXL_SELF_TEST_OFF		0
#define ADXL_SELF_TEST_ON 		1
#define ADXL_HALF_BW			1

// ADXL362 Bit Flags
#define ADXL_DATA_READY        0x40
#define ADXL_FIFO_READY        0x02
#define ADXL_FIFO_WATERMARK    0x04
#define ADXL_FIFO_OVERRUN      0x08
#define ADXL_ACTIVE            0x10
#define ADXL_INACTIVE          0x20
#define ADXL_AWAKE             0x40
#define ADXL_ERR               0x80

// ADXL362 Range Settings
#define ADXL362_RANGE_2G       0
#define ADXL362_RANGE_4G       1
#define ADXL362_RANGE_8G       2

// ADXL362 Output Data Rate Settings
#define ADXL362_ODR_12_5_HZ    0
#define ADXL362_ODR_25_HZ      1
#define ADXL362_ODR_50_HZ      2
#define ADXL362_ODR_100_HZ     3
#define ADXL362_ODR_200_HZ     4
#define ADXL362_ODR_400_HZ     5

typedef struct imu_type {
    int16_t xAxis;
    int16_t yAxis;
    int16_t zAxis;
} imu_type_t;

typedef enum { false, true } bool; // Needed as the compiler does not support boolean

typedef enum {
    SPI_SUCCESS,  // Indicates successful SPI transaction
    SPI_ERROR     // Indicates error in SPI transaction
} SPI_Status;

// High-level sensor management functions
void AdxlSetMeasurementModeDma(void);
void AdxlSetStandByModeDma(void);
void AdxlSetRangeModeDma();
void AdxlSelfTest(void);
struct imu_type *AdxlReadAxesDma();
ErrorStatus SdkEvalSpiDmaRead(uint8_t* pBuffer, uint8_t RegisterAddr, uint8_t NumByteToRead);
ErrorStatus SdkEvalSpiDmaWrite(uint8_t* pBuffer, uint8_t RegisterAddr, uint8_t NumByteToWrite);

#endif /* ADXL362_H_ */
