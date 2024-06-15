//BLE TEST//
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "BlueNRG1_conf.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "serial_port.h"
#include "clock.h"
#include "SDK_EVAL_Config.h"
#include "SerialPort_config.h"
#include "adxl362.h"
#include "adxl362_hal.h"
#include "BlueNRG1_rtc.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define BLE_NEW_SERIALPORT_VERSION_STRING "2.0.0"

#ifndef DEBUG
#define DEBUG 1
#endif

/* Private macros ------------------------------------------------------------*/
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

// Variables for DMA operation
volatile FlagStatus spi_eot;
extern uint8_t buffer_spi_tail;
extern volatile uint8_t buffer_spi_used;

#define PREPROC_BUFFER_SIZE 250 // Number of samples in a batch
#define BATCH_SIZE 25 //25 readings inner batch
RTC_DateTimeType batchTimestamps[10]; //Timestamping preprocessed data

typedef struct{
    struct imu_type imuData[PREPROC_BUFFER_SIZE]; // Array to hold a batch of accelerometer data
    RTC_DateTimeType timestamp; // Timestamp for the batch
} DataBatch;

DataBatch dataBatch;
int dataBatchIndex = 0; // Index for batch logic

extern volatile uint32_t msTicks; //msTicks is updated by SysTick_Handler


/*void Process_InputData(uint8_t * rx_data, uint16_t data_size)
{
    // Process input data received from Bluetooth (if needed)
}*/

int main(void)
{
    uint8_t ret;


    SystemInit();
    SdkEvalIdentification();
    SdkEvalComUartInit(UART_BAUDRATE);


    if (SdkEvalSpiDmaInit(4000000) != SUCCESS) {
        printf("Error initializing DMA communication.\r\n");
    }

    SysTick_Config(SYST_CLOCK/1000);


    AdxlSetStandByModeDma();
    AdxlSetRangeModeDma();
    AdxlSetMeasurementModeDma();


    RTC_Configuration();
    RTC_Clockwatch_Configuration();


    SdkEvalIdentification();


    Clock_Init();


    SdkEvalComIOConfig(Process_InputData);


    ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
    if (ret != BLE_STATUS_SUCCESS) {
        printf("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
        while (1);
    }


    ret = SerialPort_DeviceInit();
    if (ret != BLE_STATUS_SUCCESS) {
        printf("SerialPort_DeviceInit()--> Failed 0x%02x\r\n", ret);
        while (1);
    }

    printf("BLE Stack Initialized & Device Configured\r\n");

    uint32_t lastTick = 0;
    while (1) {

        BTLE_StackTick();

        // Sample IMU data every 40ms to get 25 samples per second
        if ((msTicks - lastTick) >= 40) {
            lastTick = msTicks;

            uint8_t status = AdxlReadRegisterDma(ADXL_STATUS);
            if (status & ADXL_DATA_READY) {
                struct imu_type* imuR = AdxlReadAxesDma();
                if (imuR != NULL) {
                    // Copy data to the current batch
                    dataBatch.imuData[dataBatchIndex] = *imuR;
                    dataBatchIndex++;

                    if (dataBatchIndex == PREPROC_BUFFER_SIZE) {
                        for (int batch = 0; batch < PREPROC_BUFFER_SIZE / BATCH_SIZE; batch++) {
                            // Timestamp the start of each batch processing
                            RTC_GetTimeDate(&batchTimestamps[batch]);
                            printf("Batch %d Timestamp: %02d:%02d:%02d\r\n", batch+1,
                                    batchTimestamps[batch].Hour, batchTimestamps[batch].Minute, batchTimestamps[batch].Second);

                            // Process or print each batch here
                            for (int i = 0; i < BATCH_SIZE; i++) {
                                int index = batch * BATCH_SIZE + i;
                                printf("X-axis: %d, Y-axis: %d, Z-axis: %d\r\n",
                                       dataBatch.imuData[index].xAxis, dataBatch.imuData[index].yAxis, dataBatch.imuData[index].zAxis);
                            }
                        }

                        // Reset index for next total buffer filling
                        dataBatchIndex = 0;
                    }

                    // When batch is full, timestamp it, print, and reset for the next batch
                    if (dataBatchIndex == PREPROC_BUFFER_SIZE) {
                        RTC_GetTimeDate(&dataBatch.timestamp); // Timestamp the batch
                        printf("Batch Timestamp: %02d:%02d:%02d\r\n", dataBatch.timestamp.Hour, dataBatch.timestamp.Minute, dataBatch.timestamp.Second);
                        for (int i = 0; i < PREPROC_BUFFER_SIZE; i++) {
                            printf("X-axis: %d, Y-axis: %d, Z-axis: %d\r\n",
                                   dataBatch.imuData[i].xAxis, dataBatch.imuData[i].yAxis, dataBatch.imuData[i].zAxis);
                        }
                        dataBatchIndex = 0; // Reset index for next batch
                    }
                }
            }
        }

        // Process buffered SPI data
        if (buffer_spi_used > 0) {
            while (buffer_spi_used > 0) {
                buffer_spi_tail = (buffer_spi_tail + 1) % SPI_BUFF_SIZE;
                __disable_irq();
                buffer_spi_used--;
                __enable_irq();
            }
        }
    }
}


#ifdef USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printff("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif
