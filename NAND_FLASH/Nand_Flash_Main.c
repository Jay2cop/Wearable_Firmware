/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : SPI/Master_Polling/main.c
* Author             : RF Application Team
* Version            : V1.0.0
* Date               : September-2015
* Description        : Code demostrating the SPI master functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_x_device.h"
#include <stdio.h>
#include "BlueNRG1_conf.h"
#include "BlueNRG1_it.h"
#include "SDK_EVAL_Config.h"
#include "SDK_EVAL_SPI.h"
#include "adxl362.h"
#include "adxl362_hal.h"
#include "winbond_w25n01gwzeig_HAL.h"
#include "winbond_w25n01gwzeig.h"
#include <string.h>

/** @addtogroup BlueNRG1_StdPeriph_Examples BlueNRG1 Standard Peripheral Examples
* @{
*/

/** @addtogroup SPI_Examples SPI Examples
* @{
*/

/** @addtogroup SPI_Master_DMA SPI Master DMA
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*
#define TOGGLE_DL1                         '1'
#define TOGGLE_DL2                         '2'
#define TOGGLE_DL3                         '3'
#define TOGGLE_ALL_LED                     '4'
*/

/* Variables used for DMA operation */
//volatile FlagStatus spi_eot;
//extern uint8_t buffer_spi_tail;
//extern volatile uint8_t buffer_spi_used;
/* Private variables ---------------------------------------------------------*/
//extern volatile FlagStatus GpioIrqFlag;
//extern volatile uint8_t irqFlag;
//extern volatile uint32_t irqTick;
//extern volatile uint8_t bufferFull;

//extern volatile uint8_t spi_buffer_tx[SPI_BUFF_SIZE];
//extern volatile uint8_t spi_buffer_rx[SPI_BUFF_SIZE];

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
//void processCommand(void);
//void helpMessage(void);
//ErrorStatus SPI_Master_Configuration(uint32_t baudrate);
//void DMASpi_Sending(uint32_t buffer, uint32_t size) ;

/* Private functions ---------------------------------------------------------*/

/**
* @brief  Main program code
* @param  None
* @retval None
*/

//#define BUFFER_SIZE 50
//int bufferIndex = 0;
//struct imu_type imuBuffer[BUFFER_SIZE];
//extern volatile uint32_t msTicks;
//extern volatile uint8_t rtcTickOccurred;

//#define TIMESTAMP_BUFFER_SIZE 10
//RTC_DateTimeType timestampBuffer[TIMESTAMP_BUFFER_SIZE];
//int timestampIndex = 0;
//typedef struct {
 //   struct imu_type imuData[BUFFER_SIZE]; // Accelerometer data
 //   RTC_DateTimeType timestamp; // Timestamp for the batch
//} TimestampedDataBatch;

//int main (void){
//	return 0;
//}

//**  WORKING READING AND WRITTING INTO NAND FLASH TEST**//

int main(void) {
	char data[] = "Third year project"; //setting up what to write into the memory
	char readBuffer[128] = {0}; // Initialize the buffer to zero
	uint16_t columnAddress = 0; // Starting at the beginning of the page
	uint32_t pageAddress = 0; // Specify the page address to write to
	char idBuffer[20][7];
		 uint8_t iter = 1;

	//system initilization
	SystemInit();

	SdkEvalIdentification();
	//UART configuration
	SdkEvalComUartInit(UART_BAUDRATE);

	//SPI initialization for NAND
	if (SpiDmaInitNAND(8000000) != SUCCESS) {
		printf("Error initializing DMA communication.\r\n");
	}
	//Configure SysTick to generate interrupt with 1ms period
	SysTick_Config(SYST_CLOCK/1000);

	//Print ID
	for(int i = 0; i < iter; i++) {
		HAL_SPI_ReadManufacturerDeviceID(idBuffer, i);
	}
	for(int i = 0; i < iter; i++) {
		printf("ID: %s\r\n", idBuffer[i]);
	}

	NAND_WriteEnable();
	NAND_BulkErase();
	//NAND_BlockErase(pageAddress);
	printf("Data to be written: %s\r\n", data);
	NAND_LoadProgData(columnAddress, data, sizeof(data) - 1);
	NAND_ProgramExecute(pageAddress);
	NAND_Read(pageAddress, columnAddress, readBuffer, sizeof(readBuffer) - 1);
	readBuffer[sizeof(data) - 1] = '\0';
	printf("Read back data: %s\r\n", readBuffer);
	return 0;
}

//** Clean NAND working id read of the FLASH loop **//
/*
int main(void) {
	SystemInit();
	SdkEvalIdentification();
	SdkEvalComUartInit(UART_BAUDRATE);

	//SPI initialization for NAND
	if (SpiDmaInitNAND(8000000) != SUCCESS) {
		printf("Error initializing DMA communication.\r\n");
	}
	//Configure SysTick to generate interrupt with 1ms period
	SysTick_Config(SYST_CLOCK/1000);
	//NAND_Reset();
	 char idBuffer[20][7]; // Buffer to store 20 IDs, each 6 chars + null terminator
	 uint8_t iter = 1;
		for(int i = 0; i < iter; i++) {
			HAL_SPI_ReadManufacturerDeviceID(idBuffer, i);
		}

		// Print the buffer
		for(int i = 0; i < iter; i++) {
			printf("ID: %s\r\n", idBuffer[i]);
		}

		return 0;
}
*/
//Clean code:
/*
// Variables for DMA operation
volatile FlagStatus spi_eot;
extern uint8_t buffer_spi_tail;
extern volatile uint8_t buffer_spi_used;

// External variables
//extern volatile uint8_t irqFlag;
//extern volatile uint32_t irqTick;
//extern volatile uint8_t bufferFull;
//extern volatile uint8_t spi_buffer_tx[SPI_BUFF_SIZE];
//extern volatile uint8_t spi_buffer_rx[SPI_BUFF_SIZE];

#define PREPROC_BUFFER_SIZE 250 // Number of samples in a batch
#define BATCH_SIZE 25 //25 readings inner batch
RTC_DateTimeType batchTimestamps[10]; //Timestamping preprocessed data

typedef struct {
    struct imu_type imuData[PREPROC_BUFFER_SIZE]; // Array to hold a batch of accelerometer data
    RTC_DateTimeType timestamp; // Timestamp for the batch
} DataBatch;

DataBatch dataBatch;
int dataBatchIndex = 0; // Index for batch logic
int batchProcessingIndex = 0;//Proccessing index

extern volatile uint32_t msTicks; //msTicks is updated by SysTick_Handler
//extern volatile uint8_t rtcTickOccurred;

int main(void) {
    //System initialization
    SystemInit();
    SdkEvalIdentification();
    SdkEvalComUartInit(UART_BAUDRATE);

    //SPI initialization
    if (SdkEvalSpiDmaInit(4000000) != SUCCESS) {
        printf("Error initializing DMA communication.\r\n");
    }
    //Configure SysTick to generate interrupt with 1ms period
    SysTick_Config(SYST_CLOCK/1000);

    //Accelerometer initialization
    AdxlSetStandByModeDma();
    AdxlSetRangeModeDma();
    AdxlSetMeasurementModeDma();

    //RTC configuration for timestamping
    RTC_Configuration();
    RTC_Clockwatch_Configuration();

    uint32_t lastTick = 0;
    while (1) {
        // Sample every 40ms to get 25 samples per second
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
// End of clean code
*/
//int main(void)
//{
	/* System initialization function */
	//SystemInit();

	/* Identify BlueNRG1 platform */
	//SdkEvalIdentification();

	/* UART initialization */
	//SdkEvalComUartInit(UART_BAUDRATE);

	/* SPI initialization */
	//if (SdkEvalSpiDmaInit(4000000) != SUCCESS) {
	//        printf("Error initializing DMA communication.\r\n");
	 //   }

	/* Configure SysTick to generate interrupt with 25ms period */
	//SysTick_Config(SYST_CLOCK/1000);
	//uint8_t dataOut;
	//dataOut = AdxlReadRegisterDma(ADXL_DEVID_AD);

	    // Assuming 0xFF is an unlikely valid device ID and chosen as error indicator
	//    if (dataOut != 0xFF) {
	//        printf("Device ID: 0x%02X\r\n", dataOut);
	//    } else {
	//        printf("Failed to read device ID\r\n");
	//    }
	//while(1){
	//ADXL362_InitDma();
	//}
	//Device id test
	//while(1){
	//uint8_t deviceId;
	//if (AdxlReadDeviceIdDma(&deviceId) == SUCCESS) {
		// If the function returns SUCCESS, deviceId will now contain the read value
		//printf("ADXL362 Device ID: 0x%02X\r\n", deviceId);
	//} else {
		//printf("Failed to read ADXL362 Device ID\n");
	//}
	//}
	//device ID MDA test
	//uint8_t deviceIdBuffer[BUFFER_SIZE];
	//uint8_t bufferIndex = 0;

	//while (bufferIndex < BUFFER_SIZE) {
	//	if (AdxlReadDeviceIdDma(&deviceIdBuffer[bufferIndex]) == SUCCESS) {
	//		bufferIndex++;
	//	}
	//}
	//for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
	//	printf("Buffer[%d]: 0x%02X\r\n", i, deviceIdBuffer[i]);
	//}

	//Loopback mosi miso test dma
	//TestSpiDma();

	//setting accelerometer using dma

	//AdxlSetStandByModeDma();
	//AdxlSetRangeModeDma();
	//AdxlSetMeasurementModeDma();
	//AdxlAddBuffer(NULL,1);
	  /* Infinite loop */
	  // Simple polling of data ready flag to receive the accelerometer data
	 //while(1) {
	//			  status = AdxlReadRegisterDma(ADXL_STATUS);
	//			  printf("%i \r\n", status);
	//		      if(status & ADXL_DATA_READY)
	//		    	      {
	//			  	  	  imuR = AdxlReadAxesDma();
	//		    	      }else{
	//		    	    	  delay_ms(100);
			//    	      {
			    	    //	  printf("buffer enetered\r\n");
		    	  //    }
		//	    	     }
	//	}
	/* Configure RTC in timer mode to generate interrupt each 0.5s */
	//RTC_Configuration();

	/* Configure RTC in clockwatch mode to generate interrupt on time and date match */
	//RTC_Clockwatch_Configuration();

	// Assuming rtcTickOccurred is correctly set by an RTC interrupt handler
	//RTC_InitType RTC_InitStructure;
	//RTC_DateTimeType currentTime;
	//RTC_Configuration();

	// Set initial time
	//currentTime.Hour = 12;
	//currentTime.Minute = 30;
	//currentTime.Second = 45;
	//currentTime.WeekDay = 3;
	//currentTime.MonthDay = 10;
	//currentTime.Month = 2;
	//currentTime.Year = 2024;
	//RTC_SetTimeDate(&currentTime);
	//while(1){
	//	if (rtcTickOccurred) {
	//}
	        // Capture current time
	//        RTC_GetTimeDate(&currentTime);

	        // Save the captured time into the buffer
	       // if (timestampIndex < TIMESTAMP_BUFFER_SIZE) {
	        //    timestampBuffer[timestampIndex++] = currentTime;
	        //}

	        // Once the buffer is full, print all timestamps and reset the index
	        //if (timestampIndex == TIMESTAMP_BUFFER_SIZE) {
	         //   for (int i = 0; i < TIMESTAMP_BUFFER_SIZE; i++) {
	           //     printf("Buffered Timestamp: %02d:%02d:%02d\r\n",
	            //           timestampBuffer[i].Hour, timestampBuffer[i].Minute, timestampBuffer[i].Second);
	            //}
	            //timestampIndex = 0; // Reset for next batch
	        //}

	       // rtcTickOccurred = 0; // Reset the flag
	//}
//}
	//TimestampedDataBatch imuBuffer[BUFFER_SIZE];
	//int bufferIndex = 0; // Index for individual data points within a batch
	//int batchIndex = 0; // Index for batches of data
	//uint32_t lastTick = 0;
	//while (1) {
	 //       // Execute every ~33ms to get ~30 samples per second
	 //       if ((msTicks - lastTick) >= 33) {
	 //       	lastTick = msTicks; // Update lastTick with the latest timestamp

	//			uint8_t status = AdxlReadRegisterDma(ADXL_STATUS);
	//			if (status & ADXL_DATA_READY) {
		//			struct imu_type* imuR = AdxlReadAxesDma();
			//		if (imuR != NULL) {
				//		if (bufferIndex < BUFFER_SIZE) {
							// Copy data to buffer to prevent accidental modification
					//		imuBuffer[bufferIndex].xAxis = imuR->xAxis;
					//		imuBuffer[bufferIndex].yAxis = imuR->yAxis;
					//		imuBuffer[bufferIndex].zAxis = imuR->zAxis;
					//		bufferIndex++;

					//	}

						// Check if buffer is full and print values
					//	if (bufferIndex == BUFFER_SIZE) {
					//		RTC_GetTimeDate(&currentTime);
					//		printf("Timestamp: %02d:%02d:%02d\r\n", currentTime.Hour, currentTime.Minute, currentTime.Second);
					//		for (int i = 0; i < BUFFER_SIZE; i++) {
					//			printf("X-axis: %d, Y-axis: %d, Z-axis: %d\r\n",
					//				   imuBuffer[i].xAxis, imuBuffer[i].yAxis, imuBuffer[i].zAxis);
					//		}
					//		bufferIndex = 0; // Reset the buffer index for new readings
					//	}
					//}
				//}
			//}

	    // Additionally, check if there's buffered SPI data to process
	    // This could be relevant if your SPI data reception is not solely for the accelerometer
	    //if (buffer_spi_used > 0) {
	        // Process data from spi_buffer_rx starting from buffer_spi_tail
	      //  while (buffer_spi_used > 0) {
	            // Example processing: handle SPI data (e.g., parsing or further processing)
	            // This part depends on what the received SPI data represents beyond accelerometer readings

	            // Update buffer_spi_tail and buffer_spi_used as shown previously
	        //    buffer_spi_tail = (buffer_spi_tail + 1) % SPI_BUFF_SIZE;
	         //   __disable_irq();
	         //   buffer_spi_used--;
	          //  __enable_irq();
	        //}
	    //}
	//}

//}


/**
* @brief  Help message code
* @param  None
* @retval None
*/

/**
  * @brief  SPI Master initialization.
  * @param  None
  * @retval None
  */

//BLE TEST//
/* Includes ------------------------------------------------------------------*/
/*
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

#define BLE_NEW_SERIALPORT_VERSION_STRING "2.0.0"

#ifndef DEBUG
#define DEBUG 1
#endif

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


//void Process_InputData(uint8_t * rx_data, uint16_t data_size)
//{
//    // Process input data received from Bluetooth (if needed)
//}

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
*/

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

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
