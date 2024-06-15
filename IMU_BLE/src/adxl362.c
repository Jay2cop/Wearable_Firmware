#include "adxl362_HAL.h"
#include "adxl362.h"
#include <stdio.h>
#include "BlueNRG1_spi.h"

volatile uint8_t spi_buffer_tx[SPI_BUFF_SIZE];
volatile uint8_t spi_buffer_rx[SPI_BUFF_SIZE];


ErrorStatus SdkEvalSpiDmaRead( uint8_t* pBuffer, uint8_t RegisterAddr, uint8_t NumByteToRead )
{
  SPI_SetMasterCommunicationMode(SPI_FULL_DUPLEX_MODE);

  spi_buffer_tx[0] = ADXL_READ;
  spi_buffer_tx[1] = RegisterAddr;
  for(uint8_t i=0;i<NumByteToRead;i++) {
    spi_buffer_tx[2+i] = 0xFF;
  }
  while(spi_eot==RESET);
  spi_eot = RESET;

  SDK_EVAL_SPI_CS_LOW();

  DMA_CH_SPI_TX->CNDTR = NumByteToRead+2;
  DMA_CH_SPI_RX->CNDTR = NumByteToRead+2;
  DMA_CH_SPI_RX->CCR_b.EN = SET;
  DMA_CH_SPI_TX->CCR_b.EN = SET;

  while(spi_eot==RESET);

  for(uint8_t i=0;i<NumByteToRead;i++) {
	  pBuffer[i] = spi_buffer_rx[2+i];
  }

  return SUCCESS;
}

ErrorStatus SdkEvalSpiDmaWrite( uint8_t* pBuffer, uint8_t RegisterAddr, uint8_t NumByteToWrite )
{
  SPI_SetMasterCommunicationMode(SPI_FULL_DUPLEX_MODE);

  spi_buffer_tx[0] = ADXL_WRITE;
  spi_buffer_tx[1] = RegisterAddr;
  for(uint8_t i=0;i<NumByteToWrite;i++) {
    spi_buffer_tx[2+i] = pBuffer[i];
  }
  while(spi_eot==RESET);
  spi_eot = RESET;

  SDK_EVAL_SPI_CS_LOW();

  DMA_CH_SPI_TX->CNDTR = NumByteToWrite+2;
  DMA_CH_SPI_RX->CNDTR = NumByteToWrite+2;

  DMA_CH_SPI_RX->CCR_b.EN = SET;
  DMA_CH_SPI_TX->CCR_b.EN = SET;

  while(spi_eot == RESET);

  SDK_EVAL_SPI_CS_HIGH();

  return SUCCESS;
}

void AdxlSetRangeModeDma()
{
	uint8_t filter_ctl_value = (ADXL362_RANGE_2G << 6) | ADXL362_ODR_25_HZ;
	SdkEvalSpiDmaWrite(&filter_ctl_value, ADXL362_REG_FILTER_CTL, 1);
	delay_ms(10);
}

void AdxlSetMeasurementModeDma()
{
	uint8_t MeasureCommand[] = {ADXL362_MEASURE_ON};
	SdkEvalSpiDmaWrite(MeasureCommand, ADXL362_REG_POWER_CTL, 1);
	delay_ms(10);
}

void AdxlSetStandByModeDma()
{
	uint8_t MeasureCommand[] = {ADXL362_MEASURE_STANDBY};
	SdkEvalSpiDmaWrite(MeasureCommand, ADXL362_REG_POWER_CTL, 1);
	delay_ms(10);
}

void AdxlSelfTestDma()
{
	uint8_t filterCtlCommand[] = {ADXL362_REG_POWER_CTL, ADXL362_RANGE_8G | ADXL362_ODR_100_HZ | ADXL_HALF_BW};
	SdkEvalSpiDmaWrite(filterCtlCommand, sizeof(filterCtlCommand)/sizeof(filterCtlCommand[0]), 1);
	delay_ms(10);

	uint8_t selfTestCommand[] = {ADXL_SELF_TEST_ON};
	SdkEvalSpiDmaWrite(selfTestCommand, ADXL_SELF_TEST, 1);
	delay_ms(10);

	uint8_t selfTestOffCommand[] = {ADXL_SELF_TEST_OFF};
	SdkEvalSpiDmaWrite(selfTestOffCommand, ADXL_SELF_TEST, 1);
}

struct imu_type *AdxlReadAxesDma() {
    static struct imu_type imu;
    uint8_t inBuf[6];

    // Clear the SPI buffers before starting the DMA transfer
    for (int i = 0; i < sizeof(spi_buffer_rx); i++) {
        spi_buffer_rx[i] = 0;
    }
    for (int i = 0; i < sizeof(spi_buffer_tx); i++) {
        spi_buffer_tx[i] = 0;
    }

    // Initiate the SPI DMA read operation to read the axes data
	if (SdkEvalSpiDmaRead(inBuf, 0x0E, sizeof(inBuf)) != SUCCESS) {
		printf("SPI DMA Read Failed\r\n");
		return NULL;
	}

	// Debugging: Print raw data for verification
	//printf("Raw Data: ");
	//for (int i = 0; i < sizeof(inBuf); i++) {
	//	printf("%02X ", inBuf[i]);
	//}
	//printf("\r\n");

	// Extract axis data
	imu.xAxis = (int16_t)((inBuf[1] << 8) | inBuf[0]);
	imu.yAxis = (int16_t)((inBuf[3] << 8) | inBuf[2]);
	imu.zAxis = (int16_t)((inBuf[5] << 8) | inBuf[4]);

	//printf("X-axis: %d, Y-axis: %d, Z-axis: %d\r\n", imu.xAxis, imu.yAxis, imu.zAxis);

	// Basic error handling
	if (imu.xAxis == 0 && imu.yAxis == 0 && imu.zAxis == 0) {
		printf("Warning: IMU data may not be updating correctly.\r\n");
	}

	return &imu;
}
