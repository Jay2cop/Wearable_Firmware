#include "SDK_EVAL_SPI.h"
#include "adxl362_HAL.h"
#include "adxl362.h"
#include <string.h>
#include <stdio.h>
#include "bluenrg_x_device.h"
#include "BlueNRG1_spi.h"

extern volatile uint8_t spi_buffer_tx[SPI_BUFF_SIZE];
extern volatile uint8_t spi_buffer_rx[SPI_BUFF_SIZE];

ErrorStatus SdkEvalSpiDmaInit(uint32_t baudrate)
{
  SPI_InitType SPI_InitStructure;
  GPIO_InitType GPIO_InitStructure;
  DMA_InitType DMA_InitStructure;
  NVIC_InitType NVIC_InitStructure;

  /* Enable SPI and GPIO clocks */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO | CLOCK_PERIPH_SPI | CLOCK_PERIPH_DMA, ENABLE);

  /* Configure SPI pins */
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = SDK_EVAL_SPI_PERIPH_OUT_PIN | SDK_EVAL_SPI_PERIPH_SCLK_PIN | SDK_EVAL_SPI_PERIPH_IN_PIN;
  GPIO_InitStructure.GPIO_Mode = Serial0_Mode;
  GPIO_InitStructure.GPIO_Pull = ENABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init(&GPIO_InitStructure);

  /* Configure CS pin */
  GPIO_InitStructure.GPIO_Pin = SDK_EVAL_SPI_PERIPH_SENSOR_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
  GPIO_Init(&GPIO_InitStructure);
  SDK_EVAL_SPI_CS_HIGH();

  /* Configure SPI in master mode */
  SPI_StructInit(&SPI_InitStructure);
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b ;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_BaudRate = baudrate;
  SPI_Init(&SPI_InitStructure);

  /* Set null character */
  SPI_SetDummyCharacter(0xFF);

  /* Clear RX and TX FIFO */
  SPI_ClearTXFIFO();
  SPI_ClearRXFIFO();

  /* Set communication mode */
  SPI_SetMasterCommunicationMode(SPI_FULL_DUPLEX_MODE);

  /* Configure DMA SPI TX channel */
  DMA_InitStructure.DMA_PeripheralBaseAddr = SPI_DR_BASE_ADDR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)spi_buffer_tx;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = (uint32_t)SPI_BUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA_CH_SPI_TX, &DMA_InitStructure);

  /* Configure DMA SPI RX channel */
  DMA_InitStructure.DMA_PeripheralBaseAddr = SPI_DR_BASE_ADDR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)spi_buffer_rx;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = (uint32_t)SPI_BUFF_SIZE;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_Init(DMA_CH_SPI_RX, &DMA_InitStructure);

  /* Enable DMA_CH_SPI_RX Transfer Complete interrupt */
  DMA_FlagConfig(DMA_CH_SPI_RX, DMA_FLAG_TC, ENABLE);

  /* Enable SPI_TX/SPI_RX DMA requests */
  SPI_DMACmd(SPI_DMAReq_Tx | SPI_DMAReq_Rx, ENABLE);

  /* Enable the DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = HIGH_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable SPI functionality */
  SPI_Cmd(ENABLE);

  return SUCCESS;
}

ErrorStatus TestSpiDma(void) {
    const char testMessage[] = "Hello DMA SPI";
    int messageLength = strlen(testMessage) + 1;

    if (messageLength > SPI_BUFF_SIZE) {
        return ERROR;
    }

    strncpy((char *)spi_buffer_tx, testMessage, messageLength);

    spi_eot = RESET;

   for (int i = 0; i < messageLength; i++) {
	   spi_buffer_rx[i] = 0;
   }

    SDK_EVAL_SPI_CS_LOW();

    DMA_CH_SPI_TX->CNDTR = messageLength;
    DMA_CH_SPI_RX->CNDTR = messageLength;
    DMA_CH_SPI_RX->CCR_b.EN = SET;
    DMA_CH_SPI_TX->CCR_b.EN = SET;

    while (spi_eot == RESET);

	for (int i = 0; i < messageLength; i++) {
		if (spi_buffer_tx[i] != spi_buffer_rx[i]) {
			printf("Mismatch at position %d: sent %02x, received %02x\n", i, spi_buffer_tx[i], spi_buffer_rx[i]);
			return ERROR;
		}
	}

    SDK_EVAL_SPI_CS_HIGH();
    printf("Loopback test successful, data matches\r\n");
    return SUCCESS;
}

uint8_t AdxlReadRegisterDma(uint8_t reg) {
    uint8_t receivedByte;
    if(SdkEvalSpiDmaRead(&receivedByte, reg, 1) == SUCCESS) {
        return receivedByte;
    }
    else {
        printf("Read Error\n");
        return 0;
    }
}

ErrorStatus AdxlReadDeviceIdDma(uint8_t* devId) {
    // Set up the command to read the device ID
    spi_buffer_tx[0] = ADXL_READ;
    spi_buffer_tx[1] = ADXL_DEVID_AD;
    spi_buffer_tx[2] = 0x00;

    spi_eot = RESET;

    SDK_EVAL_SPI_CS_LOW();

    DMA_CH_SPI_TX->CNDTR = 3;
    DMA_CH_SPI_RX->CNDTR = 3;
    DMA_CH_SPI_RX->CCR_b.EN = SET;
    DMA_CH_SPI_TX->CCR_b.EN = SET;

    while (spi_eot == RESET);

    SDK_EVAL_SPI_CS_HIGH();

    *devId = spi_buffer_rx[2];

    //printf("Device ID: 0x%02X\r\n", *devId);

    return SUCCESS;
}

void delay_ms(uint32_t ms) {
    volatile uint32_t count;
    uint32_t delay = ms * 1;
    for (count = 0; count < delay; ++count) {
        __asm__ volatile ("nop");
    }
}

