#include "SDK_EVAL_SPI.h"
#include "winbond_w25n01gwzeig_HAL.h"
#include "winbond_w25n01gwzeig.h"
#include <string.h>
#include <stdio.h>
#include "bluenrg_x_device.h"
#include "BlueNRG1_spi.h"


//#define SPI_BUFF_SIZE 	7

volatile uint8_t spi_buffer_tx_NAND[256];
volatile uint8_t spi_buffer_rx_NAND[256];


volatile uint8_t spi_buffer_tx[SPI_BUFF_SIZE];
volatile uint8_t spi_buffer_rx[SPI_BUFF_SIZE];
//NAND flash functions

//SPI DMA configuration for cs2 line
ErrorStatus SpiDmaInitNAND(uint32_t baudrate)
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

  /* Configure NAND flash CS pin (cs2) */
  GPIO_InitStructure.GPIO_Pin = s_SpiCs2PinVersion[SDK_PLATFORMS_NUMBER - 1]; // Use GPIO_Pin_11 as defined for cs2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
  GPIO_Init(&GPIO_InitStructure);

  // Initially set the NAND flash CS high (deselected)
  GPIO_SetBits(s_SpiCs2PinVersion[SDK_PLATFORMS_NUMBER - 1]);

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
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)spi_buffer_tx_NAND;
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
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)spi_buffer_rx_NAND;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = (uint32_t)SPI_BUFF_SIZE;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
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

//abstract cs low/high, since the NAND uses cs2 line
void HAL_CS_Low(void) {
	GPIO_ResetBits(s_SpiCs2PinVersion[SDK_PLATFORMS_NUMBER - 1]);
}

void HAL_CS_High(void) {
	GPIO_SetBits(s_SpiCs2PinVersion[SDK_PLATFORMS_NUMBER - 1]);
}

//send data
void HAL_SendData(uint8_t command, uint32_t address, uint8_t dummyBytes, uint8_t* buf, uint32_t len) {
	HAL_CS_Low();
    if (address != 0xFFFFFFFF) {
        HAL_SPI_Transmit(command, address, dummyBytes, buf, len); // Address is provided
    } else {
        HAL_SPI_Transmit(command, 0xFFFFFFFF, dummyBytes, buf, len); //No address needed
    }
    HAL_CS_High();
}

//Receive data
void HAL_ReceiveData(uint8_t command, uint8_t regAddr, uint8_t dummyBytes, uint8_t* buf, uint32_t len) {
    HAL_CS_Low();
    uint8_t dataWithRegAddr[1] = {regAddr};

    // Check if the operation requires sending a register address as part of the command
    if(command == W25N_READ_STATUS_REG || command == 0x0F) {
        HAL_SPI_Transmit(command, 0, dummyBytes, dataWithRegAddr, 1);
    } else {
        // For other commands not requiring additional parameters, just send the command
        HAL_SPI_Transmit(command, 0xFFFFFFFF, dummyBytes, NULL, 0);
    }

    // Receive the data after sending the command
    HAL_SPI_Receive(buf, len);
}

ErrorStatus HAL_SPI_Receive(uint8_t* buf, uint32_t len) {
    spi_eot = RESET;
    HAL_CS_Low();
    DMA_CH_SPI_RX->CNDTR = len;
    DMA_CH_SPI_TX->CNDTR = len;

    DMA_CH_SPI_TX->CCR_b.EN = SET;
    DMA_CH_SPI_RX->CCR_b.EN = SET;
    while (spi_eot == RESET);

    // Copy received data to the provided buffer
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = spi_buffer_rx_NAND[i+4];
    }

    return SUCCESS;
}

ErrorStatus HAL_SPI_Transmit(uint8_t command, uint32_t address, uint8_t dummyBytes, const uint8_t* buf, uint32_t len){
	SPI_SetMasterCommunicationMode(SPI_FULL_DUPLEX_MODE);
	uint8_t test[6];

	memset((void*)spi_buffer_tx_NAND, 0, sizeof(spi_buffer_tx_NAND));
	memset((void*)spi_buffer_rx_NAND, 0, sizeof(spi_buffer_rx_NAND));

    // Preparing the command and address
    uint32_t totalLen = 1;
    spi_buffer_tx_NAND[0] = command;

    if (address != 0xFFFFFFFF) { // 0xFFFFFFFF signifies no address needed
        spi_buffer_tx_NAND[1] = (address >> 16) & 0xFF;
        spi_buffer_tx_NAND[2] = (address >> 8) & 0xFF;
        spi_buffer_tx_NAND[3] = address & 0xFF;
        totalLen += 3; // Add 3 bytes for address
    }

    // Adding dummy bytes if required
    for (uint32_t i = 0; i < dummyBytes; i++) {
        spi_buffer_tx_NAND[totalLen + i] = 0x00;
    }
    totalLen += dummyBytes;

    // If there's data to send, append it to the transmission buffer
	if (buf != NULL && len > 0) {
		for (uint32_t i = 0; i < len; i++) {
			spi_buffer_tx_NAND[totalLen + i] = buf[i];
		}
		totalLen += len; // Adjust total length to include the data
	}

    // Transmitting the command (and address/dummy bytes)
    spi_eot = RESET;

    DMA_CH_SPI_TX->CNDTR = totalLen;
    DMA_CH_SPI_RX->CNDTR = totalLen;
    DMA_CH_SPI_RX->CCR_b.EN = SET;
    DMA_CH_SPI_TX->CCR_b.EN = SET;
    while (spi_eot == RESET);

    // Copy received data to the test buffer for troubleshooting
        for (uint32_t i = 0; i < len; i++) {
            test[i] = spi_buffer_rx_NAND[i];
        }

    return SUCCESS;
}

uint8_t HAL_SPI_ReadStatusRegister(void) {
	HAL_SendData(W25N_WRITE_ENABLE, 0, 0, NULL, 0);

	uint8_t statusRegValue[1];
	HAL_ReceiveData(W25N_READ_STATUS_REG, 0, 0, statusRegValue, 1);

	return -1;
}

ErrorStatus HAL_SPI_ReadManufacturerDeviceID(char idBuffer[][7], int index) {
    uint8_t length = 4;
    uint8_t id[3]; //3 relevant for the ID

    SPI_SetMasterCommunicationMode(SPI_FULL_DUPLEX_MODE);
    spi_buffer_tx_NAND[0] = 0x9F; // register address manufacture id

    for(uint8_t i = 1; i < length; i++){
        spi_buffer_tx_NAND[i] = 0xFF; //dummy bytes
    }

    while(spi_eot == RESET);
    spi_eot = RESET;

    HAL_CS_Low();

    DMA_CH_SPI_TX->CNDTR = length;
    DMA_CH_SPI_RX->CNDTR = length;

    DMA_CH_SPI_RX->CCR_b.EN = SET;
    DMA_CH_SPI_TX->CCR_b.EN = SET;

    while(spi_eot == RESET);

    HAL_CS_High();

    for(uint8_t i=0; i<3; i++){
    	id[i] = spi_buffer_rx_NAND[i+1];
    }

    // Format the ID
    snprintf(idBuffer[index], 7, "%02X%02X%02X", id[0], id[1], id[2]);

    return SUCCESS;
}

void HAL_delay_ms(uint32_t ms) {
    volatile uint32_t count;
    uint32_t delay = ms * 1;
    for (count = 0; count < delay; ++count) {
        __asm__ volatile ("nop");
    }
}
