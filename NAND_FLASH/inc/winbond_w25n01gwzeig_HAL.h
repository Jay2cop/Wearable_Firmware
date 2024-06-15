#ifndef WINBOND_W25N01GWZEIG_HAL_H
#define WINBOND_W25N01GWZEIG_HAL_H

#include <stdint.h>
#include "bluenrg_x_device.h"
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"

ErrorStatus SpiDmaInitNAND(uint32_t baudrate);
void HAL_CS_Low(void);
void HAL_CS_High(void);

ErrorStatus HAL_SPI_Receive(uint8_t* buf, uint32_t len);
ErrorStatus HAL_SPI_Transmit(uint8_t command, uint32_t address, uint8_t dummyBytes, const uint8_t* buf, uint32_t len);
void HAL_SendData(uint8_t command, uint32_t address, uint8_t dummyBytes, uint8_t* buf, uint32_t len);
void HAL_ReceiveData(uint8_t command, uint8_t regAddr, uint8_t dummyBytes, uint8_t* buf, uint32_t len);
uint8_t HAL_SPI_ReadStatusRegister(void);
void HAL_delay_ms(uint32_t ms);
ErrorStatus HAL_SPI_ReadManufacturerDeviceID(char idBuffer[][7], int index);

#endif
