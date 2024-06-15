#ifndef ADXL362_HAL_H
#define ADXL362_HAL_H

#include <stdint.h>
#include "bluenrg_x_device.h"
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADXL362_IRQ_PIN                 SDK_EVAL_IRQ_SENSOR_PIN
#define ADXL_SPI_FREQUENCY              (4000000)

// Macro to configure IRQ pin
#define adxl362_IO_ITConfig()                                               \
    {SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);                 \
     GPIO_Init(&(GPIO_InitType){ADXL362_IRQ_PIN, GPIO_Input, DISABLE, DISABLE}); \
     NVIC_Init(&(NVIC_InitType){GPIO_IRQn, LOW_PRIORITY, ENABLE});          \
     GPIO_EXTIConfig(&(GPIO_EXTIConfigType){ADXL362_IRQ_PIN, GPIO_IrqSense_Edge, IRQ_ON_FALLING_EDGE}); \
     GPIO_ClearITPendingBit(ADXL362_IRQ_PIN);                               \
     GPIO_EXTICmd(ADXL362_IRQ_PIN, ENABLE);}

ErrorStatus SdkEvalSpiDmaInit(uint32_t baudrate);
ErrorStatus TestSpiDma(void);
ErrorStatus AdxlReadDeviceIdDma(uint8_t* devId);
uint8_t AdxlReadRegisterDma(uint8_t reg);
void delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif // ADXL362_HAL_H
