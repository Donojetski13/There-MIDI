#include "stdint.h"

#ifndef VS1053_H_
#define VS1053_H_

#include "stm32l4xx_hal.h"

/* Pin configuration */
extern SPI_HandleTypeDef 			hspi2;
#define HSPI_VS1053					&hspi2
#define GPIO_PIN_SET				((uint8_t) 1U)
#define GPIO_PIN_RESET				((uint8_t) 0U)

#define VS1053_DREQ_PORT			GPIOG
#define VS1053_DREQ_PIN				GPIO_PIN_0 // last SPI status

#define	VS1053_XRST_PORT			GPIOG
#define	VS1053_XRST_PIN				GPIO_PIN_1 // PB0 reset

#define VS1053_XCS_PORT				GPIOB
#define VS1053_XCS_PIN				GPIO_PIN_2 // command start
//#define VS1053_XDCS_PORT			GPIOD
//#define VS1053_XDCS_PIN			GPIO_PIN_3

/* Commands */
#define VS1053_WRITE_CMD	0x02
#define VS1053_READ_CMD		0x03

/* Functions */
uint8_t VS1053_Init();
void VS1053_Reset();
uint8_t VS1053_SoftReset();
uint8_t VS1053_SetVolume(uint8_t volumeLeft, uint8_t volumeRight);
uint8_t VS1053_SetMode(uint16_t mode);
uint8_t VS1053_GetMode(uint16_t *mode);
uint8_t VS1053_AutoResync();
uint8_t VS1053_SetDecodeTime(uint16_t time);
//uint8_t VS1053_SendEndFill(uint16_t num);
uint8_t VS1053_IsBusy();
uint8_t VS1053_SciWrite(uint8_t address, uint16_t input);
uint8_t VS1053_SciRead(uint8_t address, uint16_t *res);
//uint8_t VS1053_SdiWrite(uint8_t input);
//uint8_t VS1053_SdiWrite32(uint8_t *input32);

extern uint8_t endFillByte;

#endif
