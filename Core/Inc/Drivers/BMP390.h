#ifndef BMI088_IMU_H
#define BMI088_IMU_H

#include "stm32g4xx_hal.h"

/* Register defines
 * Starts on page 33 of BMP 390 data sheet
 * */
#define BMP_CHIP_ID 			0x00
#define BMP_REV_ID				0x01
#define BMP_ERR_REG				0x02
#define BMP_STATUS				0x03

#define BMP_PRESSURE_0			0x04
#define BMP_PRESSURE_1			0x05
#define BMP_PRESSURE_2			0x06

#define BMP_TEMPERATURE_0		0x07
#define BMP_TEMPERATURE_1		0x08
#define BMP_TEMPERATURE_2		0x09

#define BMP_SENSORTIME_0			0x0C
#define BMP_SENSORTIME_1			0x0D
#define BMP_SENSORTIME_2			0x0E

#define BMP_EVENT				0x10
#define BMP_INT_STATUS			0x11
#define BMP_FIFO_LENGTH_0		0x12
#define BMP_FIFO_LENGTH_1		0x13
#define BMP_FIFO_DATA			0x14
#define BMP_FIFO_WTM_0	0x15
#define BMP_FIFO_WTM_1	0x16

#define BMP_FIFO_CONFIG_1		0x17
#define BMP_FIFO_CONFIG_2		0x18
#define BMP_INT_CTRL			0x19
#define BMP_IF_CONF				0x1A
#define BMP_PWR_CTRL			0x1B
#define BMP_OSR					0x1C
#define BMP_ODR					0x1D
#define BMP_CONFIG				0x1F
#define BMP_CMD					0x7E


typedef struct {

	/* SPI */
	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef 	  *csPinBank;
	uint16_t 		   cspin;

	/* DMA */
	uint8_t reading;
	uint8_t TxBuf[8];
	volatile uint8_t RxBuf[8];

	/* Conversion constants (raw to m/s^2 and raw to rad/s) */
	float pressConversion;

	/* Pressure, Temperature, and Altitude*/
	float press;
	float temp;
	float altitude;

} BMP390;

/*
 *
 * INITIALISATION
 *
 */
uint8_t BMP390_Init(BMP390 *baro,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csPinBank, uint16_t csPin);

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */
uint8_t BMP390_ReadRegister(BMP390 *baro, uint8_t regAddr, uint8_t *data);
uint8_t BMP390_WriteRegister(BMP390 *baro, uint8_t regAddr, uint8_t data);

/*
 *
 * POLLING
 *
 */
uint8_t BMP390_Read(BMI088 *imu);

/*
 *
 * DMA
 *
 */
uint8_t BMP390_ReadDMA(BMP390 *baro);
void 	BMP390_ReadDMA_Complete(BMP390 *baro);

#endif
