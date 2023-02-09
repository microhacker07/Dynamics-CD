/*
 * BMP390.c
 *
 *  Created on: Feb 7, 2023
 *      Author: Nathaniel Bock
 */

#include "BMP390.h"

/*
 *
 * INITIALISATION
 *
 */
uint8_t BMP390_Init(BMP390 *baro,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csPinBank, uint16_t csPin) {

	/* Store interface parameters in struct */
	baro->spiHandle 	= spiHandle;
	baro->csPinBank 	= csPinBank;
	baro->csPin 		= csPin;

	/* Clear DMA flags */
	baro->reading = 0;

	uint8_t status = 0;

	/*
	 *
	 * ACCELEROMETER
	 *
	 */

	/* Perform accelerometer soft reset */

	/* Check chip ID */
	uint8_t chipID;
	status += BMP390_ReadRegister(baro, BMP_CHIP_ID, &chipID);

	if (chipID != BMP_DEFAULT_CHIP_ID) {

		return 1;

	}
//	HAL_Delay(10);
//
//	/* Configure accelerometer  */
//	status += BMP390_WriteAccRegister(baro, BMI_ACC_CONF, 0xA8); /* (no oversampling, ODR = 100 Hz, BW = 40 Hz) */
//	HAL_Delay(10);
//
//	status += BMP390_WriteAccRegister(baro, BMI_ACC_RANGE, 0x00); /* +- 3g range */
//	HAL_Delay(10);
//
//	/* Enable accelerometer data ready interrupt */
//	status += BMP390_WriteAccRegister(baro, BMI_INT1_IO_CONF, 0x0A); /* INT1 = push-pull output, active high */
//	HAL_Delay(10);
//
//	status += BMP390_WriteAccRegister(baro, BMI_INT1_INT2_MAP_DATA, 0x04);
//	HAL_Delay(10);
//
//	/* Put accelerometer into active mode */
//	status += BMP390_WriteAccRegister(baro, BMI_ACC_PWR_CONF, 0x00);
//	HAL_Delay(10);
//
//	/* Turn accelerometer on */
//	status += BMP390_WriteAccRegister(baro, BMI_ACC_PWR_CTRL, 0x04);
//	HAL_Delay(10);
//
//	/* Pre-compute accelerometer conversion constant (raw to m/s^2) */
//	baro->accConversion = 9.81f / 32768.0f * 2.0f * 1.5f; /* Datasheet page 27 */
//
//	/* Set accelerometer TX buffer for DMA */
//	baro->accTxBuf[0] = BMI_ACC_DATA | 0x80;
//
//	/*
//	 *
//	 * GYROSCOPE
//	 *
//	 */
//
//	HAL_GPIO_WritePin(baro->csGyrPinBank, baro->csGyrPin, GPIO_PIN_SET);
//
//	/* Perform gyro soft reset */
//	status += BMP390_WriteGyrRegister(baro, BMI_GYR_SOFTRESET, 0xB6);
//	HAL_Delay(250);
//
//	/* Check chip ID */
//	status += BMP390_ReadGyrRegister(baro, BMI_GYR_CHIP_ID, &chipID);
//
//	if (chipID != 0x0F) {
//
//		//return 0;
//
//	}
//	HAL_Delay(10);
//
//	/* Configure gyroscope */
//	status += BMP390_WriteGyrRegister(baro, BMI_GYR_RANGE, 0x01); /* +- 1000 deg/s */
//	HAL_Delay(10);
//
//	status += BMP390_WriteGyrRegister(baro, BMI_GYR_BANDWIDTH, 0x07); /* ODR = 100 Hz, Filter bandwidth = 32 Hz */
//	HAL_Delay(10);
//
//	/* Enable gyroscope data ready interrupt */
//	status += BMP390_WriteGyrRegister(baro, BMI_GYR_INT_CTRL, 0x80); /* New data interrupt enabled */
//	HAL_Delay(10);
//
//	status += BMP390_WriteGyrRegister(baro, BMI_INT3_INT4_IO_CONF, 0x01); /* INT3 = push-pull, active high */
//	HAL_Delay(10);
//
//	status += BMP390_WriteGyrRegister(baro, BMI_INT3_INT4_IO_MAP, 0x01); /* Data ready interrupt mapped to INT3 pin */
//	HAL_Delay(10);
//
//	/* Pre-compute gyroscope conversion constant (raw to rad/s) */
//	baro->gyrConversion = 0.01745329251f * 1000.0f / 32768.0f; /* Datasheet page 39 */
//
//	/* Set gyroscope TX buffer for DMA */
//	baro->gyrTxBuf[0] = BMI_GYR_DATA | 0x80;
//
	return status;

}

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */

uint8_t BMP390_ReadRegister(BMP390 *baro, uint8_t regAddr, uint8_t *data) {

	uint8_t txBuf[2] = {regAddr | 0x80, 0x00};
	uint8_t rxBuf[2];

	HAL_GPIO_WritePin(baro->csPinBank, baro->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(baro->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(baro->csPinBank, baro->csPin, GPIO_PIN_SET);

	if (status == 1) {

		*data = rxBuf[2];

	}

	return status;

}

uint8_t BMP390_WriteRegister(BMP390 *baro, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(baro->csPinBank, baro->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_Transmit(baro->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	while(HAL_SPI_GetState(baro->spiHandle) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(baro->csPinBank, baro->csPin, GPIO_PIN_SET);

	return status;
}



/*
 *
 * POLLING
 *
 */
uint8_t BMP390_Read(BMP390 *baro) {

	/* Read raw barometer data */
	uint8_t txBuf[7] = {(BMP_PRESSURE | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Register addr, 6 bytes data */
	uint8_t rxBuf[7];

	HAL_GPIO_WritePin(baro->csPinBank, baro->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(baro->spiHandle, txBuf, rxBuf, 8, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(baro->csPinBank, baro->csPin, GPIO_PIN_SET);

	/* Form signed 16-bit integers */
//	int32_t press_raw = (int32_t) ((rxBuf[3] << 8) | rxBuf[2]);
//	int16_t accY = (int16_t) ((rxBuf[5] << 8) | rxBuf[4]);
//	int16_t accZ = (int16_t) ((rxBuf[7] << 8) | rxBuf[6]);
//
//	/* Convert to m/s^2 */
//	baro->acc_mps2[0] = baro->accConversion * accX;
//	baro->acc_mps2[1] = baro->accConversion * accY;
//	baro->acc_mps2[2] = baro->accConversion * accZ;

	return status;

}

/*
 *
 * DMA
 *
 */
uint8_t BMP390_ReadDMA(BMP390 *baro) {

	HAL_GPIO_WritePin(baro->csPinBank, baro->csPin, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive_DMA(baro->spiHandle, baro->TxBuf, (uint8_t *) baro->RxBuf, 7) == HAL_OK) {

		baro->reading = 1;
		return 1;

	} else {

		HAL_GPIO_WritePin(baro->csPinBank, baro->csPin, GPIO_PIN_SET);
		return 0;

	}

}

void BMP390_ReadDMA_Complete(BMP390 *baro) {

	HAL_GPIO_WritePin(baro->csPinBank, baro->csPin, GPIO_PIN_SET);
	baro->reading = 0;

//	/* Form signed 16-bit integers */
//	int16_t accX = (int16_t) ((baro->accRxBuf[3] << 8) | baro->accRxBuf[2]);
//	int16_t accY = (int16_t) ((baro->accRxBuf[5] << 8) | baro->accRxBuf[4]);
//	int16_t accZ = (int16_t) ((baro->accRxBuf[7] << 8) | baro->accRxBuf[6]);
//
//	/* Convert to m/s^2 */
//	baro->acc_mps2[0] = baro->accConversion * accX;
//	baro->acc_mps2[1] = baro->accConversion * accY;
//	baro->acc_mps2[2] = baro->accConversion * accZ;

}
