
#include "BMI088.h"
/*
 * BMI088.c
 *
 *  Created on: Nov 14, 2023
 *      Author: xange
 */




uint8_t BMI088_Init(BMI088 *imu,
						SPI_HandleTypeDef *SPI_Handle,
						GPIO_TypeDef *GPIO_Acc_Pin_Bank, uint16_t csAccPin
						)  {

		/*
		 * Store interface parameters in struct
		 */
		imu-> SPI_Handle = SPI_Handle;
		imu-> csAccPinBank = GPIO_Acc_Pin_Bank;
		//imu-> csGyrPinBank = GPIO_Gyr_Pin_Bank;
		imu-> csAccPin = csAccPin;
		//imu-> csGyrPin = csGyrPin;


		uint8_t status = 0;

		/*
		 * Accelerometer requires rising edge on CS pin at start up to activate SPI.
		 */
		HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);
		HAL_Delay(50);


		/*
		 * Perform accelerometer soft reset
		 */
			BMI088_WriteAccRegister(imu, BMI_ACC_SOFTRESET, 0xB6);
		HAL_Delay(50);

		uint8_t chipID;
		status += BMI088_ReadAccRegister(imu, BMI_ACC_CHIP_ID, &chipID);

		if (chipID != 0x1E) {

		//	return 0;

		}
		HAL_Delay(10);

		/* Configure accelerometer  */
		status += BMI088_WriteAccRegister(imu, BMI_ACC_CONF, 0xA8); /* (no oversampling, ODR = 100 Hz, BW = 40 Hz) */
		HAL_Delay(10);

		status += BMI088_WriteAccRegister(imu, BMI_ACC_RANGE, 0x00); /* +- 3g range */
		HAL_Delay(10);



		/* Put accelerometer into active mode */
		status += BMI088_WriteAccRegister(imu, BMI_ACC_PWR_CONF, 0x00);
		HAL_Delay(10);

		/* Turn accelerometer on */
		status += BMI088_WriteAccRegister(imu, BMI_ACC_PWR_CTRL, 0x04);
		HAL_Delay(10);

		/* Pre-compute accelerometer conversion constant (raw to m/s^2) */
		imu->accConversion = 9.81f / 32768.0f * 2.0f * 1.5f; /* Datasheet page 27 */

		return status;
}


uint8_t BMI088_ReadAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data){

	uint8_t txBuf[3] = { (regAddr|0x80), 0x00, 0x00};
	uint8_t rxBuf[3];


	HAL_GPIO_WritePin(imu-> csAccPinBank, imu-> csAccPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->SPI_Handle, txBuf, rxBuf, 3, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu-> csAccPinBank, imu-> csAccPin, GPIO_PIN_SET);

	if(status==1){

		*data = rxBuf[2];

	}

	return status;
}

uint8_t BMI088_WriteAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_Transmit(imu->SPI_Handle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	while(HAL_SPI_GetState(imu->SPI_Handle) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);

	return status;

}

uint8_t BMI088_ReadAccelerometer(BMI088 *imu) {

	/* Read raw accelerometer data */
	uint8_t txBuf[8] = {(BMI_ACC_DATA | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Register addr, 1 byte dummy, 6 bytes data */
	uint8_t rxBuf[8];

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->SPI_Handle, txBuf, rxBuf, 8, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);

	/* Form signed 16-bit integers */
	int16_t accX = (int16_t) ((rxBuf[3] << 8) | rxBuf[2]);
	int16_t accY = (int16_t) ((rxBuf[5] << 8) | rxBuf[4]);
	int16_t accZ = (int16_t) ((rxBuf[7] << 8) | rxBuf[6]);

	/* Convert to m/s^2 */
	imu->acc_mps2[0] = imu->accConversion * accX;
	imu->acc_mps2[1] = imu->accConversion * accY;
	imu->acc_mps2[2] = imu->accConversion * accZ;

	return status;

}





