/*
 * BMI270.c
 *
 *  Created on: Sep 28, 2023
 *      Author: xange
 */

#include "BMI270.h"




uint8_t BMI270_Init(BMI270 *imu, SPI_HandleTypeDef *SPI_Handle, GPIO_TypeDef *csPinBank, uint16_t csPin){

	imu->SPI_Handle = SPI_Handle;
	imu->csPinBank = csPinBank;
	imu->csPin = csPin;


	//Clear DMA Flags.
	imu->readingAccFlag = 0;
	imu->readingGyrFlag = 0;

	uint8_t status = 0;

	//BMI270 requires rising edge on CSB at start-up SPI
	HAL_GPIO_WritePin(imu->csPinBank , imu->csPin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(imu->csPinBank , imu->csPin, GPIO_PIN_SET);
	HAL_Delay(50);

	//Perform a soft reset on the chip by writing the value oxb6.
	status += BMI270_WriteRegister(imu, BMI270_REG_CMD, 0xB6);
	HAL_Delay(100);

	//Check chip id.
	uint8_t chip_id_check;

	status +=BMI270_ReadRegister(imu, BMI270_REG_CHIP_ID, &chip_id_check);
	HAL_Delay(10);


	if( chip_id_check != BMI270_CHIP_ID_VALUE ){

		return 0;

	}
	/*BMI270_REG_PWR_CTRL
	 *Bit 0: Aux sensor enable.
	 *Bit 1: Gyroscope sensor enable.
	 *Bit 2: Accelerometer sensor enable.
	 *Bit 3: temperature sensor enable.
	 */
	status += BMI270_WriteRegister(imu, BMI270_REG_PWR_CTRL, 0x0e);
	HAL_Delay(10);
	/* Power mode configuration register. BMI270_REG_PWR_CONF
	*Bit 0: Advance power save disabled. 0x00 disable, 0x01 enable.
	*Bit 1: FIFO read disabled in low power mode. 0x00 disable, 0x01 enable.
	*Bit 2: Fast power up enable. 0x00 disable, 0x01 enabled.
	*/
	status += BMI270_WriteRegister(imu, BMI270_REG_PWR_CONF, 0x0);
	HAL_Delay(10);
	/* BMI270_REG_ACC_CONF default 0xA8.
	 *
	 * Bit 3-0: acc_odr(output data rate). default 100 hz
	 * Bit 6-4: acc_bwp (bandwidth parameters). default norm_avg4. acc_filt_perf=1 == norm acc_filt_perf=0 == avg4
	 * Bit 7: acc_filter_perf.
	 *
	 */
	status += BMI270_WriteRegister(imu, BMI270_REG_ACC_CONF, 0xA8);
	HAL_Delay(10);
	status += BMI270_WriteRegister(imu, BMI270_REG_GYR_CONF, 0xA9);
		HAL_Delay(10);
	/* BMI270_REG_ACC_RANGE
	 *
	 * Bit 1-0: acc_range.
	 */
	status += BMI270_WriteRegister(imu, BMI270_REG_ACC_RANGE, 0x01); //+-4g
	HAL_Delay(10);






	return status;
}

uint8_t BMI270_ReadRegister(BMI270 *imu, uint8_t regAddr, uint8_t *data){

		uint8_t txBuf[3] = {(regAddr | 0x80), 0x00, 0x00};
		uint8_t rxBuf[3];

		HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);
		uint8_t status = (HAL_SPI_TransmitReceive(imu->SPI_Handle, txBuf, rxBuf, 3, 200) == HAL_OK);
		HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);

		if (status == 1) {

			*data = rxBuf[2];

		}



		return status;
}




uint8_t BMI270_WriteRegister(BMI270 *imu, uint8_t regAddr, uint8_t data){

	uint8_t txBuff[2]= { regAddr, data };
	uint8_t rxBuff[2];

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->SPI_Handle, txBuff, rxBuff, 2, HAL_MAX_DELAY ) == HAL_OK);
	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);


	return status;

}


uint8_t BMI270_ReadAccelerometer(BMI270 *imu) {

	/* Read raw accelerometer data */
	uint8_t txBuf[8] = {(BMI270_REG_ACC_X_LSB | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Register addr, 1 byte dummy, 6 bytes data */
	uint8_t rxBuf[8];

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->SPI_Handle, txBuf, rxBuf, 8, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);

	/* Form signed 16-bit integers */
	int16_t accX = (int16_t) ((rxBuf[3] << 8) | rxBuf[2]);
	int16_t accY = (int16_t) ((rxBuf[5] << 8) | rxBuf[4]);
	int16_t accZ = (int16_t) ((rxBuf[7] << 8) | rxBuf[6]);

	/* Convert to m/s^2 */
	imu->acc_mps2[0] =  accX;
	imu->acc_mps2[1] =  accY;
	imu->acc_mps2[2] =  accZ;

	return status;

}















