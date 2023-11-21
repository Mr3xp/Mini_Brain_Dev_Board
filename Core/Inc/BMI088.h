/*
 * BMI088.h
 *
 *  Created on: Nov 14, 2023
 *      Author: xange
 */

#ifndef INC_BMI088_H_
#define INC_BMI088_H_

#include "stm32f4xx_hal.h"

/*
 * Sensor Registers Address
 */
#define BMI_ACC_CHIP_ID 		0x00
#define BMI_ACC_DATA 			0x12
#define BMI_TEMP_DATA 			0x22
#define BMI_ACC_CONF 			0x40
#define BMI_ACC_RANGE 			0x41
#define BMI_INT1_IO_CONF 	   	0x53
#define BMI_INT1_INT2_MAP_DATA 	0x58
#define BMI_ACC_PWR_CONF 		0x7C
#define BMI_ACC_PWR_CTRL 		0x7D
#define BMI_ACC_SOFTRESET 		0x7E

#define BMI_GYR_CHIP_ID			0x00
#define BMI_GYR_DATA			0x02
#define	BMI_GYR_RANGE			0x0F
#define	BMI_GYR_BANDWIDTH		0x10
#define	BMI_GYR_SOFTRESET		0x14
#define	BMI_GYR_INT_CTRL		0x15
#define	BMI_INT3_INT4_IO_CONF	0x16
#define BMI_INT3_INT4_IO_MAP	0x18



/*
 * Sensor Struct
 */
typedef struct {

	//SPI handle
		SPI_HandleTypeDef *SPI_Handle;
		GPIO_TypeDef *csAccPinBank;
		GPIO_TypeDef *csGyrPinBank;
		uint16_t csAccPin;
		uint16_t csGyrPin;

	//DMA
		uint8_t readingAccFlag;
		uint8_t readingGyrFlag;
		uint8_t accTxBuff;
		uint8_t gyrTxBuff;
		volatile uint8_t accRxBuff;
		volatile uint8_t gyrRxBuff;


		float accConversion;
		float gyrConversion;

		//Acceleration data (x,y,z) in m/s^2
		float acc_mps2[3];

		//Gyroscope data (x,y,z) in rad/s
		float gyr_rps[3];

		//Temperature data in Celcius
		float temp_C;

}BMI088;

uint8_t BMI088_Init(BMI088 *imu,
						SPI_HandleTypeDef *SPI_Handle,
						GPIO_TypeDef *GPIO_Acc_Pin_Bank, uint16_t csAccPin
						);

uint8_t BMI088_ReadAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data);
uint8_t BMI088_ReadGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data);

uint8_t BMI088_WriteAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t data);
uint8_t BMI088_WriteGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t data);

uint8_t BMI088_ReadAccelerometer(BMI088 *imu);


#endif /* INC_BMI088_H_ */
