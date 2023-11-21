/*
 * BMI270.h
 *
 *  Created on: Sep 26, 2023
 *      Author: xange
 */

#ifndef INC_BMI270_H_
#define INC_BMI270_H_

#include "stm32f4xx_hal.h"

/*
 *
 * Registers
 *
 */
#define BMI270_CHIP_ID_VALUE 0x24
#define BMI270_REG_CHIP_ID 0x00
#define BMI270_REG_ERROR_REG 0x02
#define BMI270_REG_STATUS 0x03
#define BMI270_REG_ACC_X_LSB 0x0C
#define BMI270_REG_ACC_X_MSB 0x0D
#define BMI270_REG_ACC_Y_LSB 0x0E
#define BMI270_REG_ACC_Y_MSB 0x0F
#define BMI270_REG_ACC_Z_LSB 0x10
#define BMI270_REG_ACC_Z_MSB 0x11
#define BMI270_REG_GYR_X_LSB 0x12
#define BMI270_REG_GYR_X_MSB 0x13
#define BMI270_REG_GYR_Y_LSB 0x14
#define BMI270_REG_GYR_Y_MSB 0x15
#define BMI270_REG_GYR_Z_LSB 0x16
#define BMI270_REG_GYR_Z_MSB 0x17
#define BMI270_REG_INTERNAL_STATUS 0x21
#define BMI270_REG_TEMPERATURE_LSB 0x22
#define BMI270_REG_TEMPERATURE_MSB 0x23
#define BMI270_REG_FIFO_DATA 0x26
#define BMI270_REG_ACC_CONF 0x40
#define BMI270_REG_ACC_RANGE 0x41
#define BMI270_REG_GYR_CONF 0x42
#define BMI270_REG_GYR_RANGE 0x43
#define BMI270_REG_FIFO_DOWNSTREAM 0x45
#define BMI270_REG_FIFO_WATERMARK_LSB 0x46
#define BMI270_REG_FIFO_WATERMARK_MSB 0x47
#define BMI270_REG_FIFO_CONF_LSB 0x48
#define BMI270_REG_FIFO_CONF_MSB 0x49
#define BMI270_REG_SATURATIOM 0x4A
#define BMI270_REG_ERROR_REG_MASK 0x52
#define BMI270_REG_INT1_IO_CTRL 0x53
#define BMI270_REG_INT2_IO_CTRL 0x54
#define BMI270_REG_INT_LATCH 0x55
#define BMI270_REG_INT_MAP_DATA 0x58
#define BMI270_REG_INIT_CTRL 0x59
#define BMI270_REG_INIT_ADDR_LSB 0x5B
#define BMI270_REG_INIT_ADDR_MSB 0x5C
#define BMI270_REG_INIT_DATA 0x5E
#define BMI270_REG_INTERNAL_ERROR 0x5F
#define BMI270_REG_IF_CONFIG 0x6B
#define BMI270_REG_ACC_SELF_TEST 0x6D
#define BMI270_REG_GYR_SELF_TEST_AXES 0x6E
#define BMI270_REG_NV_CONF 0x70
#define BMI270_REG_ACC_OFFSET_X 0x71
#define BMI270_REG_ACC_OFFSET_Y 0x72
#define BMI270_REG_ACC_OFFSET_Z 0x73
#define BMI270_REG_GYR_USER_OFFSET_X 0x74
#define BMI270_REG_GYR_USER_OFFSET_Y 0x75
#define BMI270_REG_GYR_USER_OFFSET_Z 0x76
#define BMI270_REG_GYR_USER_OFFSET 0x77
#define BMI270_REG_PWR_CONF 0x7C
#define BMI270_REG_PWR_CTRL 0x7D
#define BMI270_REG_CMD 0x7E



/*
 *  SENSOR STRUCT
 */

typedef struct {

	// SPI Handle
	SPI_HandleTypeDef *SPI_Handle;
	GPIO_TypeDef *csPinBank;
	uint16_t 	csPin;


	//DMA
	uint8_t readingAccFlag;
	uint8_t readingGyrFlag;
	uint8_t accTxBuff;
	uint8_t gyrTxBuff;
	volatile uint8_t accRxBuff;
	volatile uint8_t gyrRxBuff;




	//Acceleration data (x,y,z) in m/s^2
	float acc_mps2[3];

	//Gyroscope data (x,y,z) in rad/s
	float gyr_rps[3];

	//Temperature data in Celcius
	float temp_C;

} BMI270;



	/*
	 * Initialisation
	 */
	uint8_t BMI270_Init(BMI270 *imu, SPI_HandleTypeDef *SPI_Handle, GPIO_TypeDef *csPinBank, uint16_t csPin);


	/*
	 * Simple read/write of registers
	 */
	uint8_t BMI270_ReadRegister(BMI270 *imu, uint8_t regAddr, uint8_t *data);
	uint8_t BMI270_WriteRegister(BMI270 *imu, uint8_t regAddr, uint8_t data);


	/*
	 * Reading accelerometer and gyroscope data
	 */
	uint8_t BMI270_ReadAccelerometer(BMI270 * imu);
	uint8_t BMI270_Read_GyrRegisters(BMI270 * imu);


	/*
	 * DMA read/write fuctions.
	 */
	uint8_t BMI270_ReadAcc_DMA(BMI270 *imu);
	uint8_t BMI270_ReadGYR_DMA(BMI270 *imu);

	void BMI270_Read_Acc_DMA_Complete(BMI270 *imu);
	void BMI_270_Read_Gyr_DMA_Complete( BMI270 *imu);






#endif /* INC_BMI270_H_ */
