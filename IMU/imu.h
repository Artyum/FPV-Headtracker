#ifndef GYRO_H_
#define GYRO_H_

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include <string.h>

#include "../I2C/i2c.h"
#include "../UART/uart.h"
#include "../FUNC/func.h"
#include "../FILTER/filter.h"


//Read count for calibration
#define GYRO_ADJUST_CNT		500

#define GYRO_RES_DPS		2000.0

//Gyro Max DPS
#define GYRO_RES 			(GYRO_RES_DPS/32767.0)

//LSM6DS33 I2C Address
#define I2C_ADDR_GA			0x6B

//LSM6DS33 registers
#define GA_FIFO_CTRL1		0x06
#define GA_FIFO_CTRL2		0x07
#define GA_FIFO_CTRL3		0x08
#define GA_FIFO_CTRL4		0x09
#define GA_FIFO_CTRL5		0x0A
#define GA_ORIENT_CFG_G		0x0B
#define GA_INT1_CTRL		0x0D
#define GA_INT2_CTRL		0x0E
#define GA_WHO_AM_I			0x0F	/*ret 0x69*/
#define GA_CTRL1_XL			0x10
#define GA_CTRL2_G			0x11
#define GA_CTRL3_C			0x12
#define GA_CTRL4_C			0x13
#define GA_CTRL5_C			0x14
#define GA_CTRL6_C			0x15
#define GA_CTRL7_C			0x16
#define GA_CTRL8_XL			0x17
#define GA_CTRL9_XL			0x18
#define GA_CTRL10_C			0x19
#define GA_STATUS_REG		0x1E
#define GA_OUTX_L_G			0x22
#define GA_OUTX_L_XL		0x28

//LIS3MDL I2C Address
#define I2C_ADDR_M			0x1E

//LIS3MDL registers
#define M_WHO_AM_I			0x0f
#define M_CTRL_REG1			0x20
#define M_CTRL_REG2			0x21
#define M_CTRL_REG3			0x22
#define M_CTRL_REG4			0x23
#define M_CTRL_REG5			0x24
#define M_STATUS_REG		0x27
#define M_OUT_X_L			0x28

extern double gft[3];
extern double aft[3];
extern double mft[3];

//IMU initialization
void imu_init();

//IMU read 3 values of X,Y,Z axis
uint8_t imuRead(uint8_t sla, const uint8_t addr, double tab[]);

uint8_t getGyro();
uint8_t getAccel();
uint8_t getMagnet();

void corGyro();
void corAccel();
void corMagnet();

void normGyro();
void normAccel();

uint8_t imuRdyGA();
uint8_t imuRdyM();

uint8_t imuTestGA();
uint8_t imuTestM();

void resetGyro();
void resetAccel();
void resetMagnet();
void resetPPM();

//Load calibration values
void readConfig();
void saveConfig();
void sendConfig();
void initPPM();
void getConfig(char *buf);

#endif /* GYRO_H_ */
