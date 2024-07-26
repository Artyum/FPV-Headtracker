#ifndef FUNC_H_
#define FUNC_H_

#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <math.h>

#include "../CPU/cpu.h"
#include "../FILTER/filter.h"
#include "../IMU/imu.h"
#include "../UART/uart.h"

//Allow for extended PPM channels length
#define EXTENDED_PPM		1

//PPM time in us
#if 0
//Timer1 1us
#define PPM_LEN				22500
#define PPM_DELAY			300
#define CH_LEN_CNT			1200
#define CH_LEN_CHG			512
#else
//Timer1 0.5us
#define PPM_LEN				45000
#define PPM_DELAY			600
#define CH_LEN_CNT			2400
#define CH_LEN_CHG			1024
#endif

#define CH_LEN_MIN			(CH_LEN_CNT-CH_LEN_CHG)
#define CH_LEN_MAX			(CH_LEN_CNT+CH_LEN_CHG)

#define CALC_CH_X			0
#define CALC_CH_Z			1

//PPM scale
#define PPM_SCALE_X			3.5
#define PPM_SCALE_Z			3.5

//PPM channels number
#define CH_NUMBER			8

//Output channels numbers for pitch and yaw (-1)
#define CH_ROLL				6
#define CH_YAW				7

//Minimalna wykrywalna prêdkoœæ obrotu DPS
#define GYRO_CUTOFF			4.0

//Others
#define M_2PI				(2*M_PI)
#define R2D					(180.0/M_PI)
#define CALIB_IMU			1
#define CALIB_PPM			2

#define LOW_WSP				0.95
#define LOW_NWSP			0.05

//Reset position time
#define POS_RESET_TIME		300

#define ACC					100000

extern volatile uint16_t ch_len[CH_NUMBER];
extern double accPitch, accRoll, magYaw;
extern double accStartX, magStartZ;
extern double gyroAngleX, gyroAngleZ;
extern int16_t newLenX, newLenZ;
extern uint8_t calibtool;
extern uint16_t ppm_cen_x;
extern uint16_t ppm_cen_z;
extern double ppm_scale_x;
extern double ppm_scale_z;
extern uint16_t ppm_low_x;
extern uint16_t ppm_low_z;
extern uint16_t ppm_top_x;
extern uint16_t ppm_top_z;

//Calculate PPM length in ms
uint16_t calcLenPPM(double gyroAngle, uint16_t center_len, uint16_t low, uint16_t top, double scale);

//Calculate angle relative to zero-position (-PI -> 0 -> +PI)
double calcAbsAngle(double angle, double start_angle);

double calcPitch();
double calcRoll();
double calcYaw(double roll, double pitch);

#endif /* FUNC_H_ */
