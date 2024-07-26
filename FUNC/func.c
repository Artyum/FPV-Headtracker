#include "func.h"

volatile uint16_t ch_len[CH_NUMBER];	//PPM channels length

double accPitch, accRoll, magYaw;
double accStartX, magStartZ;
double gyroAngleX, gyroAngleZ;
int16_t newLenX, newLenZ;

//Initial channels length
uint16_t ppm_cen_x;
uint16_t ppm_cen_z;

//Sensitivity
double ppm_scale_x;
double ppm_scale_z;

uint16_t ppm_low_x;
uint16_t ppm_low_z;
uint16_t ppm_top_x;
uint16_t ppm_top_z;

//Enable calibration tool
uint8_t calibtool;

//Korekty Gyro
double gct[3];
double EEMEM gctmem[3] = { 0.0, 0.0, 0.0 };

void calibGyro(double gct[]) {
	uint16_t gcnt = 0; //Read counter
	double sum[3] = {0.0, 0.0, 0.0}; //Sum
	double raw[3];
	uint8_t i;

	while (gcnt<GYRO_ADJUST_CNT) {
		//Odczekanie na nowy pomiar z Gyro
		_delay_ms(5);

		if (imuRead(I2C_ADDR_GA, GA_OUTX_L_G, raw)) {
			for (i=0; i<3; i++) sum[i] += raw[i];
			gcnt++;
		}
	}

	//Calculate corrections for X, Y and Z
	for (i=0; i<3; i++) {
		gct[i] = -(double)sum[i]/(double)gcnt;

		//Save values in EPROM
		eeprom_busy_wait();
		eeprom_update_float((float *)&gctmem[i], gct[i]);
	}
}

double calcPitch() {
	return atan2(aft[1],aft[2]);
}

double calcRoll() {
	return asin(-aft[0]);
}

double calcYaw(double pitch, double roll) {
	//Calculate tilt-compensated yaw direction
	double sinp = sin(pitch);
	double cosp = cos(pitch);
	double sinr = sin(roll);
	double cosr = cos(roll);

	double y = mft[0]*cosr + mft[2]*sinr;
	double x = mft[0]*sinp*sinr + mft[1]*cosp - mft[2]*sinp*cosr;

	return atan2(y,x);
}

double calcAbsAngle(double angle, double start_angle) {
	double s = angle-start_angle;
	if (s>M_PI) s=s-M_2PI;
	else if (s<-M_PI) s=s+M_2PI;
	return s;
}

uint16_t calcLenPPM(double gyroAngle, uint16_t center, uint16_t low, uint16_t top, double scale) {
	//New PPM channel length
	//int len = (double)center + (gyroAngle*scale/SERVO_MAX_ANLGE)*(double)CH_LEN_CHG;
	int len = (double)center + gyroAngle*scale;
	if (len<0) len=0;

	//Check margins
	if (len < low) return low;
	if (len > top) return top;

	#if EXTENDED_PPM==0
	if (len < CH_LEN_MIN) return CH_LEN_MIN;
	if (len > CH_LEN_MAX) return CH_LEN_MAX;
	#endif

	return len;
}
