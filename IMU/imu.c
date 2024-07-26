#include "imu.h"

double EEMEM egct[3]   = { 0.0, 0.0, 0.0 };		//gct[3]
uint8_t EEMEM eginv[3] = { 1, 1, 1 };			//ginv[3]

double EEMEM eact[3]   = { 0.0, 0.0, 0.0 };		//act[3]
double EEMEM eacn[3]   = { 1.0, 1.0, 1.0 };		//acn[3]
uint8_t EEMEM eainv[3] = { 1, 1, 1 };			//ainv[3]

double EEMEM emct[3]   = { 0.0, 0.0, 0.0 };		//mct[3]
double EEMEM emcn[3]   = { 1.0, 1.0, 1.0 };		//mcn[3]
uint8_t EEMEM eminv[3] = { 1, 1, 1 };			//minv[3]

//PPM settings

//Initial channels length
uint16_t EEMEM eppm_cen_x = CH_LEN_CNT;
uint16_t EEMEM eppm_cen_z = CH_LEN_CNT;

uint16_t EEMEM eppm_low_x = CH_LEN_MIN;
uint16_t EEMEM eppm_low_z = CH_LEN_MIN;
uint16_t EEMEM eppm_top_x = CH_LEN_MAX;
uint16_t EEMEM eppm_top_z = CH_LEN_MAX;

double EEMEM eppm_scale_x = PPM_SCALE_X;
double EEMEM eppm_scale_z = PPM_SCALE_Z;

double EEMEM egyro_cutoff = GYRO_CUTOFF;

double EEMEM ekal_r = KALMAN_R;
double EEMEM ekal_q = KALMAN_Q;

double gft[3];		//Gyro
double aft[3];		//Acc
double mft[3];		//Magnet

double gct[3];  	//Gryro correction
double act[3];		//Accel correction
double acn[3];		//Accel bias
double mct[3];		//Magnet correction
double mcn[3];		//Magnet bias

double gyro_cutoff;

void imu_init() {
	uint8_t r=1;

	//LSM6DS33
	if (!i2c_write_byte(I2C_ADDR_GA,	 GA_CTRL1_XL,		0b01001000)) r=0;	//Linear acceleration ODR 104Hz | +-4g | Anti-Aliasing 400Hz
	if (!i2c_write_byte(I2C_ADDR_GA,	 GA_CTRL2_G,		0b01001100)) r=0;	//Angular rate sensor ODR 104Hz | 2000 DPS
	if (!i2c_write_byte(I2C_ADDR_GA,	 GA_CTRL3_C,		0b01000100)) r=0;	//Output registers not updated until MSB and LSB have been read | Autoincrement | LSB @ lower address
	if (!i2c_write_byte(I2C_ADDR_GA,	 GA_CTRL4_C,		0b10000000)) r=0;	//XL_BW_SCAL_ORD=1 | SLEEP_G=0
	if (!i2c_write_byte(I2C_ADDR_GA,	 GA_CTRL5_C,		0b00000000)) r=0;
	if (!i2c_write_byte(I2C_ADDR_GA,	 GA_CTRL6_C,		0b00010000)) r=0;	//XL_HM_MODE=1 high-performance disabled
	if (!i2c_write_byte(I2C_ADDR_GA,	 GA_CTRL7_C,		0b10000000)) r=0;	//G_HM_MODE=1 high-performance disabled
	if (!i2c_write_byte(I2C_ADDR_GA,	 GA_CTRL8_XL,		0b00000000)) r=0;
	if (!i2c_write_byte(I2C_ADDR_GA,	 GA_CTRL9_XL,		0b00111000)) r=0;	//Linear acceleration X Y Z enabled (default)
	if (!i2c_write_byte(I2C_ADDR_GA,	 GA_CTRL10_C,		0b00111000)) r=0;	//Gyroscope X Y Z enabled (default)

	//LIS3MDL
	if (!i2c_write_byte(I2C_ADDR_M,		M_CTRL_REG1,		0b01111100)) r=0;	//High-performance mode | 80Hz | FAST_ODR=0
	//if (!i2c_write_byte(I2C_ADDR_M,	M_CTRL_REG1,		0b10011100)) r=0;	//Temp enabled | Low-performance mode | 80Hz | FAST_ODR=0
	if (!i2c_write_byte(I2C_ADDR_M,		M_CTRL_REG2,		0b00000000)) r=0;	//+/-4 gauss
	//if (!i2c_write_byte(I2C_ADDR_M,	M_CTRL_REG3,		0b00000001)) r=0;	//Single-conversion mode with 80Hz
	if (!i2c_write_byte(I2C_ADDR_M,		M_CTRL_REG3,		0b00000000)) r=0;	//Continuous-conversion mode
	//if (!i2c_write_byte(I2C_ADDR_M,	M_CTRL_REG4,		0b00000000)) r=0;	//Z-axis Low-performance mode
	//if (!i2c_write_byte(I2C_ADDR_M, 	M_CTRL_REG4,		0b00001000)) r=0;	//Z-axis High-performance mode
	if (!i2c_write_byte(I2C_ADDR_M, 	M_CTRL_REG4,		0b00001100)) r=0;	//Z-axis Ultra High-performance mode
	if (!i2c_write_byte(I2C_ADDR_M,		M_CTRL_REG5,		0b01000000)) r=0;	//Output registers not updated until MSB and LSB have been read

	//Self-test
	if (!r || !imuTestGA() || !imuTestM()) {
		uart_puts("IMU TEST ERR\r\n");
		while (1) _delay_ms(100);
	}
}

//Odczyt zmiennych z EEPROM
void readConfig() {
	for (uint8_t i=0; i<3; i++) {
		gct[i] = (double)eeprom_read_float((float *)&egct[i]);
		act[i] = (double)eeprom_read_float((float *)&eact[i]);
		acn[i] = (double)eeprom_read_float((float *)&eacn[i]);
		mct[i] = (double)eeprom_read_float((float *)&emct[i]);
		mcn[i] = (double)eeprom_read_float((float *)&emcn[i]);
	}
	ppm_cen_x = (uint16_t)eeprom_read_word((uint16_t *)&eppm_cen_x);
	ppm_cen_z = (uint16_t)eeprom_read_word((uint16_t *)&eppm_cen_z);
	ppm_low_x = (uint16_t)eeprom_read_word((uint16_t *)&eppm_low_x);
	ppm_low_z = (uint16_t)eeprom_read_word((uint16_t *)&eppm_low_z);
	ppm_top_x = (uint16_t)eeprom_read_word((uint16_t *)&eppm_top_x);
	ppm_top_z = (uint16_t)eeprom_read_word((uint16_t *)&eppm_top_z);
	ppm_scale_x = (double)eeprom_read_float((float *)&eppm_scale_x);
	ppm_scale_z = (double)eeprom_read_float((float *)&eppm_scale_z);
	gyro_cutoff = (double)eeprom_read_float((float *)&egyro_cutoff);
	kal_r = (double)eeprom_read_float((float *)&ekal_r);
	kal_q = (double)eeprom_read_float((float *)&ekal_q);
}

void saveConfig() {
	for (uint8_t i=0; i<3; i++) {
		eeprom_busy_wait(); eeprom_update_float((float *)&egct[i], gct[i]);
		eeprom_busy_wait(); eeprom_update_float((float *)&eact[i], act[i]);
		eeprom_busy_wait(); eeprom_update_float((float *)&eacn[i], acn[i]);
		eeprom_busy_wait(); eeprom_update_float((float *)&emct[i], mct[i]);
		eeprom_busy_wait(); eeprom_update_float((float *)&emcn[i], mcn[i]);
	}
	eeprom_busy_wait(); eeprom_update_word((uint16_t *)&eppm_cen_x, ppm_cen_x);
	eeprom_busy_wait(); eeprom_update_word((uint16_t *)&eppm_cen_z, ppm_cen_z);
	eeprom_busy_wait(); eeprom_update_word((uint16_t *)&eppm_low_x, ppm_low_x);
	eeprom_busy_wait(); eeprom_update_word((uint16_t *)&eppm_low_z, ppm_low_z);
	eeprom_busy_wait(); eeprom_update_word((uint16_t *)&eppm_top_x, ppm_top_x);
	eeprom_busy_wait(); eeprom_update_word((uint16_t *)&eppm_top_z, ppm_top_z);
	eeprom_busy_wait(); eeprom_update_float((float *)&eppm_scale_x, ppm_scale_x);
	eeprom_busy_wait(); eeprom_update_float((float *)&eppm_scale_z, ppm_scale_z);
	eeprom_busy_wait(); eeprom_update_float((float *)&egyro_cutoff, gyro_cutoff);
	eeprom_busy_wait(); eeprom_update_float((float *)&ekal_r, kal_r);
	eeprom_busy_wait(); eeprom_update_float((float *)&ekal_q, kal_q);
	uart_puts("$Save OK\r\n");
}

void initPPM() {
	for (uint8_t i=0; i<CH_NUMBER; i++) ch_len[i] = CH_LEN_CNT;
	ch_len[CH_ROLL] = ppm_cen_x;
	ch_len[CH_YAW] = ppm_cen_z;
	newLenX = ppm_cen_x;
	newLenZ = ppm_cen_z;
}

void sendConfig() {
	uart_puts("@HT Config:"); uart_nl();
	uart_puts("@gcX="); uart_putd(gct[0],5); uart_nl();
	uart_puts("@gcY="); uart_putd(gct[1],5); uart_nl();
	uart_puts("@gcZ="); uart_putd(gct[2],5); uart_nl();

	uart_puts("@acX="); uart_putd(act[0],5); uart_nl();
	uart_puts("@acY="); uart_putd(act[1],5); uart_nl();
	uart_puts("@acZ="); uart_putd(act[2],5); uart_nl();
	uart_puts("@abX="); uart_putd(acn[0],5); uart_nl();
	uart_puts("@abY="); uart_putd(acn[1],5); uart_nl();
	uart_puts("@abZ="); uart_putd(acn[2],5); uart_nl();

	uart_puts("@mcX="); uart_putd(mct[0],5); uart_nl();
	uart_puts("@mcY="); uart_putd(mct[1],5); uart_nl();
	uart_puts("@mcZ="); uart_putd(mct[2],5); uart_nl();
	uart_puts("@mbX="); uart_putd(mcn[0],5); uart_nl();
	uart_puts("@mbY="); uart_putd(mcn[1],5); uart_nl();
	uart_puts("@mbZ="); uart_putd(mcn[2],5); uart_nl();

	uart_puts("@PPMlowX="); uart_putl(ppm_low_x/2,10); uart_nl();
	uart_puts("@PPMcenX="); uart_putl(ppm_cen_x/2,10); uart_nl();
	uart_puts("@PPMtopX="); uart_putl(ppm_top_x/2,10); uart_nl();

	uart_puts("@PPMlowZ="); uart_putl(ppm_low_z/2,10); uart_nl();
	uart_puts("@PPMcenZ="); uart_putl(ppm_cen_z/2,10); uart_nl();
	uart_puts("@PPMtopZ="); uart_putl(ppm_top_z/2,10); uart_nl();

	uart_puts("@ScaleX="); uart_putd(ppm_scale_x,2); uart_nl();
	uart_puts("@ScaleZ="); uart_putd(ppm_scale_z,2); uart_nl();

	uart_puts("@gCutOff="); uart_putd(gyro_cutoff,2); uart_nl();

	uart_puts("@R="); uart_putd(kal_r,2); uart_nl();
	uart_puts("@Q="); uart_putd(kal_q,2); uart_nl();
}

uint8_t imuRead(uint8_t sla, const uint8_t addr, double tab[]) {
	uint8_t data[6];
	if (i2c_read_sequence(sla, addr ,data, 6)) {
		tab[1] = (int16_t)((data[1]<<8) | data[0]);  //X -> Y
		tab[0] = (int16_t)((data[3]<<8) | data[2]);  //Y -> X
		tab[2] = (int16_t)((data[5]<<8) | data[4]);  //Z
		return 1;
	}
	else
		return 0;
}

uint8_t getGyro() {
	if (imuRead(I2C_ADDR_GA, GA_OUTX_L_G, gft)) return 1;
	else return 0;
}

uint8_t getAccel() {
	if (imuRead(I2C_ADDR_GA, GA_OUTX_L_XL, aft)) return 1;
	else return 0;
}

uint8_t getMagnet() {
	if (imuRead(I2C_ADDR_M, M_OUT_X_L, mft)) return 1;
	else return 0;
}

void corGyro() {
	static double xk[3];
	static double pk[3];

	for (uint8_t i=0; i<3; i++) {
		gft[i] = -gft[i] + gct[i];
		gft[i] = kalman(gft[i],&xk[i],&pk[i]);
	}
}

void normGyro() {
	for (uint8_t i=0; i<3; i++) {
		//RAW to DPS conversion
		gft[i] *= GYRO_RES;

		//Lowpass cut-off
		if (fabs(gft[i])<=gyro_cutoff) gft[i]=0.0;
	}
}

void corAccel() {
	static double xk[3];
	static double pk[3];

	for (uint8_t i=0; i<3; i++) {
		aft[i] = (aft[i] + act[i]) * acn[i];
		aft[i] = kalman(aft[i],&xk[i],&pk[i]);
	}
}

void normAccel() {
	double accmod = sqrt(aft[0]*aft[0] + aft[1]*aft[1] + aft[2]*aft[2]);
	for (uint8_t i=0; i<3; i++) aft[i] /= accmod;
}

void corMagnet() {
	static double xk[3];
	static double pk[3];

	for (uint8_t i=0; i<3; i++) {
		mft[i] = (mft[i] + mct[i]) * mcn[i];
		mft[i] = kalman(mft[i],&xk[i],&pk[i]);
	}
}

uint8_t imuRdyGA() {
	uint8_t reg;
	if (i2c_read_byte(I2C_ADDR_GA, GA_STATUS_REG, &reg)) {
		if ((reg & 3) == 3) return 1;
	}
	return 0;
}

uint8_t imuRdyM() {
	uint8_t reg;
	if (i2c_read_byte(I2C_ADDR_M, M_STATUS_REG, &reg)) {
		if ((reg & 15) == 15) return 1;
	}
	return 0;
}

uint8_t imuTestGA() {
	uint8_t reg;
	if (i2c_read_byte(I2C_ADDR_GA, GA_WHO_AM_I, &reg)) {
		if (reg == 0x69) return 1;
	}
	return 0;
}

uint8_t imuTestM() {
	uint8_t reg;
	if (i2c_read_byte(I2C_ADDR_M, M_WHO_AM_I, &reg)) {
		if (reg == 0x3d) return 1;
	}
	return 0;
}

void resetGyro() {
	for (uint8_t i=0; i<3; i++) {
		gct[i] = 0.0;
	}
	sendConfig();
}

void resetAccel() {
	for (uint8_t i=0; i<3; i++) {
		act[i] = 0.0;
		acn[i] = 1.0;
	}
	sendConfig();
}

void resetMagnet() {
	for (uint8_t i=0; i<3; i++) {
		mct[i] = 0.0;
		mcn[i] = 1.0;
	}
	sendConfig();
}

void resetPPM() {
	ppm_cen_x = CH_LEN_CNT;
	ppm_cen_z = CH_LEN_CNT;
	ppm_scale_x = PPM_SCALE_X;
	ppm_scale_z = PPM_SCALE_Z;
	gyro_cutoff = GYRO_CUTOFF;
	ppm_low_x = CH_LEN_MIN;
	ppm_low_z = CH_LEN_MIN;
	ppm_top_x = CH_LEN_MAX;
	ppm_top_z = CH_LEN_MAX;
	sendConfig();
}

void getConfig(char *buf) {
	//cg;gctx;gcty;gctz;ginvx;ginvy;ginvz
	//ca;actx;acty;actz;ainvx;ainvy;ainvz
	//cm;mctx;mcty;mctz;minvx;minvy;minvz

	char *txt;
	uint8_t i;
	uint16_t reti;
	double retd;

	//uart_puts(buf);
	//uart_nl();

	uint8_t d=0;
	if (buf[1]=='g') d=1;
	if (buf[1]=='a') d=2;
	if (buf[1]=='m') d=3;
	if (buf[1]=='p') d=4;

	if (d) {
		i=0;
		txt = strsep(&buf,";"); //skip first list item

		while ((txt=strsep(&buf,";"))) {
			retd = strtod(txt, NULL);

			if (d==1) {
				if (i<=2)			gct[i] = retd;
			}
			if (d==2) {
				if (i<=2)			act[i] = retd;
				if ((i>=3)&&(i<=5))	acn[i-3] = retd;
			}
			if (d==3) {
				if (i<=2)			mct[i] = retd;
				if ((i>=3)&&(i<=5))	mcn[i-3] = retd;
			}
			if (d==4) {
				reti = strtol(txt, NULL, 10);
				reti *= 2;	//us -> 0.5us

				if (i==0)	ppm_cen_x = reti;
				if (i==1)	ppm_cen_z = reti;
				if (i==2)	ppm_scale_x = retd;
				if (i==3)	ppm_scale_z = retd;
				if (i==4)	gyro_cutoff = retd;
				if (i==5)	ppm_low_x = reti;
				if (i==6)	ppm_low_z = reti;
				if (i==7)	ppm_top_x = reti;
				if (i==8)	ppm_top_z = reti;
				if (i==9)	kal_r = retd;
				if (i==10)	kal_q = retd;
			}
			//uart_putl(i,10); uart_putc(';');
			//uart_putl(reti,10); uart_putc(';');
			//uart_putl(retd*ACC,10); uart_nl();

			i++;
		}

		#if EXTENDED_PPM==0
		if (ppm_low_x<CH_LEN_MIN) ppm_low_x=CH_LEN_MIN;
		if (ppm_low_z<CH_LEN_MIN) ppm_low_z=CH_LEN_MIN;
		if (ppm_top_x>CH_LEN_MAX) ppm_top_x=CH_LEN_MAX;
		if (ppm_top_z>CH_LEN_MAX) ppm_top_z=CH_LEN_MAX;
		#endif
	}
	sendConfig();
}
