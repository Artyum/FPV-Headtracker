/*
 *  FPV Headtracker
 *
 *  Created on: 2016
 *
 * 	ATmega328@16MHz
 * 	Sensors: LSM6DS33, LIS3MDL
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <math.h>

#include "IMU/imu.h"
#include "FILTER/filter.h"
#include "BTN/btn.h"
#include "CPU/cpu.h"
#include "FUNC/func.h"
#include "I2C/i2c.h"
#include "UART/uart.h"

double xx, px, xz, pz;

int main(void) {
	//Initialization
	cpu_init();
	uart_init(UBRR_250000);
	i2c_init();
	imu_init();

	//Read calibration data
	readConfig();
	//sendConfig();

	//Setting default PPM channels length
	initPPM();

	//Initial position reset
	t_resetpos = POS_RESET_TIME;

	//Main loop
    while (1) {
		//Short press - Reset central position
		key_check(&btn_status, &t_btn, BTN_PINPORT, BTN_PIN);

		//Position reset
		if (btn_status==KEY_UP_SHORT) t_resetpos = POS_RESET_TIME;

		//Get from UART
		if (rxrdy) {
			if (rxbuf[0]=='E') calibtool = CALIB_IMU;
			if (rxbuf[0]=='F') calibtool = CALIB_PPM;
			if (rxbuf[0]=='C') sendConfig();
			if (rxbuf[0]=='S') saveConfig();
			if (rxbuf[0]=='G') resetGyro();
			if (rxbuf[0]=='A') resetAccel();
			if (rxbuf[0]=='M') resetMagnet();
			if (rxbuf[0]=='P') resetPPM();
			if (rxbuf[0]=='c') getConfig((char*)rxbuf);

			LED_ON; _delay_ms(50); LED_OFF;
			rxrdy = 0;
		}

		//Loop time
		uint16_t ms = t_ms;
		if ((ms>=5) && imuRdyGA()) {

			double dt = (double)ms/1000.0; //Delta t in seconds
			t_ms = 0;

			if (getGyro() && getAccel()) {
				corGyro();
				corAccel();

				//Get new magnetometer readings
				if (imuRdyM()) {
					getMagnet();
					corMagnet();
				}

				if (calibtool==CALIB_IMU) {
					uart_putc('#');
					for (uint8_t i=0; i<3; i++) { uart_putl(gft[i],10); uart_putc(';'); }
					for (uint8_t i=0; i<3; i++) { uart_putl(aft[i],10); uart_putc(';'); }
					for (uint8_t i=0; i<3; i++) { uart_putl(mft[i],10); uart_putc(';'); }
					uart_nl();
				}

				normGyro();
				normAccel();

				accPitch = calcPitch();
				accRoll = calcRoll();
				magYaw = calcYaw(accPitch, accRoll);
				//magAngleZ = kalman(yawTiltCompensation(accAngleX,accAngleY),&xz,&pz);

				//uart_putl(accRoll*R2D, 10); uart_puts("\t"); uart_putl(accPitch*R2D, 10); uart_puts("\t"); uart_putl(magYaw*R2D, 10); uart_nl();

				//Reset pozycji
				uint8_t rst = 0;
				if (t_resetpos) {
					rst = 1;
					LED_ON;

					accStartX = accPitch;
					magStartZ = magYaw;
				}

				accPitch = calcAbsAngle(accPitch, accStartX);
				magYaw = calcAbsAngle(magYaw, magStartZ);

				//uart_putl(accRoll*R2D, 10); uart_puts("\t"); uart_putl(accPitch*R2D, 10); uart_puts("\t"); uart_putl(magYaw*R2D, 10); uart_nl();
				//uart_putl(-gft[0],10); uart_puts("\t"); uart_putl(accRoll*R2D,10); uart_puts("\t"); uart_putl(-gft[2],10); uart_puts("\t"); uart_putl(magYaw*R2D,10); uart_nl();
				//uart_putl(gft[0],10); uart_puts("\t"); uart_putl(accRoll*R2D,10); uart_puts("\t"); uart_putd(dt,5); uart_nl();

				//Filter
				mergeAngles(&gyroAngleX, gft[0], accPitch, dt, rst);
				mergeAngles(&gyroAngleZ, gft[2], magYaw, dt, rst);

				//uart_putl(gyroAngleX,10); uart_puts("\t"); uart_putl(gyroAngleZ,10); uart_nl();

				//PPM generation
				newLenX = calcLenPPM(gyroAngleX, ppm_cen_x, ppm_low_x, ppm_top_x, ppm_scale_x);
				newLenZ = calcLenPPM(gyroAngleZ, ppm_cen_z, ppm_low_z, ppm_top_z, ppm_scale_z);

				//newLenX = kalman(calcLenPPM(gyroAngleX, ppm_cen_x, ppm_low_x, ppm_top_x, ppm_scale_x), &xx, &px);
				//newLenZ = kalman(calcLenPPM(gyroAngleZ, ppm_cen_z, ppm_low_z, ppm_top_z, ppm_scale_z), &xz, &pz);

				//uart_putl(gyroAngleX,10); uart_puts("\t");
				//uart_putl(ppm_cen_x,10); uart_puts("\t");
				//uart_putl(ppm_low_x,10); uart_puts("\t");
				//uart_putl(ppm_top_x,10); uart_puts("\t");
				//uart_putl(ppm_scale_x,10);uart_puts("\t");
				//uart_putl(newLenX/2,10); uart_nl();
				//uart_putl(newLenX/2,10); uart_puts("\t"); uart_putl(newLenZ/2,10); uart_nl();

				if (calibtool==CALIB_PPM) {
					uart_putc('%');
					uart_putl(newLenX/2,10); uart_putc(';');
					uart_putl(newLenZ/2,10);
					uart_nl();
				}

				//Final set PPM channels
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
					ch_len[CH_ROLL] = newLenX;
					ch_len[CH_YAW] = newLenZ;
				}
			}
		}
		_delay_us(250);
	} //END while (1)

} //END main()

