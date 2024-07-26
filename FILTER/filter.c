#include "filter.h"

double kal_r;
double kal_q;

void mergeAngles(double *gyroAngle, double gyro, double accAngle, double dt, uint8_t rst) {
	double wsp;

	if (rst) {
		wsp = WSP_RST;
	}
	else {
		wsp = WSP_MAX - (WSP_MAX-WSP_LOW)*fabs(gyro)/GFT_MAX;
		//if (wsp < WSP_LOW) wsp = WSP_LOW;
		//uart_putl(wsp*100,10); uart_nl();
	}
	*gyroAngle = wsp*(*gyroAngle+gyro*dt) + (1.0-wsp)*(accAngle*R2D);
}

double kalman(double zk, double *xk, double *Pk) {
	//Phase prediction
	//*Xk = Xk_1;
	*Pk = *Pk + kal_q;

	//Phase correction
	double Kk = *Pk / (*Pk + kal_r);
	*xk = *xk + Kk*(zk - *xk);
	*Pk = (1 - Kk)*(*Pk);

	//The new estimate
	return *xk;
}
