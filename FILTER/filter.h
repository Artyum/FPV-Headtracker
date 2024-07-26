#ifndef FILTER_H_
#define FILTER_H_

#include <math.h>
#include <avr/io.h>

#include "../FUNC/func.h"

//Complementary filter factors
#define WSP_LOW		0.96
#define WSP_MAX		1.0
#define GFT_MAX		25.0
#define WSP_RST		0.7

//Kalman filter factors
#define KALMAN_R	1.0
#define KALMAN_Q	0.5

extern double kal_r;
extern double kal_q;

void mergeAngles(double *gyroAngle, double gyro, double accAngle, double dt, uint8_t rst);
double kalman(double zk, double *xk, double *Pk);

#endif /* FILTER_H_ */
