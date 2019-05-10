
#include "deadreckoner.h"
#include <math.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"


#define UNSIGNED_LONG_MAX 4294967295
#define WHEEL_FORWARD 1
#define WHEEL_REVERSE -1

#define PI (3.14159265359)
#define TWO_PI (3.14159265359*2)

// if we had microseconds, set TIMER_RES to 1000000
#define micros HAL_GetTick
#define TIMER_RES (1000.0)

static void computeAngularVelocities(DEADRECKONER *);
static unsigned long getChange(unsigned long, unsigned long);


DEADRECKONER *DeadReckoner(volatile long *pLeftTicks, volatile long *pRightTicks, double ticksPerRev, double wheelDiameter, double wheelBase, int useDegrees) {
    // get zeroed structure
    DEADRECKONER *d = calloc(1, sizeof(DEADRECKONER));

	d->leftOmegaDirection = 1;
	d->rightOmegaDirection = 1;

	d->leftTicks = pLeftTicks;
	d->rightTicks = pRightTicks;
	d->ticksPerRev = ticksPerRev;
	d->radius = wheelDiameter/2;
	d->length = wheelBase;
	d->toRadPerSec = TIMER_RES * TWO_PI / d->ticksPerRev;

    d->useDegrees = useDegrees;

    return d;
}

void DeadReckoner_Destroy( DEADRECKONER **d ) {
    if (d) {
        if (*d) {
            free(*d);
            *d = NULL;
        }
    }
}

void setXYT(DEADRECKONER *d, double x, double y, double t) {
	d->xc = x;
	d->yc = y;
    if (d->useDegrees){
	    d->theta = (t/180.0)*PI;
    } else {
	    d->theta = t;
    }
}

void getXYT(DEADRECKONER *d, double *x, double *y, double *t) {
	if (x) *x = d->xc;
	if (y) *y = d->yc;
	if (t) {
        if (d->useDegrees){
            *t = (d->theta/PI)*180.0;
        } else {
            *t = d->theta;
        }
    }
}


void getWl(DEADRECKONER *d, double *wl, double *wr) {
	// TODO: Check this
	if (wl) *wl = d->wl * d->dt_omega / (UNSIGNED_LONG_MAX - d->dt_omega);
	if (wr) *wr = d->wr * d->dt_omega / (UNSIGNED_LONG_MAX - d->dt_omega);
}


void getW(DEADRECKONER *d, double *w) {
	if (w) *w = d->w * d->dt_integration / (UNSIGNED_LONG_MAX - d->dt_integration);
}


void setLROmegaDirection(DEADRECKONER *d, int directionL, int directionR) {
	d->leftOmegaDirection = directionL;
	d->rightOmegaDirection = directionR;
}

void getLROmegaDirection(DEADRECKONER *d, int *directionL, int *directionR) {
	if(directionL) *directionL = d->leftOmegaDirection;
	if(directionR) *directionR = d->rightOmegaDirection;
}


void computeAngularVelocities(DEADRECKONER *d) {
	// Time elapsed after computing the angular velocity previously.
	// change in time is defined as previous - current to prevent round off error.
	d->dt_omega = getChange(d->prevWheelComputeTime, micros()); // in milliseconds

	double changeLeftTicks = *d->leftTicks - d->leftTicksPrev;//getChange(*d->leftTicks, d->leftTicksPrev);
	double changeRightTicks = *d->rightTicks - d->rightTicksPrev;//getChange(*d->rightTicks, d->rightTicksPrev);

	d->wl = d->leftOmegaDirection * changeLeftTicks / d->dt_omega * d->toRadPerSec;
	d->wr = d->rightOmegaDirection * changeRightTicks / d->dt_omega * d->toRadPerSec;

	//double toRPM = 30.0 / PI;
	//Serial.print("\twl: "); Serial.print(wl*toRPM, 5);
	//Serial.print("\twr: "); Serial.print(wr*toRPM, 5);
	//Serial.print("\tdt: "); Serial.print(dt_omega);
	//Serial.print("\tlt: "); Serial.print(changeLeftTicks);
	//Serial.print("\trt: "); Serial.println(changeRightTicks);

	d->leftTicksPrev = *d->leftTicks;
	d->rightTicksPrev = *d->rightTicks;

	d->prevWheelComputeTime = micros();
}

unsigned long getChange(unsigned long current, unsigned long previous) {
	// Overflow has occured
	if (current < previous) {
		return UNSIGNED_LONG_MAX - previous + current;
	}
	// No overflow
	return current - previous;
}

void computePosition(DEADRECKONER *d) {
	d->dt_integration = getChange(d->prevIntegrationTime, micros());
    // don't do anything unless some time has passed.
    if (0 == d->dt_integration){
        return;
    }

	computeAngularVelocities(d);
	// Time elapsed after the previous position has been integrated.
	// change in time is defined as previous - current to prevent round off error.

	float dt = ((float)d->dt_integration) / TIMER_RES; // convert to seconds

	// Dead reckoning equations

	float Vl = d->wl * d->radius;
	float Vr = d->wr * d->radius;
	float v = (Vr + Vl) / 2.0;
	d->w = (Vr - Vl) / d->length;
	// Uses 4th order Runge-Kutta to integrate numerically to find position.
	float xNext = d->xc + dt * v*(2 + cos(dt*d->w / 2))*cos(d->theta + dt * d->w / 2) / 3;
	float yNext = d->yc + dt * v*(2 + cos(dt*d->w / 2))*sin(d->theta + dt * d->w / 2) / 3;
	float thetaNext = d->theta + dt * d->w;

	d->xc = xNext;
	d->yc = yNext;
	d->theta = thetaNext;

	// float toRPM = 30 / PI;
	// float dist = sqrt(d->xc*d->xc + d->yc * d->yc);
	//Serial.print("\tdist: "); Serial.print(dist);
	//Serial.print("\twl: "); Serial.print(wl*toRPM, 5);
	//Serial.print("\twr: "); Serial.print(wr*toRPM, 5);
	//Serial.print("\tVl: "); Serial.print(Vl);
	//Serial.print("\tVr: "); Serial.print(Vr);
	//Serial.print("\tw: "); Serial.print(w, 5);
	//Serial.print("\tx: "); Serial.print(xc);
	//Serial.print("\ty: "); Serial.print(yc);
	//Serial.print("\ttheta: "); Serial.println(theta*RAD_TO_DEG);

	d->prevIntegrationTime = micros();
}

void reset(DEADRECKONER *d, int resetTheta) {
	d->xc = 0;
	d->yc = 0;
	d->leftTicksPrev = *d->leftTicks;
	d->rightTicksPrev = *d->rightTicks;
    if (resetTheta) d->theta = 0;
}