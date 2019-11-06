#pragma once
#include <stdint.h>

typedef struct DeadReckoner_tag {
	volatile int32_t *leftTicks, *rightTicks; // Number of total wheel encoder tick counts for left and right wheels.
	int32_t leftTicksPrev, rightTicksPrev; // Number of total wheel encoder tick counts at time computeAngularVelocities() is called.
	double xc, yc; // Latest position coordinates in ticks.
	double wl, wr; // Latest left and right angular velocity of the wheels in radians per second.
	double ticksPerRev; // Number of tick registers per second of the encoder.
	float w; // Angular velocity of the robot.
	double length; // Length from left wheel to right wheel.
	double radius; // Radius of the wheel.
	double theta;
	double toRadPerSec; // ticks/microsecond to rad/s conversion factor
	uint32_t prevIntegrationTime;
	uint32_t prevWheelComputeTime;
	int leftOmegaDirection;
	int rightOmegaDirection;
	uint32_t dt_omega;
	uint32_t dt_integration;
    int useDegrees; // indicate to pass theta in degrees
} DEADRECKONER;

// create an instance
DEADRECKONER *DeadReckoner(
    volatile int32_t *pLeftTicks,
    volatile int32_t *pRightTicks,
    double ticksPerRev,
    double wheelDiameter,
    double wheelBase,
    int useDegrees);
// delete an instance
void DeadReckoner_Destroy( DEADRECKONER **d );
// call this regularly.
void computePosition(DEADRECKONER *d);
// * read postion
void getXYT(DEADRECKONER *d, double *x, double *y, double *t);

// reset position
void setXYT(DEADRECKONER *d, double x, double y, double t);
// but this is easier if you just want all zeros.
void reset(DEADRECKONER *d, int resetTheta);

// read angular velocity of wheels
void getWl(DEADRECKONER *d, double *wl, double *wr);

void getW(DEADRECKONER *d, double *w);


// we don't use these, since hall ticks go up and down
void setLROmegaDirection(DEADRECKONER *d, int directionL, int directionR);
void getLROmegaDirection(DEADRECKONER *d, int *directionL, int *directionR);

// the global one
extern DEADRECKONER *deadreconer;
