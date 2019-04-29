#ifndef _DeadReckoner_h
#define _DeadReckoner_h



typedef struct INTEGER_XYT_POSN_tag {
    long x;
    long y;
    long degrees;
} INTEGER_XYT_POSN;

typedef struct DeadReckoner_tag {
	volatile long *leftTicks, *rightTicks; // Number of total wheel encoder tick counts for left and right wheels.
	long leftTicksPrev, rightTicksPrev; // Number of total wheel encoder tick counts at time computeAngularVelocities() is called.
	double xc, yc; // Latest position coordinates in ticks.
	double wl, wr; // Latest left and right angular velocity of the wheels in radians per second.
	double ticksPerRev; // Number of tick registers per second of the encoder.
	float w; // Angular velocity of the robot.
	double length; // Length from left wheel to right wheel.
	double radius; // Radius of the wheel.
	double theta;
	double toRadPerSec; // ticks/microsecond to rad/s conversion factor
	unsigned long prevIntegrationTime;
	unsigned long prevWheelComputeTime;
	int leftOmegaDirection;
	int rightOmegaDirection;
	unsigned long dt_omega;
	unsigned long dt_integration;
    int useDegrees; // indicate to pass theta in degrees
} DEADRECKONER;

// create an instance
DEADRECKONER *DeadReckoner(
    volatile long *pLeftTicks, 
    volatile long *pRightTicks, 
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

#endif