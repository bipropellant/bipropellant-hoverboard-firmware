#pragma once

#include "control_structures.h"

#define BLDC_CONTROL_TYPE_ORIGINAL -1
#define BLDC_CONTROL_TYPE_COMMUTATION 0
#define BLDC_CONTROL_TYPE_TRAPEZOIDAL 1
#define BLDC_CONTROL_TYPE_SINUSOIDAL 2
#define BLDC_CONTROL_TYPE_SINUSOIDAL3RDHARMONIC 3

extern void readADCs();
extern void BldcController_Init();

typedef struct tag_BLDC_PARAMS{
  int ctrlTypSel; // first for a reason - so we can partial write to it.
  int phaAdvEna;
  int commDeacvHi;
  int commAcvLo;

  int callFrequency;

  volatile int initialized;

  int overruns;
} BLDC_PARAMS;

extern BLDC_PARAMS BldcControllerParams;


// structure to hold all the things we read.


extern volatile ELECTRICAL_PARAMS electrical_measurements;