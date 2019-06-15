#pragma once

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
#pragma pack(push, 4) // all used types (float and int) are 4 bytes

typedef struct tag_MOTOR_ELECTRICAL{
        float dcAmps;
        float dcAmpsAvgAcc;
        float dcAmpsAvg;
        int r1;
        int r2;
        int q;

        int dcAmpsx100;

        int pwm_limiter;
        int pwm_requested;
        int pwm_actual;

        unsigned int limiter_count;
} MOTOR_ELECTRICAL;

typedef struct tag_ELECTRICAL_PARAMS{
    int bat_raw;
    float batteryVoltage;

    int board_temp_raw;
    float board_temp_filtered;
    float board_temp_deg_c;

    int charging;

    int dcCurLim; // amps*100
    int dc_adc_limit; // limit expressed in terms of ADC units.

    MOTOR_ELECTRICAL motors[2];

} ELECTRICAL_PARAMS;
#pragma pack(pop)

extern volatile ELECTRICAL_PARAMS electrical_measurements;