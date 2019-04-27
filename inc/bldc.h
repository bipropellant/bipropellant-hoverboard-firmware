#ifndef BLDC_H
#define BLDC_H



// structure to hold all the things we read.
typedef struct tag_ELECTRICAL_PARAMS{
    int bat_raw;
    float batteryVoltage;

    int board_temp_raw;
    float board_temp_filtered;
    float board_temp_deg_c;

    int charging;

    float dcCurLim;

    struct {
        float dcAmps;
        float dcAmpsAvgAcc;
        float dcAmpsAvg;
        int r1;
        int r2;
        int q;
    } motors[2];

} ELECTRICAL_PARAMS;

extern volatile ELECTRICAL_PARAMS electrical_measurements;
#endif
