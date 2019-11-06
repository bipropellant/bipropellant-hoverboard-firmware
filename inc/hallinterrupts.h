/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2018 Simon Hailes <btsimonh@googlemail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include "config.h"
#include "control_structures.h"
#include "defines.h"

typedef struct tag_time_stats {
    // times
    int64_t now_us;
    int64_t start_processing_us;
    int64_t processing_in_us;
    int64_t main_interval_us;
    int64_t time_in_us;
    int64_t main_delay_us;
    int64_t main_processing_us;

    int64_t nominal_delay_us;

    int32_t now_ms;
    int32_t start_processing_ms;
    int32_t processing_in_ms;
    int32_t main_interval_ms;
    int32_t time_in_ms;
    int32_t main_delay_ms;
    int32_t main_processing_ms;

    // stats
    int64_t us_lost;
    unsigned int main_late_count;

    float f_main_interval_ms;
    float f_main_processing_ms;

    int bldc_freq;
    int bldc_us;
    int bldc_cycles;
    int bldc_100k;

    int hclkFreq; // main CPU clock rate
} TIME_STATS;

extern TIME_STATS timeStats;

#define HALL_PIN_MASK (LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN | RIGHT_HALL_U_PIN | RIGHT_HALL_V_PIN | RIGHT_HALL_W_PIN)

//////////////////////////////////////////////////////////////
// change to change speed output value
#define HALL_SPEED_CALIBRATION 256000.0
// 10khz timer
#define HALL_INTERRUPT_TIMER_FREQ 100000

#define HALL_POSN_PER_REV 90
#define DEFAULT_WHEEL_SIZE_INCHES 6.5

//////////////////////////////////////////////////////////////
// this is the Hall data we gather, and can be read elsewhere
// one for each wheel
extern volatile HALL_DATA_STRUCT HallData[2];

extern volatile uint8_t hall_ul;
extern volatile uint8_t hall_vl;
extern volatile uint8_t hall_wl;

extern volatile uint8_t hall_ur;
extern volatile uint8_t hall_vr;
extern volatile uint8_t hall_wr;

////////////////////////////////////////////////////////////////////////////
// the one and only function we need to call to start it gathering Hall data
void HallInterruptinit(void);

// functions you can call
void HallInterruptSetWheelDiameterInches(float inches);
void HallInterruptSetWheelDiameterMM(float mm);
void HallInterruptReset();
int64_t HallGetuS(); // returns micro seconds (10 x the timer it is based on.....)

// interrupt routine
void HallInterruptsInterrupt(void);


////////////////////////////////////////////////////////////////////////////
// it may be useful to read the current position and zero it at the same time
// this function provides this as an option, as well as getting a single
// snapshot with interrupts disabled
#pragma pack(push, 4)  // int and int32_t are both 4 byte
typedef struct tag_HALL_POSN {
    struct {
        int HallPosn; // 90 per revolution
        int HallSpeed; // speed part calibrated to speed demand value
        int HallPosn_mm; // posn in m
        int HallSpeed_mm_per_s; // speed in m/s
        uint32_t HallSkipped;
    } wheel[2];
} HALL_POSN;
#pragma pack(pop)

void HallInterruptReadPosn( HALL_POSN *p, int Reset );

#pragma pack(push, 1)  // struct is a mix of many different types with varying size
typedef struct tag_HALL_PARAMS{
    uint8_t hall_u;
    uint8_t hall_v;
    uint8_t hall_w;

    uint8_t hall;
    uint8_t last_hall;

    int64_t hall_time;
    int64_t last_hall_time;
    unsigned int timerwraps;
    unsigned int last_timerwraps;

    int incr;

    int zerospeedtimeout;

    // contant - modifies sign of calculated values
    int direction;


    int dmacount;

    int hall_change_in_bldc_count;
} HALL_PARAMS;
#pragma pack(pop)

extern volatile HALL_DATA_STRUCT HallData[2];
extern TIM_HandleTypeDef h_timer_hall;
extern volatile HALL_PARAMS local_hall_params[2];
extern volatile int64_t timerwraps;
// extern volatile int64_t now_us;
