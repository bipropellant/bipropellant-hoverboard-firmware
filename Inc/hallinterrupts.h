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
#ifndef HALLINTERRUPTS_H
#define HALLINTERRUPTS_H

#include "config.h"

#ifdef HALL_INTERRUPTS

#include "defines.h"

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
typedef struct tag_HALL_DATA_STRUCT{
    int HallPosn; // 90 per revolution
    float HallSpeed; // speed part calibrated to speed demand value

    float HallPosnMultiplier; // m per hall segment

    float HallPosn_m; // posn in m
    float HallSpeed_m_per_s; // speed in m/s

    unsigned int HallTimeDiff;
    unsigned long HallSkipped;
} HALL_DATA_STRUCT;
extern volatile HALL_DATA_STRUCT HallData[2];


////////////////////////////////////////////////////////////////////////////
// the one and only function we need to call to start it gathering Hall data
void HallInterruptinit(void);

// functions you can call
void HallInterruptSetWheelDiameterInches(float inches);
void HallInterruptSetWheelDiameterMM(float mm);
void HallInterruptReset();



////////////////////////////////////////////////////////////////////////////
// it may be useful to read the current position and zero it at the same time
// this function provides this as an option, as well as getting a single 
// snapshot with interrupts disabled
typedef struct tag_HALL_POSN {
    struct {
        int HallPosn; // 90 per revolution
        float HallSpeed; // speed part calibrated to speed demand value
        float HallPosn_m; // posn in m
        float HallSpeed_m_per_s; // speed in m/s
        unsigned long HallSkipped;
    } wheel[2];
} HALL_POSN;

void HallInterruptReadPosn( HALL_POSN *p, int Reset );

#endif

#endif