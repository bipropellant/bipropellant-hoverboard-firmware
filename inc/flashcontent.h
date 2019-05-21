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

// this file defines the structure to store variables in the flash
// e.g. PID constants
// such that they can be configured through protocol.c
// and saved to flash

// a copy is kept in ram.
// a pointer to each variable alnog with methods for modification is kept in a structure in protocol

// to write to it, use protocol to write to magic, this will commit content to flash

// decimal to make it easier to type!
#define CURRENT_MAGIC 1234

#pragma pack(push, 2)
typedef struct tag_FLASH_CONTENT{
    unsigned short magic;  // write this with CURRENT_MAGIC to commit to flash

    unsigned short PositionKpx100; // pid params for Position
    unsigned short PositionKix100;
    unsigned short PositionKdx100;
    unsigned short PositionPWMLimit; // e.g. 200

    unsigned short SpeedKpx100; // pid params for Speed
    unsigned short SpeedKix100;
    unsigned short SpeedKdx100;
    unsigned short SpeedPWMIncrementLimit; // e.g. 20

    unsigned short MaxCurrLim;

    unsigned short HoverboardEnable; // non zero to enable

} FLASH_CONTENT;
#pragma pack(pop)

FLASH_CONTENT FlashContent;

const FLASH_CONTENT FlashDefaults;

#ifndef FLASH_DEFAULT_HOVERBOARD_ENABLE
#define FLASH_DEFAULT_HOVERBOARD_ENABLE 0
#endif

#define FLASH_DEFAULTS { CURRENT_MAGIC,    50, 50, 0, 200,    20, 10, 0, 10,    1500,   FLASH_DEFAULT_HOVERBOARD_ENABLE }
