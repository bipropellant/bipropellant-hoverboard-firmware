#pragma once
#include "stm32f1xx_hal.h"


//////////////////////////////////////////////////////////
// macro types for the hoverboard control style
// add to add other control combinations
#define HOVERBOARD_WITH_SOFTWARE_SERIAL_B2_C9 1
#define USART2_CONTROLLED 2
#define USART3_CONTROLLED 3
#define SOFTWARE_SERIAL_A2_A3 4
#define HOVERBOARD_WITH_SOFTWARE_SERIAL_B2_C9_6WORDSENSOR 5
#define HOVERBOARD 6

// thoery says this is the only thing you need to change....
// Can also be preset from platformio.ini when using platform.io build environment
#ifndef CONTROL_TYPE
  #define CONTROL_TYPE HOVERBOARD_WITH_SOFTWARE_SERIAL_B2_C9
#endif
//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
// implementaiton of specific for macro control types
// provide a short explaination here
#if (CONTROL_TYPE == HOVERBOARD)
  // this control type allows the board to be used AS a hoverboard,
  // responding to sensor movements when in hoverboard mode.
  /// and uses softwareserial for serial control on B2/C9
  #define INCLUDE_PROTOCOL INCLUDE_PROTOCOL2
  #define READ_SENSOR
  #define CONTROL_SENSOR
  #define SOFTWARE_SERIAL
  #define SOFTWATCHDOG_TIMEOUT -1   // Disable Watchdog, uses the same timer as SOFTWARE_SERIAL
  #define DEBUG_SOFTWARE_SERIAL
  #define SERIAL_USART2_IT
  #define SERIAL_USART3_IT
  #define USART2_BAUD     52350    // reported baudrate for other sensor boards (6 word)?
  #define USART3_BAUD     52350    // reported baudrate for other sensor boards (6 word)?
  #define USART2_WORDLENGTH UART_WORDLENGTH_9B
  #define USART3_WORDLENGTH UART_WORDLENGTH_9B
  #define SERIAL_USART_IT_BUFFERTYPE unsigned short
  #define USART2_BAUD_SENSE 1
  #define USART3_BAUD_SENSE 1
#endif

#if (CONTROL_TYPE == HOVERBOARD_WITH_SOFTWARE_SERIAL_B2_C9)
  // this control type allows the board to be used AS a hoverboard,
  // responding to sensor movements when in hoverboard mode.
  /// and uses softwareserial for serial control on B2/C9
  #define INCLUDE_PROTOCOL INCLUDE_PROTOCOL2
  #define READ_SENSOR
  #define CONTROL_SENSOR
  #define SOFTWARE_SERIAL
  #define SOFTWARE_SERIAL_RX_PIN GPIO_PIN_2
  #define SOFTWARE_SERIAL_RX_PORT GPIOB
  #define SOFTWARE_SERIAL_TX_PIN GPIO_PIN_9
  #define SOFTWARE_SERIAL_TX_PORT GPIOC
  //#define DEBUG_SERIAL_ASCII
  #define DEBUG_SOFTWARE_SERIAL
//    #define USART2_BAUD     52177    // control via usart from GD32 based sensor boards @52177 baud (10 word)
//    #define USART3_BAUD     52177    // control via usart from GD32 based sensor boards @52177 baud (10 word)
  #define SENSOR_WORDS 10
  #define SERIAL_USART2_IT
  #define SERIAL_USART3_IT
  #define USART2_BAUD     26315    // reported baudrate for other sensor boards (6 word)?
  #define USART3_BAUD     26315    // reported baudrate for other sensor boards (6 word)?
  #define USART2_WORDLENGTH UART_WORDLENGTH_9B
  #define USART3_WORDLENGTH UART_WORDLENGTH_9B
  #define SERIAL_USART_IT_BUFFERTYPE unsigned short
//#define USART2_BAUD     32100    // reported baudrate for another sensor board  (10 word 'Denver' brand hoverboards)
//#define USART3_BAUD     32100    // reported baudrate for another sensor board  (10 word 'Denver' brand hoverboards)
  // possibly baud rate based on ~2.5ms frame interval, so baud dependent on word count?
#endif

#if (CONTROL_TYPE == HOVERBOARD_WITH_SOFTWARE_SERIAL_B2_C9_6WORDSENSOR)
  // this control type allows the board to be used AS a hoverboard,
  // responding to sensor movements when in hoverboard mode.
  /// and uses softwareserial for serial control on B2/C9
  #define INCLUDE_PROTOCOL INCLUDE_PROTOCOL2
  #define READ_SENSOR
  #define CONTROL_SENSOR
  #define SOFTWARE_SERIAL
  #define SOFTWARE_SERIAL_RX_PIN GPIO_PIN_2
  #define SOFTWARE_SERIAL_RX_PORT GPIOB
  #define SOFTWARE_SERIAL_TX_PIN GPIO_PIN_9
  #define SOFTWARE_SERIAL_TX_PORT GPIOC
  //#define DEBUG_SERIAL_ASCII
  #define DEBUG_SOFTWARE_SERIAL
//#define USART2_BAUD     52177    // control via usart from GD32 based sensor boards @52177 baud (10 word)
//#define USART3_BAUD     52177    // control via usart from GD32 based sensor boards @52177 baud (10 word)
  #define USART2_BAUD     26315    // reported baudrate for other sensor boards (6 word)?
  #define USART3_BAUD     26315    // reported baudrate for other sensor boards (6 word)?
  #define USART2_WORDLENGTH UART_WORDLENGTH_9B
  #define USART3_WORDLENGTH UART_WORDLENGTH_9B
  #define SERIAL_USART_IT_BUFFERTYPE unsigned short
  #define SENSOR_WORDS 6
  #define SERIAL_USART2_IT
  #define SERIAL_USART3_IT
//#define USART2_BAUD     32100    // reported baudrate for another sensor board (maybe 7 word)?
//#define USART3_BAUD     32100    // reported baudrate for another sensor board (maybe 7 word)?
  // possibly baud rate based on ~2.5ms frame interval, so baud dependent on word count?
#endif


#if (CONTROL_TYPE == SOFTWARE_SERIAL_A2_A3)
  // hoverboard sensor functionality is disabled
  // and uses softwareserial for serial control on A2/A3 -
  // which are actually USART pins!
  #define INCLUDE_PROTOCOL INCLUDE_PROTOCOL2
  #define SOFTWARE_SERIAL
  #define SOFTWARE_SERIAL_RX_PIN GPIO_PIN_2    // PB10/USART3_TX Pin29      PA2/USART2_TX/ADC123_IN2  Pin16
  #define SOFTWARE_SERIAL_RX_PORT GPIOA
  #define SOFTWARE_SERIAL_TX_PIN GPIO_PIN_3    // PB11/USART3_RX Pin30      PA3/USART2_RX/ADC123_IN3  Pin17
  #define SOFTWARE_SERIAL_TX_PORT GPIOA
  //#define DEBUG_SERIAL_ASCII
#endif


// implementaiton of specific for macro control types
#if (CONTROL_TYPE == USART2_CONTROLLED)
  // hoverboard sensor functionality is disabled
  // and control is via USART2
  #define INCLUDE_PROTOCOL INCLUDE_PROTOCOL2
  #define SERIAL_USART2_IT
  #define PASE_ADV_ENA 0
#endif


// implementaiton of specific for macro control types
#if (CONTROL_TYPE == USART3_CONTROLLED)
  // hoverboard sensor functionality is disabled
  // and control is via USART3
  #define INCLUDE_PROTOCOL INCLUDE_PROTOCOL2
  #define SERIAL_USART3_IT
  #define PASE_ADV_ENA 0
#endif


//////////////////////////////////////////////////////////
// Default Values


// ############################### ENABLE FLASH STORAGE MECHANISM ###############################
// this includes flasharea.c and flashaccess.c
#ifndef FLASH_STORAGE
  #define FLASH_STORAGE 1
#endif

// ############################### ENABLE INTERRUPT READING OF HALL SENSORS FOR POSITION ###############################
#ifndef HALL_INTERRUPTS
  #define HALL_INTERRUPTS 1
#endif


#ifndef USART2_BAUD_SENSE
  #define USART2_BAUD_SENSE 0
#endif
#ifndef USART3_BAUD_SENSE
  #define USART3_BAUD_SENSE 0
#endif

// used when USART2 is enabled by power button as a protocol port.
#ifndef USART2PROTOCOLBAUD
  #define USART2PROTOCOLBAUD 9600
#endif

#ifndef SENSOR_WORDS
  #define SENSOR_WORDS 10
#endif

#ifndef SOFTWARE_SERIAL_RX_PIN
  #define SOFTWARE_SERIAL_RX_PIN GPIO_PIN_2
#endif
#ifndef SOFTWARE_SERIAL_RX_PORT
  #define SOFTWARE_SERIAL_RX_PORT GPIOB
#endif
#ifndef   SOFTWARE_SERIAL_TX_PIN
  #define SOFTWARE_SERIAL_TX_PIN GPIO_PIN_9
#endif
#ifndef   SOFTWARE_SERIAL_TX_PORT
  #define SOFTWARE_SERIAL_TX_PORT GPIOC
#endif
#ifndef FLASH_DEFAULT_HOVERBOARD_ENABLE
  #define FLASH_DEFAULT_HOVERBOARD_ENABLE 1
#endif
#ifndef USART2_BAUD
  #define USART2_BAUD 115200
#endif
#ifndef USART3_BAUD
  #define USART3_BAUD 115200
#endif
#ifndef SERIAL_USART_IT_BUFFERTYPE
  #define SERIAL_USART_IT_BUFFERTYPE unsigned char
#endif
#ifndef USART2_WORDLENGTH
  #define USART2_WORDLENGTH UART_WORDLENGTH_8B
#endif
#ifndef USART3_WORDLENGTH
  #define USART3_WORDLENGTH UART_WORDLENGTH_8B
#endif

#ifndef SOFTWARE_SERIAL_BAUD
  #define SOFTWARE_SERIAL_BAUD 9600
#endif


// ############################### DO-NOT-TOUCH SETTINGS ###############################

#ifndef PWM_FREQ
  #define PWM_FREQ         16000      // PWM frequency in Hz
#endif
#ifndef DEAD_TIME
  #define DEAD_TIME        32         // PWM deadtime
#endif
#ifndef DELAY_IN_MAIN_LOOP
  #define DELAY_IN_MAIN_LOOP 5        // in ms. default 5. it is independent of all the timing critical stuff. do not touch if you do not know what you are doing.
#endif

#ifndef INPUT_TIMEOUT
  #define INPUT_TIMEOUT    30         // number of wrong / missing input commands before wheels are disabled
#endif
// ############################### GENERAL ###############################

// ############################### MOTOR CONTROL (overwrite) #########################
#ifndef CTRL_TYP_SEL
  #define CTRL_TYP_SEL            3   // [-] Control method selection: -1 = Original 'Block PWM', 0 = Commutation , 1 = Pure Trapezoidal , 2 = Sinusoidal, 3 = Sinusoidal 3rd armonic (default)
#endif
#ifndef PHASE_ADV_ENA
  #define PHASE_ADV_ENA           1   // [-] Phase advance enable parameter: 0 = disabled, 1 = enabled (default)
#endif
// note: these two are now set from main based on 80/70 being (maybe) ok at 8khz
#ifndef COMM_DEACV_HI
  #define COMM_DEACV_HI          30   // [rpm] Commutation method deactivation speed high (above this value the control switches from Commutation method to Selected method above)
#endif

#ifndef COMM_ACV_LO
  #define COMM_ACV_LO            15   // [rpm] Commutation method activation speed low
#endif

// How to calibrate: connect GND and RX of a 3.3v uart-usb adapter to the right sensor board cable (be careful not to use the red wire of the cable. 15v will destroye verything.). if you are using nunchuck, disable it temporarily. enable DEBUG_SERIAL_USART3 and DEBUG_SERIAL_ASCII use asearial terminal.

// Battery voltage calibration: connect power source. see <How to calibrate>. write value nr 5 to BAT_CALIB_ADC. make and flash firmware. then you can verify voltage on value 6 (devide it by 100.0 to get calibrated voltage).
#ifndef BAT_CALIB_REAL_VOLTAGE
  #define BAT_CALIB_REAL_VOLTAGE        43.0       // input voltage measured by multimeter
#endif
#ifndef BAT_CALIB_ADC
  #define BAT_CALIB_ADC                 1704       // adc-value measured by mainboard (value nr 4 on UART debug output)
#endif

#ifndef BAT_NUMBER_OF_CELLS
  #define BAT_NUMBER_OF_CELLS     10        // normal Hoverboard battery: 10s
#endif

#ifndef BAT_LOW_LVL1_ENABLE
  #define BAT_LOW_LVL1_ENABLE     0         // to beep or not to beep, 1 or 0
#endif
#ifndef BAT_LOW_LVL1
  #define BAT_LOW_LVL1            3.6       // gently beeps at this voltage level. [V/cell]
#endif

#ifndef BAT_LOW_LVL2_ENABLE
  #define BAT_LOW_LVL2_ENABLE     1         // to beep or not to beep, 1 or 0
#endif
#ifndef BAT_LOW_LVL2
  #define BAT_LOW_LVL2            3.5       // your battery is almost empty. Charge now! [V/cell]
#endif
#ifndef BAT_LOW_DEAD
  #define BAT_LOW_DEAD            3.37      // undervoltage poweroff. (while not driving) [V/cell]
#endif

#ifndef DC_CUR_LIMIT
  #define DC_CUR_LIMIT     15         // DC current limit in amps per motor. so 15 means it will draw 30A out of your battery. it does not disable motors, it is a soft current limit.
#endif

// Board overheat detection: the sensor is inside the STM/GD chip. it is very inaccurate without calibration (up to 45°C). so only enable this funcion after calibration! let your board cool down. see <How to calibrate>. get the real temp of the chip by thermo cam or another temp-sensor taped on top of the chip and write it to TEMP_CAL_LOW_DEG_C. write debug value 8 to TEMP_CAL_LOW_ADC. drive around to warm up the board. it should be at least 20°C warmer. repeat it for the HIGH-values. enable warning and/or poweroff and make and flash firmware.
#ifndef TEMP_CAL_LOW_ADC
  #define TEMP_CAL_LOW_ADC        1655      // temperature 1: ADC value
#endif
#ifndef TEMP_CAL_LOW_DEG_C
  #define TEMP_CAL_LOW_DEG_C      35.8      // temperature 1: measured temperature [°C]
#endif
#ifndef TEMP_CAL_HIGH_ADC
  #define TEMP_CAL_HIGH_ADC       1588      // temperature 2: ADC value
#endif
#ifndef TEMP_CAL_HIGH_DEG_C
  #define TEMP_CAL_HIGH_DEG_C     48.9      // temperature 2: measured temperature [°C]
#endif
#ifndef TEMP_WARNING_ENABLE
  #define TEMP_WARNING_ENABLE     0         // to beep or not to beep, 1 or 0, DO NOT ACTIVITE WITHOUT CALIBRATION!
#endif
#ifndef TEMP_WARNING
  #define TEMP_WARNING            60        // annoying fast beeps [°C]
#endif
#ifndef TEMP_POWEROFF_ENABLE
  #define TEMP_POWEROFF_ENABLE    0         // to poweroff or not to poweroff, 1 or 0, DO NOT ACTIVITE WITHOUT CALIBRATION!
#endif
#ifndef TEMP_POWEROFF
  #define TEMP_POWEROFF           65        // overheat poweroff. (while not driving) [°C]
#endif

#ifndef INACTIVITY_TIMEOUT
  #define INACTIVITY_TIMEOUT 8        // minutes of not driving until poweroff. it is not very precise.
#endif

// ############################### SERIAL DEBUG ###############################

//#define DEBUG_SERIAL_USART3         // right sensor board cable, disable if I2C (nunchuk) is used!
//#define DEBUG_SERIAL_SENSOR         // send to USART3 sensor board, without framing, at the CONTROL_SENSOR_BAUD rate
//#define DEBUG_SERIAL_SERVOTERM
//#define DEBUG_SERIAL_ASCII          // "1:345 2:1337 3:0 4:0 5:0 6:0 7:0 8:0\r\n"

// ############################### INPUT ###############################

// ###### CONTROL VIA RC REMOTE ######
// left sensor board cable. Channel 1: steering, Channel 2: speed.
//#define CONTROL_PPM                 // use PPM-Sum as input. disable DEBUG_SERIAL_USART2!
//#define PPM_NUM_CHANNELS 6          // total number of PPM channels to receive, even if they are not used.


// ###### CONTROL VIA TWO POTENTIOMETERS ######
// ADC-calibration to cover the full poti-range: connect potis to left sensor board cable (0 to 3.3V) (do NOT use the red 15V wire in the cable!). see <How to calibrate>. turn the potis to minimum position, write value 1 to ADC1_MIN and value 2 to ADC2_MIN. turn to maximum position and repeat it for ADC?_MAX. make, flash and test it.
// Check out https://github.com/bipropellant/bipropellant-hoverboard-firmware/wiki/ADC-Configurations for sample configurations.

//#define CONTROL_ADC               // use ADC as input. disable DEBUG_SERIAL_USART2!
#ifndef ADC1_MIN
  #define ADC1_MIN         0        // min ADC1-value while poti at minimum-position (0 - 4095)
#endif

#ifndef ADC1_ZERO
  #define ADC1_ZERO     1500        // ADC1-value while poti at zero-position (0 - 4095)
#endif

#ifndef ADC1_MAX
  #define ADC1_MAX      4095        // max ADC1-value while poti at maximum-position (0 - 4095)
#endif

#ifndef ADC1_MULT_NEG
  #define ADC1_MULT_NEG  500.0f     // Use 1000.0f to calibrate from MIN to MAX
#endif

#ifndef ADC1_MULT_POS
  #define ADC1_MULT_POS 1500.0f     // Use 1000.0f to calibrate from MIN to MAX
#endif


#ifndef ADC2_MIN
  #define ADC2_MIN         0        // min ADC2-value while poti at minimum-position (0 - 4095)
#endif

#ifndef ADC2_ZERO
  #define ADC2_ZERO     2000        // ADC2-value while poti at zero-position (0 - 4095)
#endif

#ifndef ADC2_MAX
  #define ADC2_MAX      4095        // max ADC2-value while poti at maximum-position (0 - 4095)
#endif

#ifndef ADC2_MULT_NEG
  #define ADC2_MULT_NEG  300.0f     // Use 1000.0f to calibrate from MIN to MAX
#endif

#ifndef ADC2_MULT_POS
  #define ADC2_MULT_POS  300.0f     // Use 1000.0f to calibrate from MIN to MAX
#endif


#ifndef ADC_OFF_START
  #define ADC_OFF_START    0          // Start Value of Area at which other inputs can be active (0 - 4095) Applies to Speed ADC
#endif

#ifndef ADC_OFF_END
  #define ADC_OFF_END   1000          // End Value of Area at which other inputs can be active (0 - 4095) Applies to Speed ADC
#endif

#ifndef ADC_OFF_FILTER
  #define ADC_OFF_FILTER 1.0f         // Additional low pass Filter applied only to ADC Off functionality. 1.0=No Filter, 0.1 lots of Filtering
#endif

#ifndef ADC_SWITCH_CHANNELS
  #define ADC_SWITCH_CHANNELS 1       // define if ADC1 is used for Steer and ADC2 for Speed
#endif

#ifndef ADC_REVERSE_STEER
  #define ADC_REVERSE_STEER 1         // define if ADC1 is used for Steer and ADC2 for Speed
#endif

#ifndef ADC_TANKMODE
  #define ADC_TANKMODE 0              // define if each input should control one wheel
#endif

// ###### CONTROL VIA NINTENDO NUNCHUCK ######
// left sensor board cable. keep cable short, use shielded cable, use ferrits, stabalize voltage in nunchuck, use the right one of the 2 types of nunchucks, add i2c pullups. use original nunchuck. most clones does not work very well.
//#define CONTROL_NUNCHUCK            // use nunchuck as input. disable DEBUG_SERIAL_USART3!


//#define WHEEL_SIZE_INCHES 8.5 - set to your wheelsize to override the default 6.5


// ############################### SOFTWARE SERIAL ###############################
//
// there should now be a free choice of serial GPIO pins
#define SOFTWARE_SERIAL_BAUD 9600

// ############################### SERIAL PROTOCOL ###############################
#define NO_PROTOCOL 0
#define INCLUDE_PROTOCOL2 2 // enables processing of input characters through 'machine_protocol.c'

//#define INCLUDE_PROTOCOL NO_PROTOCOL
#ifndef INCLUDE_PROTOCOL
  #define INCLUDE_PROTOCOL NO_PROTOCOL
#endif
// Log PWM value in position/speed control mode
//define LOG_PWM

// ############################### DRIVING BEHAVIOR ###############################

// inputs:
// - cmd1 and cmd2: analog normalized input values. -1000 to 1000
// - button1 and button2: digital input values. 0 or 1
// - adc_buffer.l_tx2 and adc_buffer.l_rx2: unfiltered ADC values (you do not need them). 0 to 4095
// outputs:
// - speedR and speedL: normal driving -1000 to 1000
// - weakr and weakl: field weakening for extra boost at high speed (speedR > 700 and speedL > 700). 0 to ~400

#ifndef FILTER
  #define FILTER              0.1  // lower value == softer filter. do not use values <0.01, you will get float precision issues.
#endif
#ifndef SPEED_COEFFICIENT
  #define SPEED_COEFFICIENT   0.5  // higher value == stronger. 0.0 to ~2.0?
#endif
#ifndef STEER_COEFFICIENT
  #define STEER_COEFFICIENT   0.5  // higher value == stronger. if you do not want any steering, set it to 0.0; 0.0 to 1.0
#endif
#ifndef INVERT_R_DIRECTION
  #define INVERT_R_DIRECTION  1
#endif
#ifndef INVERT_L_DIRECTION
  #define INVERT_L_DIRECTION  1
#endif
#ifndef SWITCH_WHEELS
  #define SWITCH_WHEELS       0    // switch right and left wheel. Watch out, you probably also need to invert directions.
#endif
#ifndef BEEPS_BACKWARD
  #define BEEPS_BACKWARD 0    // 0 or 1
#endif


#if (INCLUDE_PROTOCOL == NO_PROTOCOL)
  #undef INCLUDE_PROTOCOL
#endif

// end of macro control type definitions
//////////////////////////////////////////////////////////

// ############################### VALIDATE SETTINGS ###############################

#if defined(DEBUG_SERIAL_USART2) && defined(DEBUG_SERIAL_USART3)
  #error DEBUG_SERIAL_USART2 and DEBUG_SERIAL_USART3 not allowed, choose one.
#endif

#if defined(DEBUG_SERIAL_USART2)
  #ifdef SENSOR_BOARD_CABLE_LEFT_IN_USE
    #error SERIAL_USART2 not allowed, cable already in use.
  #else
    #define SENSOR_BOARD_CABLE_LEFT_IN_USE
  #endif
#endif

#if defined(CONTROL_ADC)
  #ifdef SENSOR_BOARD_CABLE_LEFT_IN_USE
    #error CONTROL_ADC not allowed, cable already in use.
  #else
    #define SENSOR_BOARD_CABLE_LEFT_IN_USE
  #endif
#endif

#if defined(CONTROL_PPM)
  #ifdef SENSOR_BOARD_CABLE_LEFT_IN_USE
    #error CONTROL_PPM not allowed, cable already in use.
  #else
    #define SENSOR_BOARD_CABLE_LEFT_IN_USE
  #endif
  #ifdef CONTROL_METHOD_DEFINED
    #error CONTROL_PPM not allowed, another control Method is already defined.
  #else
    #define CONTROL_METHOD_DEFINED
  #endif
#endif


#if defined(DEBUG_SERIAL_USART3)
  #ifdef SENSOR_BOARD_CABLE_RIGHT_IN_USE
    #error SERIAL_USART3 not allowed, cable already in use.
  #else
    #define SENSOR_BOARD_CABLE_RIGHT_IN_USE
  #endif
#endif


#if defined(CONTROL_NUNCHUCK)
  #ifdef SENSOR_BOARD_CABLE_RIGHT_IN_USE
    #error CONTROL_NUNCHUCK not allowed, cable already in use.
  #else
    #define SENSOR_BOARD_CABLE_RIGHT_IN_USE
  #endif
  #ifdef CONTROL_METHOD_DEFINED
    #error CONTROL_NUNCHUCK not allowed, another control Method is already defined.
  #else
    #define CONTROL_METHOD_DEFINED
  #endif
#endif

#if defined(INCLUDE_PROTOCOL)
  #ifdef CONTROL_METHOD_DEFINED
    #error INCLUDE_PROTOCOL not allowed, another control Method is already defined.
  #else
    #define CONTROL_METHOD_DEFINED
  #endif
#endif

#if defined(SERIAL_USART2_IT)
  #ifdef SENSOR_BOARD_CABLE_LEFT_IN_USE
    #error SERIAL_USART2_IT not allowed, cable already in use.
  #else
    #define SENSOR_BOARD_CABLE_LEFT_IN_USE
  #endif
#endif

#if defined(SERIAL_USART3_IT)
  #ifdef SENSOR_BOARD_CABLE_RIGHT_IN_USE
    #error SERIAL_USART3_IT not allowed, cable already in use.
  #else
    #define SENSOR_BOARD_CABLE_RIGHT_IN_USE
  #endif
#endif

#if defined(INCLUDE_PROTOCOL) && !(defined(SERIAL_USART2_IT) || defined(SERIAL_USART3_IT) || defined(SOFTWARE_SERIAL) )
  #error Either SERIAL_USART2_IT, SERIAL_USART3_IT or SOFTWARE_SERIAL has to be selected when using INCLUDE_PROTOCOL.
#endif
