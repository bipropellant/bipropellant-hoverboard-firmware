#pragma once
#include "stm32f1xx_hal.h"

// **************** NOTE!!!! ******
// if using platformio env, then this is set, and config.h is ignored!!!!!
#ifndef IGNORE_GLOBAL_CONFIG
  // **************** NOTE!!!! ******

  //////////////////////////////////////////////////////////
  // macro types for the hoverboard control style
  // add to add other control combinations
  #define HOVERBOARD_WITH_SOFTWARE_SERIAL_B2_C9 1
  #define USART2_CONTROLLED 2
  #define USART3_CONTROLLED 3
  #define SOFTWARE_SERIAL_A2_A3 4
  #define HOVERBOARD_WITH_SOFTWARE_SERIAL_B2_C9_6WORDSENSOR 5

  // thoery says this is the only thing you need to change....
  #define CONTROL_TYPE HOVERBOARD_WITH_SOFTWARE_SERIAL_B2_C9
  //////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////
  // implementaiton of specific for macro control types
  // provide a short explaination here
  #if (CONTROL_TYPE == HOVERBOARD_WITH_SOFTWARE_SERIAL_B2_C9)
    // this control type allows the board to be used AS a hoverboard,
    // responding to sensor movements when in hoverboard mode.
    /// and uses softwareserial for serial control on B2/C9
    #define READ_SENSOR
    #define CONTROL_SENSOR
    #define SOFTWARE_SERIAL
    #define SOFTWARE_SERIAL_RX_PIN GPIO_PIN_2
    #define SOFTWARE_SERIAL_RX_PORT GPIOB
    #define SOFTWARE_SERIAL_TX_PIN GPIO_PIN_9
    #define SOFTWARE_SERIAL_TX_PORT GPIOC
    //#define DEBUG_SERIAL_ASCII
    #define DEBUG_SOFTWARE_SERIAL
    #define FLASH_DEFAULT_HOVERBOARD_ENABLE 1
    #define SENSOR_BAUD     52177    // control via usart from GD32 based sensor boards @52177 baud (10 word)
    #define SENSOR_WORDS 10
    #define SERIAL_USART2_IT
    #define SERIAL_USART3_IT
  //#define SENSOR_BAUD     26315    // reported baudrate for other sensor boards (6 word)?
  //#define SENSOR_WORDS 6
  //#define SENSOR_BAUD     32100    // reported baudrate for another sensor board  (10 word 'Denver' brand hoverboards)
    // possibly baud rate based on ~2.5ms frame interval, so baud dependent on word count?
  #endif

  #if (CONTROL_TYPE == HOVERBOARD_WITH_SOFTWARE_SERIAL_B2_C9_6WORDSENSOR)
    // this control type allows the board to be used AS a hoverboard,
    // responding to sensor movements when in hoverboard mode.
    /// and uses softwareserial for serial control on B2/C9
    #define READ_SENSOR
    #define CONTROL_SENSOR
    #define SOFTWARE_SERIAL
    #define SOFTWARE_SERIAL_RX_PIN GPIO_PIN_2
    #define SOFTWARE_SERIAL_RX_PORT GPIOB
    #define SOFTWARE_SERIAL_TX_PIN GPIO_PIN_9
    #define SOFTWARE_SERIAL_TX_PORT GPIOC
    //#define DEBUG_SERIAL_ASCII
    #define DEBUG_SOFTWARE_SERIAL
    #define FLASH_DEFAULT_HOVERBOARD_ENABLE 1
  //#define SENSOR_BAUD     52177    // control via usart from GD32 based sensor boards @52177 baud (10 word)
  //#define SENSOR_WORDS 10
    #define SENSOR_BAUD     26315    // reported baudrate for other sensor boards (6 word)?
    #define SENSOR_WORDS 6
    #define SERIAL_USART2_IT
    #define SERIAL_USART3_IT
  //#define SENSOR_BAUD     32100    // reported baudrate for another sensor board (maybe 7 word)?
    // possibly baud rate based on ~2.5ms frame interval, so baud dependent on word count?
  #endif


  #if (CONTROL_TYPE == SOFTWARE_SERIAL_A2_A3)
    // hoverboard sensor functionality is disabled
    // and uses softwareserial for serial control on A2/A3 -
    // which are actually USART pins!
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
    #define SERIAL_USART2_IT
    //#define DEBUG_SERIAL_ASCII
  #endif


  // implementaiton of specific for macro control types
  #if (CONTROL_TYPE == USART3_CONTROLLED)
    // hoverboard sensor functionality is disabled
    // and control is via USART3
    #define SERIAL_USART3_IT
    //#define DEBUG_SERIAL_ASCII
  #endif


// ############################### ENABLE FLASH STORAGE MECHANISM ###############################
// this includes flasharea.c and flashaccess.c
  #define FLASH_STORAGE 1

// ############################### ENABLE INTERRUPT READING OF HALL SENSORS FOR POSITION ###############################
  #define HALL_INTERRUPTS 1

  // **************** NOTE!!!! ******
#endif // IGNORE_GLOBAL_CONFIG



#ifndef USART2_BAUD_SENSE
  #define USART2_BAUD_SENSE 0
#endif
#ifndef USART3_BAUD_SENSE
  #define USART3_BAUD_SENSE 0
#endif

// used when USART2 is enabled by power buutton as a protocol port.
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
  #define USART2_BAUD 26315        
#endif
#ifndef USART3_BAUD
  #define USART3_BAUD 26315        
#endif
#ifndef SERIAL_USART_IT_BUFFERTYPE
  #define SERIAL_USART_IT_BUFFERTYPE unsigned short
#endif
#ifndef USART2_WORDLENGTH
  #define USART2_WORDLENGTH UART_WORDLENGTH_9B
#endif
#ifndef USART3_WORDLENGTH
  #define USART3_WORDLENGTH UART_WORDLENGTH_9B
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

#ifndef TIMEOUT
  #define TIMEOUT          5          // number of wrong / missing input commands before emergency off
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

// ###### CONTROL VIA UART (serial) ######
//#define CONTROL_SERIAL_USART2       // left sensor board cable, disable if ADC or PPM is used!
                                      // control via usart from eg an Arduino or raspberry
// for Arduino, use void loop(void){ Serial.write((uint8_t *) &steer, sizeof(steer)); Serial.write((uint8_t *) &speed, sizeof(speed));delay(20); }

//////////////////////////////////////////////////////////////////

// ###### CONTROL VIA RC REMOTE ######
// left sensor board cable. Channel 1: steering, Channel 2: speed.
//#define CONTROL_PPM                 // use PPM-Sum as input. disable DEBUG_SERIAL_USART2!
//#define PPM_NUM_CHANNELS 6          // total number of PPM channels to receive, even if they are not used.

// ###### CONTROL VIA TWO POTENTIOMETERS ######
// ADC-calibration to cover the full poti-range: connect potis to left sensor board cable (0 to 3.3V) (do NOT use the red 15V wire in the cable!). see <How to calibrate>. turn the potis to minimum position, write value 1 to ADC1_MIN and value 2 to ADC2_MIN. turn to maximum position and repeat it for ADC?_MAX. make, flash and test it.
//#define CONTROL_ADC                 // use ADC as input. disable DEBUG_SERIAL_USART2!
#define ADC1_MIN 0                  // min ADC1-value while poti at minimum-position (0 - 4095)
#define ADC1_MAX 4095               // max ADC1-value while poti at maximum-position (0 - 4095)
#define ADC2_MIN 0                  // min ADC2-value while poti at minimum-position (0 - 4095)
#define ADC2_MAX 4095               // max ADC2-value while poti at maximum-position (0 - 4095)

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
  #define INCLUDE_PROTOCOL INCLUDE_PROTOCOL2
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
  #define INVERT_R_DIRECTION
#endif
#ifndef INVERT_L_DIRECTION
  #define INVERT_L_DIRECTION
#endif
#ifndef BEEPS_BACKWARD
  #define BEEPS_BACKWARD 0    // 0 or 1
#endif

//Turbo boost at high speeds while button1 is pressed:
//#define ADDITIONAL_CODE if (button1 && speedR > 700) { /* field weakening at high speeds */   weakl = cmd1 - 700; /* weak should never exceed 400 or 450 MAX!! */   weakr = cmd1 - 700; } else {   weakl = 0;   weakr = 0; }

// ###### SIMPLE BOBBYCAR ######
// for better bobbycar code see: https://github.com/larsmm/hoverboard-firmware-hack-bbcar
// #define FILTER              0.1
// #define SPEED_COEFFICIENT   -1
// #define STEER_COEFFICIENT   0


// ###### ARMCHAIR ######
// #define FILTER              0.05
// #define SPEED_COEFFICIENT   0.5
// #define STEER_COEFFICIENT   -0.2


#if (INCLUDE_PROTOCOL == NO_PROTOCOL)
  #undef INCLUDE_PROTOCOL
#endif

// end of macro control type definitions
//////////////////////////////////////////////////////////

// ############################### VALIDATE SETTINGS ###############################

#if defined CONTROL_SERIAL_USART2 && defined CONTROL_ADC
  #error CONTROL_ADC and CONTROL_SERIAL_USART2 not allowed. it is on the same cable.
#endif

#if defined CONTROL_SERIAL_USART2 && defined CONTROL_PPM
  #error CONTROL_PPM and CONTROL_SERIAL_USART2 not allowed. it is on the same cable.
#endif

#if defined DEBUG_SERIAL_USART3 && defined CONTROL_NUNCHUCK
  #error CONTROL_NUNCHUCK and DEBUG_SERIAL_USART3 not allowed. it is on the same cable.
#endif

#if defined DEBUG_SERIAL_USART3 && defined CONTROL_SENSOR
  #error DEBUG_SERIAL_USART3 and CONTROL_SENSOR not allowed. it is on the same cable.
#endif


#if defined CONTROL_PPM && defined CONTROL_ADC && defined CONTROL_NUNCHUCK || defined CONTROL_PPM && defined CONTROL_ADC || defined CONTROL_ADC && defined CONTROL_NUNCHUCK || defined CONTROL_PPM && defined CONTROL_NUNCHUCK
  #error only 1 input method allowed. use CONTROL_PPM or CONTROL_ADC or CONTROL_NUNCHUCK.
#endif
