
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "bldc.h"
#include "hallinterrupts.h"
#include "comms.h"
#include "protocolfunctions.h"

// Matlab includes and defines - from auto-code generation
// ###############################################################################
#include "BLDC_controller.h"           /* Model's header file */
#include "rtwtypes.h"
#include <string.h>

// initial values set to defines.
// we may allow these to be changed later.
BLDC_PARAMS BldcControllerParams = {
  .ctrlTypSel = CTRL_TYP_SEL,
  .phaAdvEna = PHASE_ADV_ENA,
  .commDeacvHi = COMM_DEACV_HI,
  .commAcvLo = COMM_ACV_LO,

  .callFrequency = 8000,

  .initialized = 0, /* set to 1 once it has been intialised */
};


RT_MODEL rtM_Left_;    /* Real-time model */
RT_MODEL rtM_Right_;   /* Real-time model */
RT_MODEL *const rtM_Left = &rtM_Left_;
RT_MODEL *const rtM_Right = &rtM_Right_;

P rtP;                           /* Block parameters (auto storage) */

DW rtDW_Left;                    /* Observable states */
ExtU rtU_Left;                   /* External inputs */
ExtY rtY_Left;                   /* External outputs */

DW rtDW_Right;                   /* Observable states */
ExtU rtU_Right;                  /* External inputs */
ExtY rtY_Right;                  /* External outputs */

// ###############################################################################


volatile ELECTRICAL_PARAMS electrical_measurements = {
  .motors[0].pwm_limiter = 1024,
  .motors[1].pwm_limiter = 1024,
};

#define DO_MEASUREMENTS


//volatile int posl = 0;
//volatile int posr = 0;
volatile int pwml = 0;
volatile int pwmr = 0;

extern volatile int speed;

extern volatile adc_buf_t adc_buffer;

extern volatile uint32_t input_timeout_counter;
extern uint8_t disablepoweroff;

uint32_t buzzerFreq = 0;
uint32_t buzzerPattern = 0;
uint32_t buzzerTimer    = 0;

uint8_t enable = 0;
volatile int64_t bldc_counter = 0;
volatile unsigned  bldc_count = 0;

volatile unsigned  bldc_count_per_hall_counter[2] = {0,0};
const int pwm_res = 64000000 / 2 / PWM_FREQ; // = 2000

const uint8_t hall_to_pos[8] = {    0,    0,    2,    1,    4,    5,    3,    0, };

static inline void blockPWM(int pwm, int pos, int *u, int *v, int *w) {
  switch(pos) {
    case 0:      *u = 0;        *v = pwm;    *w = -pwm;     break;
    case 1:      *u = -pwm;     *v = pwm;    *w = 0;        break;
    case 2:      *u = -pwm;     *v = 0;      *w = pwm;      break;
    case 3:      *u = 0;        *v = -pwm;   *w = pwm;      break;
    case 4:      *u = pwm;      *v = -pwm;   *w = 0;        break;
    case 5:      *u = pwm;      *v = 0;      *w = -pwm;     break;
    default:     *u = 0;        *v = 0;      *w = 0;        break;
  }
}

static inline void blockPhaseCurrent(int pos, int u, int v, int *q) {
  switch(pos) {
    case 0:      *q = u - v;    break;
    case 1:      *q = u;        break;
    case 2:      *q = u;        break;
    case 3:      *q = v;        break;
    case 4:      *q = v;        break;
    case 5:      *q = -(u - v); break;
    default:      *q = 0;       break;
  }
}


int offsetcount = 0;

float batteryVoltage = BAT_NUMBER_OF_CELLS * 4.0;


void BldcController_Init(){
  /* Set BLDC controller parameters */
  /* must be called after timeStats.bldc_freq has been estimated */
  // ###############################################################################
  __disable_irq(); // but we want both values at the same time, without interferance
  BldcControllerParams.initialized = 0;

  rtP.z_ctrlTypSel        = BldcControllerParams.ctrlTypSel;
  rtP.b_phaAdvEna         = BldcControllerParams.phaAdvEna;

  //round(f_ctrl * a_mechAngle * (pi/180) * (30/pi))
  float coef = ((float) timeStats.bldc_freq) * 4.0 * (3.142/180.0) * (30.0/3.142);
  rtP.cf_speedCoef        = (int) coef;
  // base these on 8khz needing 80/70
  rtP.n_commDeacvHi       = BldcControllerParams.commDeacvHi*BldcControllerParams.callFrequency/8000;
  rtP.n_commAcvLo         = BldcControllerParams.commAcvLo*BldcControllerParams.callFrequency/8000;


  char tmp[256];
  sprintf(tmp, "cf_speedCoef %d, n_commDeacvHi %d, n_commAcvLo %d\r\n", rtP.cf_speedCoef, rtP.n_commDeacvHi, rtP.n_commAcvLo);
  consoleLog(tmp);

  /* Pack LEFT motor data into RTM */
  rtM_Left->defaultParam  = &rtP;
  rtM_Left->dwork         = &rtDW_Left;
  rtM_Left->inputs        = &rtU_Left;
  rtM_Left->outputs       = &rtY_Left;

  /* Pack RIGHT motor data into RTM */
  rtM_Right->defaultParam = &rtP;
  rtM_Right->dwork        = &rtDW_Right;
  rtM_Right->inputs       = &rtU_Right;
  rtM_Right->outputs      = &rtY_Right;

  /* Initialize BLDC controllers */
  BLDC_controller_initialize(rtM_Left);
  BLDC_controller_initialize(rtM_Right);
  // ###############################################################################

  BldcControllerParams.initialized = 1;
  __enable_irq(); // but we want both values at the same time, without interferance
}


//scan 8 channels with 2ADCs @ 20 clk cycles per sample
//meaning ~80 ADC clock cycles @ 8MHz until new DMA interrupt =~ 100KHz
//=640 cpu cycles
// Careful - easy to use too many!

#define BLDC_LIMITER_DECREMENT 5

// interrupt fires when ADC read is complete; read is triggerd from PWM timer.
// so interrupt fires at PWM rate = 16khz
void DMA1_Channel1_IRQHandler() {
  unsigned int hall[2];
  // note: the h_timer_hall IS a 100khz clock, which we read here just to have a high accuracy time
  // updated at 16khz.
  uint32_t time_in = DWT->CYCCNT;

  __disable_irq(); // but we want both values at the same time, without interferance
  hall[0] = (~(LEFT_HALL_U_PORT->IDR & (LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN))/LEFT_HALL_U_PIN) & 7;
  hall[1] = (~(RIGHT_HALL_U_PORT->IDR & (RIGHT_HALL_U_PIN | RIGHT_HALL_V_PIN | RIGHT_HALL_W_PIN))/RIGHT_HALL_U_PIN) & 7;
  __enable_irq();

  DMA1->IFCR = DMA_IFCR_CTCIF1;

  volatile adc_buf_t *buf = &adc_buffers.buffers[adc_buffers.adcBufferHead];

  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  buf->hall[0] = hall[0];
  buf->hall[1] = hall[1];

  // cycle through ADC buffers, so we can read them slow time later from main.
  adc_buffers.adcBufferHead = (adc_buffers.adcBufferHead + 1)%MAX_ADC_BUFFERS;
  if (adc_buffers.adcBufferHead == adc_buffers.adcBufferTail) {
    adc_buffers.adcBufferTail = (adc_buffers.adcBufferTail + 1)%MAX_ADC_BUFFERS;
  }
  DMA1_Channel1->CMAR  = (uint32_t)&adc_buffers.buffers[adc_buffers.adcBufferHead];

  if(offsetcount < 1000) {  // calibrate ADC offsets
    offsetcount++;
    adc_buffers.offsetrl1 = (buf->rl1 + adc_buffers.offsetrl1) / 2;
    adc_buffers.offsetrl2 = (buf->rl2 + adc_buffers.offsetrl2) / 2;
    adc_buffers.offsetrr1 = (buf->rr1 + adc_buffers.offsetrr1) / 2;
    adc_buffers.offsetrr2 = (buf->rr2 + adc_buffers.offsetrr2) / 2;
    adc_buffers.offsetdcl = (buf->dcl + adc_buffers.offsetdcl) / 2;
    adc_buffers.offsetdcr = (buf->dcr + adc_buffers.offsetdcr) / 2;
    return;
  }

  //create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerTimer % buzzerFreq == 0) {
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    }
  } else {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
  }

  // reduce to 8khz by running every other interrupt.
  // used to control freq.
  bldc_count++;
  int doLeft = 1;
  int doRight = 1;
  if (!(bldc_count & 1)) {
    doLeft = 1;
    // used to measure freq
    bldc_counter++;
  } else {
    doRight = 1;
    // used to measure freq
    bldc_counter++;
  }

  if (doLeft) {
    volatile MOTOR_ELECTRICAL *m = &electrical_measurements.motors[1];
    //disable PWM when current limit is reached (current chopping)
    if(input_timeout_counter > INPUT_TIMEOUT || enable == 0 || BldcControllerParams.initialized == 0) {
      LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
      //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
    } else {
      LEFT_TIM->BDTR |= TIM_BDTR_MOE;
      //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 0);
    }

    int abs_dc = ABS(buf->dcl - adc_buffers.offsetdcl);
    if(abs_dc > electrical_measurements.dc_adc_limit) {
      if (m->pwm_limiter > BLDC_LIMITER_DECREMENT) {
        m->pwm_limiter -= BLDC_LIMITER_DECREMENT;
      }
    } else {
      if (m->pwm_limiter < 1024) {
        m->pwm_limiter++;
      }
    }

    // multiply by 1024 with shift.  divide by 0-1024 according to limiter
    int pwm_mod = (m->pwm_limiter*pwml) >> 10;
    int ul = 0, vl = 0, wl = 0;
    //update PWM channels based on position

    if (BldcControllerParams.ctrlTypSel == BLDC_CONTROL_TYPE_ORIGINAL) {
      int posl = (hall_to_pos[hall[0]] + 2) % 6;
      blockPWM(pwm_mod, posl, &ul, &vl, &wl);
    } else {
      // else use new control style
      if (BldcControllerParams.initialized) {

      __disable_irq(); // but we want all values at the same time, without interferance
        rtU_Left.b_hallA   = hall_ul;
        rtU_Left.b_hallB   = hall_vl;
        rtU_Left.b_hallC   = hall_wl;
        rtU_Left.r_DC      = -pwm_mod;
      __enable_irq();

        /* Step the controller */
        BLDC_controller_step(rtM_Left);

        //if ((rtU_Left.b_hallA != hall_ul) || (rtU_Left.b_hallB != hall_vl) || (rtU_Left.b_hallC != hall_wl)){
        //  local_hall_params[0].hall_change_in_bldc_count++;
        //}

        /* Get motor outputs here */
        ul            = rtY_Left.DC_phaA;
        vl            = rtY_Left.DC_phaB;
        wl            = rtY_Left.DC_phaC;
      // motSpeedLeft = rtY_Left.n_mot;
      // motAngleLeft = rtY_Left.a_elecAngle;
      }
    }


    LEFT_TIM->LEFT_TIM_U = CLAMP(ul + pwm_res / 2, 10, pwm_res-10);
    LEFT_TIM->LEFT_TIM_V = CLAMP(vl + pwm_res / 2, 10, pwm_res-10);
    LEFT_TIM->LEFT_TIM_W = CLAMP(wl + pwm_res / 2, 10, pwm_res-10);
  }

  if (doRight) {
    volatile MOTOR_ELECTRICAL *m = &electrical_measurements.motors[1];

    if(input_timeout_counter > INPUT_TIMEOUT || enable == 0 || BldcControllerParams.initialized == 0) {
      RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
    } else {
      RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
    }

    int abs_dc = ABS(buf->dcr - adc_buffers.offsetdcr);
    if(abs_dc > electrical_measurements.dc_adc_limit) {
      if (m->pwm_limiter > BLDC_LIMITER_DECREMENT) {
        m->pwm_limiter -= BLDC_LIMITER_DECREMENT;
      }
    } else {
      if (m->pwm_limiter < 1024) {
        m->pwm_limiter++;
      }
    }


    // multiply by 1024 with shift.  divide by 0-1024 according to limiter
    int pwm_mod = (m->pwm_limiter*pwmr) >> 10;
    int ur = 0, vr = 0, wr = 0;
    //update PWM channels based on position

    if (BldcControllerParams.ctrlTypSel == BLDC_CONTROL_TYPE_ORIGINAL) {
      int posr = (hall_to_pos[hall[1]] + 2) % 6;
      blockPWM(pwm_mod, posr, &ur, &vr, &wr);
    } else {
      // else use new control style
      if (BldcControllerParams.initialized) {
      __disable_irq(); // but we want all values at the same time, without interferance
        rtU_Right.b_hallA  = hall_ur;
        rtU_Right.b_hallB  = hall_vr;
        rtU_Right.b_hallC  = hall_wr;
        rtU_Right.r_DC     = -pwm_mod;
      __enable_irq();

        /* Step the controller */
        BLDC_controller_step(rtM_Right);

        //if ((rtU_Right.b_hallA != hall_ur) || (rtU_Right.b_hallB != hall_vr) || (rtU_Right.b_hallC != hall_wr)){
        //  local_hall_params[1].hall_change_in_bldc_count++;
        //}

        /* Get motor outputs here */
        ur            = rtY_Right.DC_phaA;
        vr            = rtY_Right.DC_phaB;
        wr            = rtY_Right.DC_phaC;
    // motSpeedRight = rtY_Right.n_mot;
    // motAngleRight = rtY_Right.a_elecAngle;
      }
    }

    RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
    RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
    RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr + pwm_res / 2, 10, pwm_res-10);
  }

  /* do something */
  uint32_t time_out = DWT->CYCCNT;
  if (time_out < time_in) {
    time_out += 0x80000000;
    time_in += 0x80000000;
  }
  timeStats.bldc_cycles = (time_out - time_in);

  // if we are tranfer complete again before we finished, we're running too fast
  if (DMA1->ISR & DMA_ISR_TCIF1) {
    BldcControllerParams.overruns++;
  }
}

void readADCs() {
  // read the last one DMA completed
  int index = (adc_buffers.adcBufferHead + (MAX_ADC_BUFFERS-1))%MAX_ADC_BUFFERS;
  volatile adc_buf_t *buf = &adc_buffers.buffers[index];

  // some parts of main use this directly.
  memcpy((void *)&adc_buffer, (void *)buf, sizeof(adc_buffer));

  batteryVoltage = batteryVoltage * 0.99 + ((float)buf->batt1 * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01;
  electrical_measurements.batteryVoltage = batteryVoltage;

  float dclAmps = (float)ABS(buf->dcl - adc_buffers.offsetdcl) * MOTOR_AMP_CONV_DC_AMP;
  electrical_measurements.motors[0].dcAmps = dclAmps;
  electrical_measurements.motors[0].dcAmpsx100 = (int)(dclAmps*100.0);
  electrical_measurements.motors[0].dcAmpsAvg = electrical_measurements.motors[0].dcAmpsAvg*0.99 + electrical_measurements.motors[0].dcAmps*0.01;
  int posl = (hall_to_pos[buf->hall[0]] + 2) % 6;
  int curl = 0;
  blockPhaseCurrent(posl, buf->rl1 - adc_buffers.offsetrl1, buf->rl2 - adc_buffers.offsetrl2, &curl);

  electrical_measurements.motors[0].r1 = buf->rl1 - adc_buffers.offsetrl1;
  electrical_measurements.motors[0].r2 = buf->rl2 - adc_buffers.offsetrl2;
  electrical_measurements.motors[0].q  = curl;

  float dcrAmps = (float)ABS(buf->dcr - adc_buffers.offsetdcr) * MOTOR_AMP_CONV_DC_AMP;
  electrical_measurements.motors[1].dcAmps = dcrAmps;
  electrical_measurements.motors[1].dcAmpsx100 = (int)(dcrAmps*100.0);

  electrical_measurements.dc_adc_limit = ((float)electrical_measurements.dcCurLim/100.0)/MOTOR_AMP_CONV_DC_AMP;

  electrical_measurements.motors[1].dcAmpsAvg = electrical_measurements.motors[1].dcAmpsAvg*0.99 + electrical_measurements.motors[1].dcAmps*0.01;
  int posr = (hall_to_pos[buf->hall[1]] + 2) % 6;

  int curr = 0;
  blockPhaseCurrent(posr, buf->rr1 - adc_buffers.offsetrr1, buf->rr2 - adc_buffers.offsetrr2, &curr);
  electrical_measurements.motors[1].r1 = buf->rr1 - adc_buffers.offsetrr1;
  electrical_measurements.motors[1].r2 = buf->rr2 - adc_buffers.offsetrr2;
  electrical_measurements.motors[1].q  = curr;
}