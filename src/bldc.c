
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "bldc.h"
#include "hallinterrupts.h"

volatile ELECTRICAL_PARAMS electrical_measurements;

#define DO_MEASUREMENTS


volatile int posl = 0;
volatile int posr = 0;
volatile int pwml = 0;
volatile int pwmr = 0;
volatile int weakl = 0;
volatile int weakr = 0;

extern volatile int speed;

extern volatile adc_buf_t adc_buffer;

extern volatile uint32_t timeout;
extern uint8_t disablepoweroff;

uint32_t buzzerFreq = 0;
uint32_t buzzerPattern = 0;

uint8_t enable = 0;

const int pwm_res = 64000000 / 2 / PWM_FREQ; // = 2000

const uint8_t hall_to_pos[8] = {
    0,
    0,
    2,
    1,
    4,
    5,
    3,
    0,
};

/*
static const float DUTY_SINUSOIDAL[6][16]= {
		{0.86328125, 0.89453125, 0.921875, 0.9453125, 0.96484375, 0.9765625, 0.98828125, 0.99609375,	0.99609375, 0.99609375, 0.98828125, 0.98046875, 0.96484375, 0.9453125, 0.92578125, 0.8984375},
		{0.8671875, 0.890625, 0.91796875, 0.94140625, 0.9609375, 0.9765625, 0.98828125, 0.99609375,	0.99609375, 0.99609375, 0.9921875, 0.98046875, 0.96484375, 0.94921875, 0.92578125, 0.8984375},
		{0.87109375, 0.8359375, 0.796875, 0.7578125, 0.71484375, 0.66796875, 0.6171875, 0.5625,	0.5078125, 0.453125, 0.39453125, 0.33203125, 0.26953125, 0.20703125, 0.140625, 0.078125},
		{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
		{0.0078125, 0.07421875, 0.13671875, 0.203125, 0.265625, 0.328125, 0.38671875, 0.44921875,0.49609375, 0.5546875, 0.60546875, 0.65625, 0.703125, 0.75, 0.7890625, 0.828125}
};
*/

static const int DUTY_SINUSOIDAL_INT[6][16]= {
		{86, 89, 92, 95, 96, 98, 99, 100,	100, 100, 99, 98, 96, 95, 93, 90},
		{87, 89, 92, 94, 96, 98, 99, 100,	100, 100, 99, 98, 96, 95, 93, 90},
		{87, 84, 80, 76, 71, 67, 62, 56,	50, 45, 39, 33, 27, 21, 14, 8},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{1, 7, 14, 20, 27, 33, 39, 45, 50, 55, 61, 66, 70, 75, 79, 83}
};

int sign[6] = {1, 1, 0, -1, -1, 0};

static inline void blockPWMSin(int pwm, int pos, int fraction, int *u, int *v, int *w) {
  *u = (pwm*(DUTY_SINUSOIDAL_INT[(pos+2)%6][fraction]-50))/50;
  *v = (pwm*(DUTY_SINUSOIDAL_INT[(pos+0)%6][fraction]-50))/50;
  *w = (pwm*(DUTY_SINUSOIDAL_INT[(pos+4)%6][fraction]-50))/50;
}


static inline void blockPWM(int pwm, int pos, int *u, int *v, int *w) {
  switch(pos) {
    case 0:
      *u = 0;
      *v = pwm;
      *w = -pwm;
      break;
    case 1:
      *u = -pwm;
      *v = pwm;
      *w = 0;
      break;
    case 2:
      *u = -pwm;
      *v = 0;
      *w = pwm;
      break;
    case 3:
      *u = 0;
      *v = -pwm;
      *w = pwm;
      break;
    case 4:
      *u = pwm;
      *v = -pwm;
      *w = 0;
      break;
    case 5:
      *u = pwm;
      *v = 0;
      *w = -pwm;
      break;
    default:
      *u = 0;
      *v = 0;
      *w = 0;
  }
}

static inline void blockPhaseCurrent(int pos, int u, int v, int *q) {
  switch(pos) {
    case 0:
      *q = u - v;
      // *u = 0;
      // *v = pwm;
      // *w = -pwm;
      break;
    case 1:
      *q = u;
      // *u = -pwm;
      // *v = pwm;
      // *w = 0;
      break;
    case 2:
      *q = u;
      // *u = -pwm;
      // *v = 0;
      // *w = pwm;
      break;
    case 3:
      *q = v;
      // *u = 0;
      // *v = -pwm;
      // *w = pwm;
      break;
    case 4:
      *q = v;
      // *u = pwm;
      // *v = -pwm;
      // *w = 0;
      break;
    case 5:
      *q = -(u - v);
      // *u = pwm;
      // *v = 0;
      // *w = -pwm;
      break;
    default:
      *q = 0;
      // *u = 0;
      // *v = 0;
      // *w = 0;
  }
}

uint32_t buzzerTimer        = 0;

int offsetcount = 0;
int offsetrl1   = 2000;
int offsetrl2   = 2000;
int offsetrr1   = 2000;
int offsetrr2   = 2000;
int offsetdcl   = 2000;
int offsetdcr   = 2000;

float batteryVoltage = BAT_NUMBER_OF_CELLS * 4.0;

int curl = 0;
// int errorl = 0;
// int kp = 5;
// volatile int cmdl = 0;

int last_pos = 0;
int timer = 0;
const int max_time = PWM_FREQ / 10;
volatile int vel = 0;

volatile int counts_to_change = 0;



//scan 8 channels with 2ADCs @ 20 clk cycles per sample
//meaning ~80 ADC clock cycles @ 8MHz until new DMA interrupt =~ 100KHz
//=640 cpu cycles
// Careful - easy to use too many!
void DMA1_Channel1_IRQHandler() {
  __disable_irq(); // but we want both values at the same time, without interferance
#ifdef HALL_INTERRUPTS
  unsigned long time = h_timer_hall.Instance->CNT;
  long long timerwraps_copy = timerwraps;
#endif
  unsigned char hall[2];
  hall[0] = (~(LEFT_HALL_U_PORT->IDR & (LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN))/LEFT_HALL_U_PIN) & 7;
  hall[1] = (~(RIGHT_HALL_U_PORT->IDR & (RIGHT_HALL_U_PIN | RIGHT_HALL_V_PIN | RIGHT_HALL_W_PIN))/RIGHT_HALL_U_PIN) & 7;
  __enable_irq();


  // ok, so that last transtion from one hall sector tyo the next took
  //HallData[i].HallTimeDiff
  // so we assume this one will...
  // and use this to chose one of 16 phases
  // which we'll modulate the pwm with?
#ifdef HALL_INTERRUPTS
  long long timenow = (timerwraps_copy << 16) | time;
  char fraction[2];
  for (int i = 0; i < 2; i++){
    long long dt = timenow - local_hall_params[i].last_hall_time;

    local_hall_params[i].dmacount -= dt/10;
    if (dt > local_hall_params[i].dmacount)
      local_hall_params[i].dmacount = dt;

    dt *= 16;
    int dta = dt/HallData[i].HallTimeDiff;

    if (dta < 0) dta = 0;
    fraction[i] = dta;
    if (fraction[i] > 15) fraction[i] = 15;
  }
#endif

  DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  if(offsetcount < 1000) {  // calibrate ADC offsets
    offsetcount++;
    offsetrl1 = (adc_buffer.rl1 + offsetrl1) / 2;
    offsetrl2 = (adc_buffer.rl2 + offsetrl2) / 2;
    offsetrr1 = (adc_buffer.rr1 + offsetrr1) / 2;
    offsetrr2 = (adc_buffer.rr2 + offsetrr2) / 2;
    offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
    offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
    return;
  }

  if (buzzerTimer % 1000 == 0) {  // because you get float rounding errors if it would run every time
    batteryVoltage = batteryVoltage * 0.99 + ((float)adc_buffer.batt1 * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01;
//#ifdef DO_MEASUREMENTS
    electrical_measurements.batteryVoltage = batteryVoltage;
//#endif
  }

  float dclAmps = ((float)ABS(adc_buffer.dcl - offsetdcl) * MOTOR_AMP_CONV_DC_AMP);
  float dcrAmps = ((float)ABS(adc_buffer.dcr - offsetdcr) * MOTOR_AMP_CONV_DC_AMP);

  electrical_measurements.motors[0].dcAmps = dclAmps;
  electrical_measurements.motors[1].dcAmps = dcrAmps;

#ifdef DO_MEASUREMENTS
  electrical_measurements.motors[0].dcAmpsAvgAcc += ABS(adc_buffer.dcl - offsetdcl);
  electrical_measurements.motors[1].dcAmpsAvgAcc += ABS(adc_buffer.dcl - offsetdcl);

  if (buzzerTimer % 1000 == 500) { // to save CPU time
    electrical_measurements.motors[0].dcAmpsAvg = electrical_measurements.motors[0].dcAmpsAvgAcc*MOTOR_AMP_CONV_DC_AMP/1000;
    electrical_measurements.motors[1].dcAmpsAvg = electrical_measurements.motors[1].dcAmpsAvgAcc*MOTOR_AMP_CONV_DC_AMP/1000;
    electrical_measurements.motors[0].dcAmpsAvgAcc = 0;
    electrical_measurements.motors[1].dcAmpsAvgAcc = 0;
  }
#endif

  //disable PWM when current limit is reached (current chopping)
  if(dclAmps > electrical_measurements.dcCurLim || timeout > TIMEOUT || enable == 0) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  } else {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
    //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 0);
  }

  if(dcrAmps > electrical_measurements.dcCurLim || timeout > TIMEOUT || enable == 0) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  int ul, vl, wl;
  int ur, vr, wr;

  posl = (hall_to_pos[hall[0]] + 2) % 6;
  posr = (hall_to_pos[hall[1]] + 2) % 6;

  blockPhaseCurrent(posl, adc_buffer.rl1 - offsetrl1, adc_buffer.rl2 - offsetrl2, &curl);

#ifdef DO_MEASUREMENTS
  electrical_measurements.motors[0].r1 = adc_buffer.rl1 - offsetrl1;
  electrical_measurements.motors[0].r2 = adc_buffer.rl2 - offsetrl2;
  electrical_measurements.motors[0].q  = curl;

  electrical_measurements.motors[1].r1 = adc_buffer.rr1 - offsetrr1;
  electrical_measurements.motors[1].r2 = adc_buffer.rr2 - offsetrr2;
  electrical_measurements.motors[1].q  = 0;//curl;
#endif

  //create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerTimer % buzzerFreq == 0) {
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    }
  } else {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
  }

  //update PWM channels based on position
  if (0){
  #ifdef HALL_INTERRUPTS
    blockPWMSin(pwml, posl, fraction[0], &ul, &vl, &wl);
    blockPWMSin(pwmr, posr, fraction[1], &ur, &vr, &wr);
  #endif
  } else {
    blockPWM(pwml, posl, &ul, &vl, &wl);
    blockPWM(pwmr, posr, &ur, &vr, &wr);
  }


  int weakul, weakvl, weakwl;
  if (pwml > 0) {
    blockPWM(weakl, (posl+5) % 6, &weakul, &weakvl, &weakwl);
  } else {
    blockPWM(-weakl, (posl+1) % 6, &weakul, &weakvl, &weakwl);
  }
  ul += weakul;
  vl += weakvl;
  wl += weakwl;

  int weakur, weakvr, weakwr;
  if (pwmr > 0) {
    blockPWM(weakr, (posr+5) % 6, &weakur, &weakvr, &weakwr);
  } else {
    blockPWM(-weakr, (posr+1) % 6, &weakur, &weakvr, &weakwr);
  }
  ur += weakur;
  vr += weakvr;
  wr += weakwr;

  LEFT_TIM->LEFT_TIM_U = CLAMP(ul + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_V = CLAMP(vl + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_W = CLAMP(wl + pwm_res / 2, 10, pwm_res-10);

  RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr + pwm_res / 2, 10, pwm_res-10);
}
