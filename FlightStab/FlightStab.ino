
#include <avr/eeprom.h>
#include <util/atomic.h>

// check bit width math
// faster device libs 

// GYRO_ORIENTATION: roll right => -ve, pitch up => -ve, yaw right => -ve

#define RX3S_V1
//#define RX3S_V2
//#define NANO_MPU6050
#define USE_SERIAL

/* RX3S_V1 ************************************************************************************************/
#if defined(RX3S_V1)
/*
 OrangeRx Stabilizer RX3S V1
 PB0  8 AIL_IN            PC0 14/A0 AIL_GAIN       PD0 0 (RXD)
 PB1  9 ELE_IN (PWM)      PC1 15/A1 ELE_GAIN       PD1 1 AIL_SW (TXD)
 PB2 10 RUD_IN (PWM)      PC2 16/A2 RUD_GAIN       PD2 2 ELE_SW
 PB3 11 (MOSI) (PWM)      PC3 17/A3 PAD            PD3 3 RUD_SW (PWM)
 PB4 12 (MISO)            PC4 18/A4 (SDA)          PD4 4 AILL_OUT
 PB5 13 LED (SCK)         PC5 19/A5 (SCL)          PD5 5 ELE_OUT (PWM)
 PB6 14 (XTAL1)           PC6 (RESET)              PD6 6 RUD_OUT (PWM)
 PB7 15 (XTAL2)                                    PD7 7 AILR_OUT
 
 AIL_SINGLE mode (DEFAULT SETTING)
 PD7 7 AUX_IN instead of AILR_OUT
 
 AIL_DUAL mode
 PB3 11 AUX_IN instead of MOSI
 PB4 12 AILR_IN instead of MISO
*/

// <VR>
#define PORTC_AIN {&ail_vr, &ele_vr, &rud_vr, NULL, NULL, NULL, NULL, NULL}

// <RX> (must in PORT B/D due to ISR)
#define PORTB_PWM_IN {&ail_in, &ele_in, &rud_in, NULL, NULL, NULL, NULL, NULL}
#define PORTD_PWM_IN {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}

// <SWITCH>
#define PORTB_DIN {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
#define PORTC_DIN {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
#define PORTD_DIN {NULL, &ail_sw, &ele_sw, &rud_sw, NULL, NULL, NULL, NULL}

#define PWM_OUT_VAR {&ail_out, &ele_out, &rud_out, &ailr_out, NULL}
#define PWM_OUT_PIN {AIL_OUT_PIN, ELE_OUT_PIN, RUD_OUT_PIN, AILR_OUT_PIN, -1}

// <SERVO>
#define AIL_OUT_PIN 4
#define ELE_OUT_PIN 5
#define RUD_OUT_PIN 6
#define AILR_OUT_PIN 7 // dual aileron mode only

#define USE_ITG3200
#define GYRO_ORIENTATION(x, y, z) {gRoll = (y); gPitch = (x); gYaw = (z);}

#endif 
/* RX3S_V1 ************************************************************************************************/

/* RX3S_V2 ************************************************************************************************/
#if defined(RX3S_V2)
/*
 OrangeRx Stabilizer RX3S V2
 PB0  8 AIL_IN            PC0 14/A0 DELTA_SW       PD0 0 AUX_SW (RXD)
 PB1  9 ELE_IN (PWM)      PC1 15/A1 AIL_GAIN       PD1 1 AIL_SW (TXD)
 PB2 10 RUD_IN (PWM)      PC2 16/A2 ELE_GAIN       PD2 2 ELE_SW
 PB3 11 AUX_IN (MOSI/PWM) PC3 17/A3 RUD_GAIN       PD3 3 RUD_SW (PWM)
 PB4 12 VTAIL_SW (MISO)   PC4 18/A4 (SDA)          PD4 4 AILL_OUT
 PB5 13 LED (SCK)         PC5 19/A5 (SCL)          PD5 5 ELE_OUT (PWM)
 PB6 14 (XTAL1)           PC6 (RESET)              PD6 6 RUD_OUT (PWM)
 PB7 15 (XTAL2)                                    PD7 7 AILR_OUT
 
 AIL_SINGLE mode (DEFAULT SETTING)
 (no change)
 
 AIL_DUAL mode A
 PB3 11 AILR_IN instead of AUX_IN (no remote gain)

 AIL_DUAL mode B
 PD2 2 AILR_IN instead of ELE_SW

*/

// <VR>
#define PORTC_AIN {NULL, &ail_vr, &ele_vr, &rud_vr, NULL, NULL, NULL, NULL}

// <RX> (must in PORT B/D due to ISR)
#define PORTB_PWM_IN {&ail_in, &ele_in, &rud_in, NULL, NULL, NULL, NULL, NULL}
#define PORTD_PWM_IN {NULL, NULL, NULL, NULL, NULL, NULL, NULL, &aux_in}
// for dual ailerons:
// portb_pwm_in[3] = aux_in
// portb_pwm_in[4] = ailr_in
// portd_pwm_in[7] = null (remove aux_in)

// <SWITCH>
#define PORTB_DIN {NULL, NULL, NULL, NULL, &vtail_sw, NULL, NULL, NULL}
#define PORTC_DIN {&delta_sw, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
#define PORTD_DIN {&aux_sw, &ail_sw, &ele_sw, &rud_sw, NULL, NULL, NULL, NULL}

#define PWM_OUT_VAR {&ail_out, &ele_out, &rud_out, NULL, NULL}
#define PWM_OUT_PIN {AIL_OUT_PIN, ELE_OUT_PIN, RUD_OUT_PIN, -1, -1}
// for dual ailerons:
// pwm_out_var[3] = &ailr_out
// pwm_out_pin[3] = AILR_OUT_PIN

// <SERVO>
#define AIL_OUT_PIN 4
#define ELE_OUT_PIN 5
#define RUD_OUT_PIN 6
#define AILR_OUT_PIN 7 // dual aileron mode only

#define USE_ITG3200
#define GYRO_ORIENTATION(x, y, z) {gRoll = (y); gPitch = (x); gYaw = (z);}

#endif
/* RX3S_V2 ************************************************************************************************/


#if defined(NANO_MPU6050)
#undef USE_ITG3200
#define USE_MPU6050
#define GYRO_ORIENTATION(x, y, z) {gRoll = -(x); gPitch = (y); gYaw = (z);}
#endif

#if defined(USE_MPU6050)
  #include "Wire.h"
  #include "I2Cdev.h"
  #include "MPU6050.h"
  MPU6050 accelgyro;
#elif defined(USE_ITG3200)
  #include "Wire.h"
  #include "I2Cdev.h"
  #include "ITG3200.h"
  ITG3200 gyro;
#endif

//#define EEPROM_CFG_VER 1

#define LED_PIN 13 // standard on all devices (SCK)

// LED set
#define LED_ON 1
#define LED_OFF 2
#define LED_INVERT 3

// LED message pulse duration
#define LED_LONG 600
#define LED_SHORT 300
#define LED_VERY_SHORT 30

// adc
volatile uint8_t ail_vr = 128;
volatile uint8_t ele_vr = 128;
volatile uint8_t rud_vr = 128;

// rx
#define RX_WIDTH_MIN 900
#define RX_WIDTH_LOW 1000
#define RX_WIDTH_MID 1500
#define RX_WIDTH_HIGH 2000
#define RX_WIDTH_MAX 2100
//#define CPPM

volatile int16_t ail_in = RX_WIDTH_MID;
volatile int16_t ailr_in = RX_WIDTH_MID;
volatile int16_t ele_in = RX_WIDTH_MID;
volatile int16_t rud_in = RX_WIDTH_MID;
volatile int16_t aux_in = RX_WIDTH_HIGH; // assume max gain if no aux_in

int16_t ail_in_mid = RX_WIDTH_MID; // calibration sets stick-neutral-position offsets
int16_t ailr_in_mid = RX_WIDTH_MID; //
int16_t ele_in_mid = RX_WIDTH_MID; //
int16_t rud_in_mid = RX_WIDTH_MID; //

// switch
int8_t ail_sw = 0;
int8_t ele_sw = 0;
int8_t rud_sw = 0;
int8_t vtail_sw = 0;
int8_t delta_sw = 0;
int8_t aux_sw = 0;

// servo
#define SERVO_FRAME_PERIOD_DEFAULT 20000
int16_t servo_frame_period = SERVO_FRAME_PERIOD_DEFAULT;

int16_t ail_out = RX_WIDTH_MID;
int16_t ele_out = RX_WIDTH_MID;
int16_t rud_out = RX_WIDTH_MID;
int16_t ailr_out = RX_WIDTH_MID;

// imu
int16_t gRoll0=0, gPitch0=0, gYaw0=0; // calibration sets zero-movement-measurement offsets
int16_t gRoll=0, gPitch=0, gYaw=0; // full scale = 16b signed = 2000 deg/sec 
int16_t aRoll=0, aPitch=0, aYaw=0; // full scale = 16b signed = 16g? (TODO)

// pid
#define PID_PERIOD 10000
#define PID_KP_DEFAULT 500 // [0, 999] 11bits signed
#define PID_KI_DEFAULT 500
#define PID_KD_DEFAULT 500
int16_t kp[3] = {PID_KP_DEFAULT, PID_KP_DEFAULT, PID_KP_DEFAULT};
int16_t ki[3] = {PID_KI_DEFAULT, PID_KI_DEFAULT, PID_KI_DEFAULT};
int16_t kd[3] = {PID_KD_DEFAULT, PID_KD_DEFAULT, PID_KD_DEFAULT};

// mixer
enum MIX_MODE {MIX_NORMAL, MIX_DELTA, MIX_VTAIL};
enum MIX_MODE mix_mode = MIX_NORMAL;

// aileron mode
enum AIL_MODE {AIL_SINGLE, AIL_DUAL};
enum AIL_MODE ail_mode = AIL_SINGLE;

#define STICK_GAIN_MAX 400
#define MASTER_GAIN_MAX 800

/***************************************************************************************************************
 * 
 ***************************************************************************************************************/

int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

/***************************************************************************************************************
 * LED
 ***************************************************************************************************************/

int8_t led_pulse_count[4];
int16_t led_pulse_msec[4];

void set_led(int8_t i)
{
  digitalWrite(LED_PIN, i == LED_OFF ? LOW : i == LED_ON ? HIGH : digitalRead(LED_PIN) ^ 1);
}

void set_led_msg(int8_t slot, int8_t pulse_count, int16_t pulse_msec) {
  led_pulse_count[slot] = pulse_count;
  led_pulse_msec[slot] = pulse_msec;
}

void update_led(long now)
{
  static int8_t slot;
  static int8_t step;
  static long next_time;
  
  if (now < next_time)
    return;

  if (step == 0) {
    slot = (slot + 1) & 0x3;
    step = led_pulse_count[slot] << 1;
    next_time = now + 1000000;
    return;
  }

  step--;
  set_led(step & 0x1 ? LED_ON : LED_OFF);
  next_time = now + (long)led_pulse_msec[slot] << 10;
}

void terminal_led()
{
  set_led_msg(3, 40, LED_VERY_SHORT); 
  while (true) {
    update_led(micros());
  }
}

/***************************************************************************************************************
 * ANALOG IN (VR)
 ***************************************************************************************************************/

volatile uint8_t *adc_portc[] = PORTC_AIN;

void start_adc(uint8_t ch)
{
  ADMUX = (ADMUX & ~0x07) | ch; // set adc channel
  ADCSRA |= (1 << ADSC); // adc start conversion
}

void start_next_adc(uint8_t ch)
{
  while (ch < 8) { // start next adc channel that has a valid mapping
    if (adc_portc[ch]) {
      start_adc(ch);
      break;
    }
    ch++;
  }
}

ISR(ADC_vect)
{
  uint8_t ch = ADMUX & 0x07; // extract the channel of the ADC result
  *adc_portc[ch] = ADCH; // save only the 8 msbs
  start_next_adc(ch+1);
}

void init_analog_in()
{
  ADMUX = (1 << REFS0) | (1 << ADLAR); // acc vref, adc left adjust, adc channel 0
  ADCSRB = 0;
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (0x07); // adc enable, manual trigger, interrupt enable, prescaler 128
  start_next_adc(0); 
}

/***************************************************************************************************************
 * DIGITAL IN (DIP SW)
 ***************************************************************************************************************/

int8_t *din_portb[] = PORTB_DIN;
int8_t *din_portc[] = PORTC_DIN;
int8_t *din_portd[] = PORTD_DIN;

void init_digital_in_port_list(int8_t **pport_list, int8_t pin_base)
{
  for (int8_t i=0; i<8; i++) {
    if (pport_list[i]) {
      pinMode(pin_base + i, INPUT);
      digitalWrite(pin_base + i, HIGH);
    }
  }    
}

void init_digital_in_sw()
{
  init_digital_in_port_list(din_portb, 8);
  init_digital_in_port_list(din_portc, 14);
  init_digital_in_port_list(din_portd, 0);
}

void read_switches()
{
  for (int8_t i=0; i<8; i++) {
    if (din_portb[i])
     *din_portb[i] = digitalRead(8 + i);
    if (din_portc[i])
     *din_portc[i] = digitalRead(14 + i);
    if (din_portd[i])
     *din_portd[i] = digitalRead(0 + i);
  } 
}


/***************************************************************************************************************
 * DIGITAL IN (RX)
 ***************************************************************************************************************/

#if defined(CPPM)
volatile int16_t *rx_chan[] = {&rud_in, &ele_in, NULL, &ail_in, &aux_in, NULL, NULL, NULL}; // open9x RETA12 

ISR(PCINT0_vect)
{
  static long last_time;
  static uint8_t last_pinb;
  static uint8_t ch;
  long now;
  uint8_t pinb, rise;

  now = micros();
  pinb = PINB;
  sei();
  rise = pinb & ~last_pinb;
  last_pinb = pinb;

  // cppm on arduino pin 8 (PORT B) 
  if (rise & (1 << (8 - 8))) {
    long width = now - last_time;
    last_time = now;
    if (width > 5000 || ch > 7)) {
      ch = 0;
    } else if (width >= RX_WIDTH_MIN && width <= RX_WIDTH_MAX) {
      *rx_chan[ch] = width;
      ch++;
    }
  }
}

#else

volatile int16_t *rx_portb[] = PORTB_PWM_IN;

// PORTB PCINT0-PCINT7
ISR(PCINT0_vect)
{
  static long rise_time[8];
  static uint8_t last_pinb;
  long now;
  uint8_t pinb, diff, rise;

  now = micros();
  pinb = PINB;
  sei();
  diff = pinb ^ last_pinb;
  rise = pinb & ~last_pinb;
  last_pinb = pinb;

  for (int8_t i = PCINT0; i <= PCINT7; i++) {
    if (rx_portb[i] && diff & (1 << i)) {
      if (rise & (1 << i)) {
        rise_time[i] = now;
      } else {
        int16_t width = now - rise_time[i];
        if (width >= RX_WIDTH_MIN && width <= RX_WIDTH_MAX) {
          *rx_portb[i] = width;
        }
      }
    }
  }
}

volatile int16_t *rx_portd[] = PORTD_PWM_IN;

// PORTD PCINT16-PCINT23
ISR(PCINT2_vect)
{
  static long rise_time[8];
  static uint8_t last_pind;
  long now;
  uint8_t pind, diff, rise;

  now = micros();
  pind = PIND;
  sei();
  diff = pind ^ last_pind;
  rise = pind & ~last_pind;
  last_pind = pind;

  for (int8_t i = PCINT16; i <= PCINT23; i++) {
    if (rx_portd[i] && diff & (1 << i)) {
      if (rise & (1 << i)) {
        rise_time[i] = now;
      } else {
        int16_t width = now - rise_time[i];
        if (width >= RX_WIDTH_MIN && width <= RX_WIDTH_MAX) {
          *rx_portd[i] = width;
        }
      }
    }
  }
}

#endif

void init_digital_in_rx()
{
  // PORTB RX
  PCICR |= (1 << PCIE0);
  for (int8_t i=0; i<8; i++) {
    if (rx_portb[i]) {
      PCMSK0 |= 1 << (PCINT0 + i);
      pinMode(8 + i, INPUT);
      digitalWrite(8 + i, HIGH);
    }
  }

  // PORTD RX
  PCICR |= (1 << PCIE2);
  for (int8_t i=0; i<8; i++) {
    if (rx_portd[i]) {
      PCMSK2 |= 1 << (PCINT16 + i);
      pinMode(0 + i, INPUT);
      digitalWrite(0 + i, HIGH);
    }
  }
}

/***************************************************************************************************************
 * DIGITAL OUT (SERVO)
 ***************************************************************************************************************/

volatile int16_t *pwm_out_var[] = PWM_OUT_VAR;
volatile int8_t pwm_out_pin[] = PWM_OUT_PIN;
int8_t last_ch;

ISR(TIMER0_COMPA_vect)
{
  static long frame_start;
  static int8_t ch_fall = -1;
  static int8_t ch_rise = 0;
  static int8_t ch = ch_rise;
  static int16_t ch_wait;
  uint8_t wait;
  
  if (ch_fall >= 0) {
    digitalWrite(pwm_out_pin[ch_fall], LOW);
    ch_fall = -1;
  }
  if (ch_rise >= 0) {
    digitalWrite(pwm_out_pin[ch_rise], HIGH);
    if (ch_rise == 0)
      frame_start = micros();
    ch_wait = (*pwm_out_var[ch_rise] + 2) >> 2;
    ch_rise = -1;
  }

  if (ch_wait > 250) {
    wait = 250; // 250 == 1000us
  } else {
    wait = ch_wait;
    ch_fall = ch;
    if (pwm_out_var[++ch]) {
      ch_rise = ch;
    } else {
      ch = -1;
      ch_wait = 32767;
    }
  }
  ch_wait -= wait;

  if (ch < 0 && micros() - frame_start > servo_frame_period) {
    ch = ch_rise = 0;
  }
  
  OCR0A += wait;
}

void init_digital_out()
{
  int8_t i = 0;
  while (pwm_out_pin[i] >= 0) {
    pinMode(pwm_out_pin[i], OUTPUT);
    i++;
  }
 
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1 << OCIE0A); // Enable CTC interrupt
} 

/***************************************************************************************************************
 * PID
 ***************************************************************************************************************/

#define PID_I_THRESHOLD 2048
#define PID_I_WINDUP (((int32_t)1 << 13) - 1)
#define PID_KP_SHIFT 3 
#define PID_KI_SHIFT 6
#define PID_KD_SHIFT 2
#define PID_OUTPUT_SHIFT 10

// kp[], kd[], ki[] = [0, 1023] 11b signed
int16_t setpoint[3] = {0, 0, 0}; // [-8192, 8191] 14b signed, stick commanded rate or 0 for stabilize-only
int16_t input[3]; // [-8192, 8191] 14b signed gyro measured rate
int16_t last_input[3]; // [-8192, 8191] 14b
int32_t sum_iterm[3]; // clamped to PID_WINDUP
int16_t output[3]; //

void compute_pid() 
{
  int8_t i;
  int32_t err, diff;

  for (i=0; i<3; i++) {
    err = input[i] - setpoint[i];
    diff = input[i] - last_input[i];
    if (abs(diff) >= PID_I_THRESHOLD) {
      sum_iterm[i] = 0;
    } else {
      sum_iterm[i] = constrain(sum_iterm[i] + ((ki[i] * err) >> PID_KI_SHIFT), -PID_I_WINDUP-1, +PID_I_WINDUP); 
    }
    output[i] = (((kp[i] * err) >> PID_KP_SHIFT) + sum_iterm[i] - ((kd[i] * diff) >> PID_KD_SHIFT)) >> PID_OUTPUT_SHIFT;
    last_input[i] = input[i];

#if defined(USE_SERIAL) && 0
    if (i == 2) {
      Serial.print(input[i]); Serial.print('\t');
      Serial.print(err); Serial.print('\t');
      Serial.print(diff); Serial.print('\t');
      Serial.print((kp[i] * err) >> PID_KP_SHIFT); Serial.print('\t');
      Serial.print((ki[i] * err) >> PID_KI_SHIFT); Serial.print('\t');
      Serial.print(sum_iterm[i]); Serial.print('\t');
      Serial.print((kd[i] * diff) >> PID_KD_SHIFT); Serial.print('\t');
      Serial.println(output[i]);
    }
#endif
  }
}

/***************************************************************************************************************
 * EEPROM
 ***************************************************************************************************************/

struct eeprom_cfg {
  uint8_t ver;
  int16_t kp[3];
  int16_t ki[3];
  int16_t kd[3];
  int16_t servo_frame_period;
  uint8_t chksum;
};

uint8_t compute_chksum(void *buf, int8_t len) 
{
  uint8_t chksum = 0;
  for (int8_t i=0; i<len; i++)
    chksum += ((uint8_t *)buf)[i];
  return (chksum ^ 0xff) + 1;
}

void write_eeprom(struct eeprom_cfg *pcfg) 
{    
  pcfg->chksum = compute_chksum(pcfg, sizeof(*pcfg)-1);
  eeprom_write_block(pcfg, 0, sizeof(*pcfg));   
}

int8_t read_eeprom(struct eeprom_cfg *pcfg, uint8_t expect_ver) 
{
  eeprom_read_block(pcfg, 0, sizeof(*pcfg));
  if (pcfg->ver != expect_ver) {
    return -1;
  }
  if (compute_chksum(pcfg, sizeof(*pcfg)) != 0) {
    return -2;
  }
  return 0;
}


/***************************************************************************************************************
 * IMU SENSOR
 ***************************************************************************************************************/

void read_imu()
{
  int16_t gx, gy, gz;
#if defined(USE_MPU6050)
  //accelgyro.getMotion6(&aRoll, &aPitch, &aYaw, &gRoll, &gPitch, &gYaw);
  accelgyro.getRotation(&gx, &gy, &gz);
  GYRO_ORIENTATION(gx, gy, gz);  
#endif

#if defined(USE_ITG3200)
  gyro.getRotation(&gx, &gy, &gz);
  GYRO_ORIENTATION(gx, gy, gz);
#endif
}

void init_imu() {
#if defined(USE_MPU6050) or defined(USE_ITG3200)
  Wire.begin();
#endif

#if defined(USE_MPU6050)
  accelgyro.initialize();
  if (!accelgyro.testConnection()) {
    set_led_msg(2, 5, LED_SHORT);
    terminal_led(); // does not return
  }
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  accelgyro.setDLPFMode(MPU6050_DLPF_BW_256);
  accelgyro.setClockSource(MPU6050_CLOCK_PLL_ZGYRO); 
#endif

#if defined(USE_ITG3200)
  gyro.initialize();
  gyro.setClockSource(ITG3200_CLOCK_PLL_XGYRO); 
  gyro.setFullScaleRange(ITG3200_FULLSCALE_2000); 
  gyro.setDLPFBandwidth(ITG3200_DLPF_BW_5); 
//  gyro.setDLPFBandwidth(ITG3200_DLPF_BW_256); 
  if (!gyro.testConnection()) {
    set_led_msg(2, 5, LED_SHORT);
    terminal_led(); // does not return
  }
#endif
}

/***************************************************************************************************************
 * CALIBRATION
 ***************************************************************************************************************/

void calibrate_init_stat(int16_t *plow, int16_t *phigh, int32_t *ptot) 
{
  for (int8_t i=0; i<4; i++) {
    plow[i] = 32767;
    phigh[i] = -32768;
    ptot[i] = 0;
  }
}

void calibrate_update_stat(int16_t *plow, int16_t *phigh, int32_t *ptot, int16_t *psample, int8_t entries) 
{
  for (int8_t i=0; i<entries; i++) {
    plow[i] = min(plow[i], psample[i]);
    phigh[i] = max(phigh[i], psample[i]);
    ptot[i] += psample[i];
  }
} 

bool calibrate_check_stat(int16_t *plow, int16_t *phigh, int16_t range, int8_t entries)
{
  int8_t good = 0;
  for (int8_t i=0; i<entries; i++) {
    if (phigh[i] - plow[i] < range)
      good++;
  }
  return (good == entries);
}

void calibrate()
{
  long now;
  int16_t count;
  int16_t sample[4], low[4], high[4];
  int32_t tot[4];

  do {
    calibrate_init_stat(low, high, tot);    
    count = 0;
    now = micros();
    while (micros() - now < 1000000) {
      read_imu();
      sample[0] = gRoll;
      sample[1] = gPitch;
      sample[2] = gYaw;
      calibrate_update_stat(low, high, tot, sample, 3);  
      if (count++ % 100 == 0)
        set_led(LED_INVERT);
    }

#if defined(USE_SERIAL)
    Serial.println("GY");
    Serial.println(count);
    for (int8_t i=0; i<3; i++) {  
      Serial.print(low[i]); Serial.print('\t');
      Serial.print(high[i]); Serial.print('\t');
      Serial.println(tot[i]/count);
    }
#endif

  } while (!calibrate_check_stat(low, high, 50, 3));

  gRoll0 = tot[0]/count;
  gPitch0 = tot[1]/count;
  gYaw0 = tot[2]/count;

  do {
    calibrate_init_stat(low, high, tot);
    count = 0;
    now = micros();
    while (micros() - now < 1000000) {

      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sample[0] = ail_in;
        sample[1] = ailr_in;
        sample[2] = ele_in;
        sample[3] = rud_in;
      }

      if (ail_mode == AIL_SINGLE)
        ailr_in = ail_in;

      calibrate_update_stat(low, high, tot, sample, 4);  
      if (count++ % 50 == 0)
        set_led(LED_INVERT);
      delayMicroseconds(1000);
    }

#if defined(USE_SERIAL)
    Serial.println("SERVO");
    Serial.println(count);
    for (int8_t i=0; i<4; i++) {  
      Serial.print(low[i]); Serial.print('\t');
      Serial.print(high[i]); Serial.print('\t');
      Serial.println(tot[i]/count);
    }
#endif

  } while (!calibrate_check_stat(low, high, 20, 4));

  ail_in_mid = tot[0]/count;
  ailr_in_mid = tot[1]/count;
  ele_in_mid = tot[2]/count;
  rud_in_mid = tot[3]/count;

  set_led(LED_OFF);
}


/***************************************************************************************************************
 * STICK CONFIGURATION
 ***************************************************************************************************************/

int8_t zone(int16_t pwm)
{
  return pwm < 1200 ? 0 : pwm < 1800 ? 1 : 2;
}


void stick_config()
{
  int16_t ail_in2, ailr_in2, ele_in2, rud_in2, aux_in2;
  int8_t z0, z1, z2=-1;
  long z0_time;
  int8_t x, y=0;
  int8_t param[5];
  
  // param[0] 0=
  // param[1]
  // param[2]
  // param[3]
  // param[4]

  while (1) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ail_in2 = ail_in;
      ailr_in2 = ailr_in;
      ele_in2 = ele_in;
      rud_in2 = rud_in;
      aux_in2 = aux_in;
    }
    
    // stick zone code
    // 1 2 3
    // 4 5 6
    // 7 8 9
    z0 = zone(ele_in2) * 3 + zone(ail_in2) + 1;
    if (z0 != z1) {
      z1 = z0;
      z0_time = micros();
    } else {
      if (micros() - z0_time > 50000) {
        if (z0 != z2) {
          // stick moved from z2 -> z0
          int8_t z2z = z2 * 10 + z1;
          switch(z2z) {
            case 52: // zone 5 to zone 2
              y = y > 0 ? y-1 : y;
              break;
            case 58: // zone 5 to zone 8
              y = y < 4 ? y+1 : y;
              break;
          }
          z2 = z0;
        }
      }
    } 
  }    
}


/***************************************************************************************************************
 * SERIAL
 ***************************************************************************************************************/

#if defined(USE_SERIAL) && 0
void serial_tx(void *buf, int8_t len);
uint8_t serial_buf[48];

void print_buf(void *buf, int8_t len)
{
  uint8_t *p = (uint8_t *)buf;
  while (len--) {
    Serial.println(*p++, HEX);    
  }
}

#define REQUEST_GET_GYRO 100
#define REQUEST_GET_ACCEL 102
#define REQUEST_GET_RX 104
#define REQUEST_GET_SW 106
#define REQUEST_GET_VR 108

struct _request_get_gyro {
  uint8_t cmd;
};

struct _response_get_gyro {
  uint8_t cmd;
  int16_t gRoll;
  int16_t gPitch;
  int16_t gYaw;
};

struct _request_get_accel {
  uint8_t cmd;
};

struct _response_get_accel {
  uint8_t cmd;
  int16_t aRoll;
  int16_t aPitch;
  int16_t aYaw;
};

struct _request_get_rx {
  uint8_t cmd;
};

struct _response_get_rx {
  uint8_t cmd;
  int16_t ail;
  int16_t ele;
  int16_t rud;
  int16_t aux;
};

struct _request_get_sw {
  uint8_t cmd;
};

struct _response_get_sw {
  uint8_t cmd;
  int16_t ail;
  int16_t ele;
  int16_t rud;
};

struct _request_get_vr {
  uint8_t cmd;
};

struct _response_get_vr {
  uint8_t cmd;
  int16_t ail;
  int16_t ele;
  int16_t rud;
};

union serial_request {
  uint8_t cmd;
  struct _request_get_gyro get_gyro;
  struct _request_get_accel get_accel;
  struct _request_get_rx get_rx;
  struct _request_get_rx get_sw;
  struct _request_get_vr get_vr;
};

union serial_response {
  uint8_t cmd;
  struct _response_get_gyro get_gyro;
  struct _response_get_accel get_accel;
  struct _response_get_rx get_rx;
  struct _response_get_sw get_sw;
  struct _response_get_vr get_vr;
};

void handle_command()
{
  int8_t len;
  serial_request *preq = (serial_request *) serial_buf;
  serial_response *presp = (serial_response *) serial_buf;

  switch (serial_buf[0]) {
  case REQUEST_GET_GYRO:
    presp->get_gyro.gRoll = gRoll;
    presp->get_gyro.gPitch = gPitch;
    presp->get_gyro.gYaw = gYaw;
    len = sizeof(presp->get_gyro);
    break;
  case REQUEST_GET_ACCEL:
    presp->get_accel.aRoll = aRoll;
    presp->get_accel.aPitch = aPitch;
    presp->get_accel.aYaw = aYaw;
    len = sizeof(presp->get_accel);
    break;
  case REQUEST_GET_RX:
    presp->get_rx.ail = ail_in;
    presp->get_rx.ele = ele_in;
    presp->get_rx.rud = rud_in;
    presp->get_rx.aux = aux_in;
    len = sizeof(presp->get_rx);
    break;
  case REQUEST_GET_SW:
    presp->get_sw.ail = ail_sw;
    presp->get_sw.ele = ele_sw;
    presp->get_sw.rud = rud_sw;
    len = sizeof(presp->get_sw);
    break;
  case REQUEST_GET_VR:
    presp->get_vr.ail = ail_vr;
    presp->get_vr.ele = ele_vr;
    presp->get_vr.rud = rud_vr;
    len = sizeof(presp->get_vr);
    break;
  default:
    preq->cmd = 254; // presp->cmd will be 255
    len = 1;
    break;
  }
  presp->cmd = preq->cmd + 1;
  serial_tx(presp, len);
}

void serial_tx(void *buf, int8_t len)
{
  // TODO: use compute_checksum()
  uint8_t chksum = len+1;
  for (int8_t i=0; i<len; i++)
    chksum += ((uint8_t *)buf)[i];
  chksum = (chksum ^ 0xff) + 1; 

  // TODO: enable non-blocking writes
  Serial.write("$");
  Serial.write(len+1);
  Serial.write((uint8_t *)buf, len);
  Serial.write(chksum);
}

void serial_rx()
{
  static enum _serial_rx_state {
    IDLE, 
    HEADER,
    LENGTH
  } 
  state = IDLE;
  static uint8_t chksum;
  static uint8_t len;
  static uint8_t i;
  uint8_t ch;

  while (Serial.available() > 0) { 
    ch = Serial.read();
    switch(state) {
    case IDLE:
      if (ch == '$') state = HEADER;
      break;
    case HEADER:
      if (2 <= ch && ch <= sizeof(serial_buf)) {
        len = ch;
        chksum = ch;
        i = 0;
        state = LENGTH;
      } else {
        state = IDLE;
      }
      break;
    case LENGTH:
      serial_buf[i++] = ch;
      chksum += ch;
      if (--len == 0) {
        if (chksum == 0) {
          handle_command(); // invoke handler against serial_buf
        } else {
          // checksum error
          for (int k=0; k<20; k++) {
            set_led(LED_INVERT);
            delay(50);
          }
        }   
        state = IDLE;
      }
      break;    
    }

    if (state == IDLE)
      break;
  }
}

#endif

/***************************************************************************************************************
 * DUMP SENSORS
 ***************************************************************************************************************/

void dump_sensors()
{
#if defined(USE_SERIAL)  
  int16_t servo_out = 1500;
  int8_t servo_dir = 20;

  while (true) {
    int16_t ail_in2, ailr_in2, ele_in2, rud_in2, aux_in2;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ail_in2 = ail_in;
      ailr_in2 = ailr_in;
      ele_in2 = ele_in;
      rud_in2 = rud_in;
      aux_in2 = aux_in;
    }  

    Serial.print("RX "); 
    Serial.print(ail_in2); Serial.print(' ');
    Serial.print(ailr_in2); Serial.print(' ');
    Serial.print(ele_in2); Serial.print(' ');
    Serial.print(rud_in2); Serial.print(' ');
    Serial.print(aux_in2); Serial.print('\t');
  
    uint8_t ail_vr2, ele_vr2, rud_vr2;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ail_vr2 = ail_vr;
      ele_vr2 = ele_vr;
      rud_vr2 = rud_vr;
    }
    start_next_adc(0);

    Serial.print("VR "); 
    Serial.print(ail_vr2); Serial.print('\t');
    Serial.print(ele_vr2); Serial.print('\t');
    Serial.print(rud_vr2); Serial.print('\t');
    
    int8_t ail_sw2, ele_sw2, rud_sw2, vtail_sw2, delta_sw2, aux_sw2;
    read_switches();
    ail_sw2 = ail_sw;
    ele_sw2 = ele_sw;
    rud_sw2 = rud_sw;
    vtail_sw2 = vtail_sw;
    delta_sw2 = delta_sw;
    aux_sw2 = aux_sw;

    Serial.print("SW "); 
    Serial.print(ail_sw2); Serial.print(' ');
    Serial.print(ele_sw2); Serial.print(' ');
    Serial.print(rud_sw2); Serial.print(' ');
    Serial.print(vtail_sw2); Serial.print(' ');
    Serial.print(delta_sw2); Serial.print(' ');
    Serial.print(aux_sw2); Serial.print('\t');
  
    read_imu();
    Serial.print("GY "); 
    Serial.print(gRoll); Serial.print('\t');
    Serial.print(gPitch); Serial.print('\t');
    Serial.print(gYaw); Serial.print('\t');

    servo_out += servo_dir;
    if (servo_out < 1000 || servo_out > 2000) {
      servo_out = constrain(servo_out, 1000, 2000);
      servo_dir = -servo_dir;
    } 
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ail_out = ele_out = rud_out = ailr_out = servo_out;      
    }

    Serial.print("MISC "); 
    Serial.print(servo_out); Serial.print(' ');
    Serial.print(mix_mode); Serial.print(' ');
    Serial.print(ail_mode); Serial.print(' ');
    Serial.print(freeRam()); Serial.print(' ');

    Serial.println();

    set_led(LED_INVERT);
    delay(50);
  }
#endif
}

/***************************************************************************************************************
 * SETUP
 ***************************************************************************************************************/
 
void setup() 
{
  int8_t i;

#if defined(USE_SERIAL)
  Serial.begin(115200);
#endif

  pinMode(LED_PIN, OUTPUT);

  // init digital in for dip switches to read config settings
  init_digital_in_sw(); // sw
  read_switches();
  
  // device-specific modes and pin assignment differences
  // mix_mode = MIX_NORMAL default
  // ail_mode = AIL_SINGLE default

#if defined(RX3S_V1)
  switch ((ele_sw ? 2 : 0) | (rud_sw ? 1 : 0)) {
  case 0: // ele rev/0, rud rev/0
    ail_mode = AIL_DUAL;
    break; 
  case 1: // ele rev/0, rud norm/1
    mix_mode = MIX_VTAIL;
    break; 
  case 2: // ele norm/1, rud rev/0
    mix_mode = MIX_DELTA;
    break; 
  }

  if (ail_mode == AIL_SINGLE) {
    // PD7 7 AUX_IN instead of AILR_OUT
    pwm_out_var[3] = NULL; // disable ailr_out
    pwm_out_pin[3] = -1; //
    rx_portd[7] = &aux_in; // enable aux_in
  }

  if (ail_mode == AIL_DUAL) {
    // PB3 11 AUX_IN instead of MOSI
    // PB4 12 AILR_IN instead of MISO
    rx_portb[3] = &aux_in; // enable aux_in
    rx_portb[4] = &ailr_in; // enable ailr_in
  }
#endif

#if defined(RX3S_V2)
  switch ((vtail_sw ? 2 : 0) | (delta_sw ? 1 : 0)) {
  case 0: // vtail on/0, delta on/0
    ail_mode = AIL_DUAL;
    break; 
  case 1: // vtail on/0, delta off/1
    mix_mode = MIX_VTAIL;
    break; 
  case 2: // vtail off/1, delta on/0
    mix_mode = MIX_DELTA;
    break; 
  }

  if (ail_mode == AIL_DUAL) {
    if (ail_sw) {
      // AIL_DUAL mode A
      // PB3 11 AILR_IN instead of AUX_IN
      rx_portb[3] = &ailr_in; // replace aux_in
    } else {
      // AIL_DUAL mode B
      // PD2 2 AILR_IN instead of ELE_SW
      din_portd[2] = NULL; // disable ele_sw
      rx_portd[2] = &ailr_in; // enable ailr_in
    }
  }
#endif

  set_led_msg(0, mix_mode + 1, LED_LONG);

  init_analog_in(); // vr
  init_digital_in_rx(); // rx
  init_digital_out(); // servo
  init_imu(); // gyro/accelgyro

  //dump_sensors();

#if defined(EEPROM_CFG_VER)
  struct eeprom_cfg cfg;
  if (read_eeprom(&cfg, EEPROM_CFG_VER) < 0) {
    // init eeprom config parameters
    cfg.ver = EEPROM_CFG_VER;
    for (i=0; i<3; i++) {
      cfg.kp[i] = kp[i];
      cfg.ki[i] = ki[i];
      cfg.kd[i] = kd[i];
    }
    cfg.servo_frame_period = servo_frame_period;
    write_eeprom(&cfg);
  } else {
    // copy from eeprom cfg
    for (i=0; i<3; i++) {
      kp[i] = cfg.kp[i];
      ki[i] = cfg.ki[i];
      kd[i] = cfg.kd[i];
    }
    servo_frame_period = cfg.servo_frame_period;
  }
#endif

  delay(500);
  calibrate();

//  stick_config();
}

/***************************************************************************************************************
 * LOOP
 ***************************************************************************************************************/

void loop() 
{ 
  long t;
  static long last_t;

  static long last_vr_time;
  static long last_pid_time;

  static int16_t vr_gain[3];
  static int16_t stick_gain[3] = {STICK_GAIN_MAX, STICK_GAIN_MAX, STICK_GAIN_MAX};
  static int16_t master_gain;

  int8_t i;

  t = micros();
  last_t = t;

  update_led(t);

#if defined(USE_SERIAL) && 0
  serial_rx();
#endif

  if (t - last_vr_time > 100123) {
    // sample all adc channels
    uint8_t ail_vr2, ele_vr2, rud_vr2;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ail_vr2 = ail_vr;
      ele_vr2 = ele_vr;
      rud_vr2 = rud_vr;
    }

    // vr_gain[] [-128, 0, 127] => [-100%, 0%, 100%] = [-128, 0, 127]
    vr_gain[0] = (int16_t)ail_vr2 - 128;
    vr_gain[1] = (int16_t)ele_vr2 - 128;
    vr_gain[2] = (int16_t)rud_vr2 - 128;

    start_next_adc(0);
    last_vr_time = t;
  }

  if (t - last_pid_time > PID_PERIOD) {
    int16_t ail_in2, ailr_in2, ele_in2, rud_in2, aux_in2;
    int16_t ail_out2, ailr_out2, ele_out2, rud_out2;

    long t1 = micros();
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ail_in2 = ail_in;
      ailr_in2 = ailr_in;
      ele_in2 = ele_in;
      rud_in2 = rud_in;
      aux_in2 = aux_in;
    }

    if (ail_mode == AIL_SINGLE)
      ailr_in = ail_in;

    // commanded rate of rotation (could be from ail/ele/rud _in, note direction/sign)
    setpoint[0] = 0;
    setpoint[1] = 0;
    setpoint[2] = 0;

    read_imu();
    gRoll -= gRoll0;
    gPitch -= gPitch0;
    gYaw -= gYaw0;

    // measured rate of rotation (from the gyro)
    input[0] = constrain(gRoll, -8192, 8191);
    input[1] = constrain(gPitch, -8192, 8191);
    input[2] = constrain(gYaw, -8192, 8191);

    compute_pid();

    // stick_gain[] [1100, <ail*|ele|rud>_in_mid, 1900] => [0%, 100%, 0%] = [0, STICK_GAIN_MAX, 0]
    int16_t ail_stick_pos = abs(((ail_in2 - ail_in_mid) + (ailr_in2 - ailr_in_mid)) >> 1);
    stick_gain[0] = STICK_GAIN_MAX - constrain(ail_stick_pos, 0, STICK_GAIN_MAX);
    stick_gain[1] = STICK_GAIN_MAX - constrain(abs(ele_in2 - ele_in_mid), 0, STICK_GAIN_MAX);
    stick_gain[2] = STICK_GAIN_MAX - constrain(abs(rud_in2 - rud_in_mid), 0, STICK_GAIN_MAX);

    // master_gain [1100, 1900] => [0%, 100%] = [0, MASTER_GAIN_MAX]
    master_gain = constrain(aux_in2 - 1100, 0, MASTER_GAIN_MAX); 
    
    for (i=0; i<3; i++) {
      // vr_gain/128, stick_gain/256, master_gain/512
      output[i] = ((((int32_t)output[i] * vr_gain[i] >> 7) * stick_gain[i]) >> 8) * master_gain >> 9;
    }

    // mixer
    int16_t tmp0, tmp1, tmp2;
    switch (mix_mode) {
    case MIX_NORMAL:
      ail_out2 = ail_in2 + output[0];
      ailr_out2 = ailr_in2 + output[0];
      ele_out2 = ele_in2 + output[1];
      rud_out2 = rud_in2 + output[2];
      break;
    case MIX_DELTA:
      tmp0 =  ail_in2 + output[0];
      tmp1 =  ele_in2 + output[1];
      ail_out2 = (tmp0 + tmp1) >> 1;
      ele_out2 = ((tmp0 - tmp1) >> 1) + RX_WIDTH_MID;
      rud_out2 = rud_in2 + output[2];
      break;
    case MIX_VTAIL:
      ail_out2 = ail_in2 + output[0];
      ailr_out2 = ailr_in2 + output[0];
      tmp1 =  ele_in2 + output[1];
      tmp2 =  rud_in2 + output[2];
      ele_out2 = (tmp2 + tmp1) >> 1;
      rud_out2 = ((tmp2 - tmp1) >> 1) + RX_WIDTH_MID;
      break;
    }

    // clamp output
    ail_out2 = constrain(ail_out2, RX_WIDTH_LOW, RX_WIDTH_HIGH);
    ailr_out2 = constrain(ailr_out2, RX_WIDTH_LOW, RX_WIDTH_HIGH);
    ele_out2 = constrain(ele_out2, RX_WIDTH_LOW, RX_WIDTH_HIGH);
    rud_out2 = constrain(rud_out2, RX_WIDTH_LOW, RX_WIDTH_HIGH);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ail_out = ail_out2;
      ailr_out = ailr_out2;
      ele_out = ele_out2;
      rud_out = rud_out2;
    }
    t1 = micros() - t1;
    
#if defined(USE_SERIAL) && 0
    Serial.print(t1); Serial.print('\t');
    Serial.print(kp[0]); Serial.print('\t'); 
    Serial.print(input[0]); Serial.print('\t');
    Serial.print(sum_iterm[0]); Serial.print('\t');
    Serial.print(output[0]); Serial.print('\t');
    Serial.print(output[1]); Serial.print('\t');
    Serial.print(output[2]); Serial.println('\t');
#endif
    last_pid_time = t;
  }
}


