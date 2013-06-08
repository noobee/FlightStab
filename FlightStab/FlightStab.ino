/* FlightStab **************************************************************************************************/

#include <Arduino.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/atomic.h>

#include "FlightStab.h"

bool ow_loop(); // OneWireSerial.ino

// GYRO_ORIENTATION: roll right => -ve, pitch up => -ve, yaw right => -ve

/***************************************************************************************************************
 * device definitions (TODO: move to separate files)
 ***************************************************************************************************************/

//#define RX3S_V1
//#define RX3S_V2
//#define NANOWII
//#define NANO_MPU6050

//#define NO_CPPM // remove cppm code
//#define NO_ONEWIRE // remove one-wire serial config code
//#define NO_STICKCONFIG // remove stick config code

//#define SERIAL_DEBUG // enable debug messages through serial port
//#define DUMP_SENSORS // dump sensors through the serial port
//#define LED_TIMING // disable LED_MSG and use LED_TIMING_START/STOP to measure timings

//#define USE_I2CDEVLIB // interrupt-based wire and i2cdev libraries
//#define USE_I2CLIGHT // poll-based i2c access routines
#if !(defined(USE_I2CDEVLIB) || defined(USE_I2CLIGHT))
#define USE_I2CLIGHT // default
#endif

/* RX3S_V1 *****************************************************************************************************/
#if defined(RX3S_V1)
#warning RX3S_V1 defined // emit device name
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
 DELTA mode
 VTAIL mode
 PD7 7 AUX_IN instead of AILR_OUT
 
 AIL_DUAL mode
 PB3 11 AUX_IN instead of MOSI
 PB4 12 AILR_IN instead of MISO
 
 CPPM enabled
 PB0 8 CPPM_IN instead of AIL_IN
 PB1 9 FLP_OUT instead of ELE_IN
 PB2 10 THR_OUT instead of RUD_IN
*/

#define DEVICE_ID DEVICE_RX3S_V1

// <VR>
#define AIN_PORTC {&ail_vr, &ele_vr, &rud_vr, NULL, NULL, NULL}

// <RX> (must in PORT B/D due to ISR)
#define RX_PORTB {&ail_in, &ele_in, &rud_in, NULL, NULL, NULL, NULL, NULL}
#define RX_PORTD {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}

// <SWITCH>
#define DIN_PORTB {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
#define DIN_PORTC {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
#define DIN_PORTD {NULL, &ail_sw, &ele_sw, &rud_sw, NULL, NULL, NULL, NULL}

// <SERVO>
#define AIL_OUT_PIN 4
#define ELE_OUT_PIN 5
#define RUD_OUT_PIN 6
#define AILR_OUT_PIN 7 // dual aileron mode only

#define PWM_OUT_VAR {&ail_out, &ele_out, &rud_out, &ailr_out, NULL /*&thr_out*/, NULL /*&flp_out*/, NULL, NULL}
#define PWM_OUT_PIN {AIL_OUT_PIN, ELE_OUT_PIN, RUD_OUT_PIN, AILR_OUT_PIN, -1, -1, -1, -1}

// <IMU>
#define USE_ITG3200
#define GYRO_ORIENTATION(x, y, z) {gyro[0] = (y); gyro[1] = (x); gyro[2] = (z);}

// CPPM
#define CPPM_PINREG PINB
#define CPPM_PINBIT 0
#define FLP_OUT_PIN 9
#define THR_OUT_PIN 10

#define F_XTAL F_16MHZ // external crystal oscillator frequency
#define F_I2C F_400KHZ // i2c bus speed
#define SCL_PIN 19
#define SDA_PIN 18

// led register
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_BIT 5
#define LED_XOR 0 // active high

// one-wire port = PD7
#define OW_BIT 7
#define OW_DDR DDRD
#define OW_PORT PORTD
#define OW_PINREG PIND

// eeprom clear pins. shorted on init means to clear eeprom
#define EEPROM_RESET_OUT_PIN 4
#define EEPROM_RESET_IN_PIN 5

#endif 
/* RX3S_V1 *****************************************************************************************************/

/* RX3S_V2 *****************************************************************************************************/
#if defined(RX3S_V2)
#warning RX3S_V2 defined // emit device name
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
 
 CPPM enabled
 PB0 8 CPPM_IN instead of AIL_IN
 PB1 9 FLP_OUT instead of ELE_IN
 PB2 10 THR_OUT instead of RUD_IN
*/

#define DEVICE_ID DEVICE_RX3S_V2V3

// <VR>
#define AIN_PORTC {NULL, &ail_vr, &ele_vr, &rud_vr, NULL, NULL}

// <RX> (must in PORT B/D due to ISR)
#define RX_PORTB {&ail_in, &ele_in, &rud_in, &aux_in, NULL, NULL, NULL, NULL}
#define RX_PORTD {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}

// <SWITCH>
#define DIN_PORTB {NULL, NULL, NULL, NULL, &vtail_sw, NULL, NULL, NULL}
#define DIN_PORTC {&delta_sw, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
#define DIN_PORTD {&aux_sw, &ail_sw, &ele_sw, &rud_sw, NULL, NULL, NULL, NULL}

// <SERVO>
#define AIL_OUT_PIN 4
#define ELE_OUT_PIN 5
#define RUD_OUT_PIN 6
#define AILR_OUT_PIN 7 // dual aileron mode only

#define PWM_OUT_VAR {&ail_out, &ele_out, &rud_out, &ailr_out, NULL /*&thr_out*/, NULL /*&flp_out*/, NULL, NULL}
#define PWM_OUT_PIN {AIL_OUT_PIN, ELE_OUT_PIN, RUD_OUT_PIN, AILR_OUT_PIN, -1, -1, -1, -1}

// <IMU>
#define USE_ITG3200
#define GYRO_ORIENTATION(x, y, z) {gyro[0] = (y); gyro[1] = (x); gyro[2] = (z);}

// CPPM
#define CPPM_PINREG PINB
#define CPPM_PINBIT 0
#define FLP_OUT_PIN 9
#define THR_OUT_PIN 10

#define F_XTAL F_16MHZ // external crystal oscillator frequency
#define F_I2C F_400KHZ // i2c bus speed
#define SCL_PIN 19
#define SDA_PIN 18

// led register
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_BIT 5
#define LED_XOR 0 // active high

// one-wire port = PD7
#define OW_BIT 7
#define OW_DDR DDRD
#define OW_PORT PORTD
#define OW_PINREG PIND

// eeprom clear pins. shorted on init means to clear eeprom
#define EEPROM_RESET_OUT_PIN 4
#define EEPROM_RESET_IN_PIN 5

//#define MOD_PCINT0 // (JohnRB) alternate PCINT0 ISR. only for RX3S_V2
#if defined(MOD_PCINT0)
#warning MOD_PCINT0 defined
#endif

int8_t rx3s_v2_wing_dual_ailB; // true if AIL_DUAL mode B

#endif
/* RX3S_V2 *****************************************************************************************************/

/* NANOWII ****************************************************************************************************/
#if defined(NANOWII)
#warning NANOWII defined // emit device name
/*
 NanoWii
 PB0 17         (PCINT0)            |                              | PD0  3 SCL (SCL)       |                          | PF0 23/A5    (ADC0)
 PB1 15  RUD_IN (SCK/PCINT1)        |                              | PD1  2 SDA (SDA)       |                          | PF1 22/A4    (ADC1)
 PB2 16  AIL_IN (MOSI/PCINT2)       |                              | PD2  0 RXD (RXD)       | PE2               (HWB_) |
 PB3 14  ELE_IN (MISO/PCINT3)       |                              | PD3  1 TXD (TXD)       |                          |
 PB4  8  AUX_IN (PCINT4)            |                              | PD4  4  SV (ADC8/ICP1) |                          | PF4 21/A3 SV (ADC4)
 PB5  9 AIL_OUT (PCINT5/OC1A/OC4B_) |                              | PD5        (TXLED)     |                          | PF5 20/A2 SV (ADC5)
 PB6 10 ELE_OUT (PCINT6/OC1B/OC4B)  | PC6  5        M (OC3A/OC4A_) | PD6 12     (OC4D_)     | PE6 7 THR/AUX2_IN (INT6) | PF6 19/A1 SV (ADC6)
 PB7 11 RUD_OUT (PCINT7/OC0A/OC1C)  | PC7 13 AILR_OUT (OC4A/ICP3)  | PD7  6   M (OC4D)      |                          | PF7 18/A0 SV (ADC7)
 
 CPPM enabled
 PE6 7 CPPM_IN instead of THR/AUX2_IN
 PC6 5 THR_OUT instead of M
 PD7 6 FLP_OUT instead of M
*/

#define DEVICE_ID DEVICE_NANOWII

// <VR> MUST BE ALL NULL
#define AIN_PORTC {NULL, NULL, NULL, NULL, NULL, NULL}

// <RX> PORTD MUST BE ALL NULL
#define RX_PORTB {NULL, &rud_in, &ail_in, &ele_in, &aux_in, NULL, NULL, NULL}
#define RX_PORTD {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}

// <SWITCH> MUST BE ALL NULL
#define DIN_PORTB {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
#define DIN_PORTC {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
#define DIN_PORTD {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}

// <SERVO>
#define AIL_OUT_PIN 9
#define ELE_OUT_PIN 10
#define RUD_OUT_PIN 11
#define AILR_OUT_PIN 13 // dual aileron mode only

#define PWM_OUT_VAR {&ail_out, &ele_out, &rud_out, &ailr_out, NULL /*&thr_out*/, NULL /*&flp_out*/, NULL, NULL}
#define PWM_OUT_PIN {AIL_OUT_PIN, ELE_OUT_PIN, RUD_OUT_PIN, AILR_OUT_PIN, -1, -1, -1, -1}

// <IMU>
#define USE_MPU6050
#define GYRO_ORIENTATION(x, y, z) {gyro[0] = (-y); gyro[1] = (-x); gyro[2] = (z);}

// CPPM
#define CPPM_PINREG PINE
#define CPPM_PINBIT 6
#define THR_OUT_PIN 5
#define FLP_OUT_PIN 6

#define F_XTAL F_16MHZ // external crystal oscillator frequency
#define F_I2C F_400KHZ // i2c bus speed
#define SCL_PIN 3
#define SDA_PIN 2

// led register
#define LED_DDR DDRD
#define LED_PORT PORTD
#define LED_BIT 5
#define LED_XOR 1 // active low

// one-wire port = PC7
#define OW_BIT 7
#define OW_DDR DDRC
#define OW_PORT PORTC
#define OW_PINREG PINC

// eeprom clear pins. shorted on init means to clear eeprom
#define EEPROM_RESET_OUT_PIN 6 // also used for flap_out in cppm mode
#define EEPROM_RESET_IN_PIN 5 // also used for thr_out in cppm mode

#endif
/* NANOWII ****************************************************************************************************/


/* NANO_MPU6050 ************************************************************************************************/
#if defined(NANO_MPU6050)
#warning NANO_MPU6050 defined // emit device name
#undef USE_ITG3200
#define USE_MPU6050
#undef GYRO_ORIENTATION
#define GYRO_ORIENTATION(x, y, z) {gyro[0] = -(x); gyro[1] = (y); gyro[2] = (z);}
#undef F_I2C
#define F_I2C F_100KHZ // i2c bus speed
#endif
/* NANO_MPU6050 ************************************************************************************************/

// standard frequency definitions
#define F_8MHZ 8000000UL
#define F_16MHZ 16000000UL
#define F_100KHZ 100000UL
#define F_400KHZ 400000UL

// emit cpu frequency if not 16MHz
#if F_CPU == F_8MHZ
#warning F_CPU == F_8MHZ
#endif
#if !((F_XTAL == F_16MHZ && F_CPU == F_16MHZ) || \
      (F_XTAL == F_16MHZ && F_CPU == F_8MHZ) || \
      (F_XTAL == F_8MHZ && F_CPU == F_8MHZ))
#error (F_XTAL,F_CPU) must be (16MHz,16MHz), (16MHz,8MHz) or (8MHz,8MHz)
#endif

// emit serial debug enabled
#if defined(SERIAL_DEBUG)
#warning SERIAL_DEBUG defined
#endif

// verify single i2c lib defined
#if defined(USE_I2CDEVLIB) && defined(USE_I2CLIGHT)
#error Cannot define both USE_I2CDEVLIB and USE_I2CLIGHT
#endif

// verify single imu defined
#if defined(USE_MPU6050) && defined(USE_ITG3200)
#error Cannot define both USE_MPU6050 and USE_ITG3200
#endif

// i2cdevlib includes
#if defined(USE_I2CDEVLIB)
#warning USE_I2CDEVLIB defined
  #include "Wire.h"
  #include "I2Cdev.h"
#if defined(USE_MPU6050)
  #include "MPU6050.h"
  MPU6050 accelgyro;
#elif defined(USE_ITG3200)
  #include "ITG3200.h"
  ITG3200 itg3205_gyro;
#endif
#endif

// device id and version
#if !defined(DEVICE_ID)
#define DEVICE_ID DEVICE_UNDEF
#endif
#if !defined(DEVICE_VER)
#define DEVICE_VER 12345678L
#endif


// LED set
#define LED_OFF 0
#define LED_ON 1
#define LED_INVERT 2

// LED message pulse duration
#define LED_LONG 600
#define LED_SHORT 200
#define LED_VERY_SHORT 30

#if defined(LED_TIMING)
#define LED_TIMING_START do {LED_PORT |= (1 << LED_BIT);} while (false)
#define LED_TIMING_STOP do {LED_PORT &= ~(1 << LED_BIT);} while (false)
#endif

// adc
volatile uint8_t ail_vr; // initialized with vr_notch_table[vr_notch[0]] = 255
volatile uint8_t ele_vr; //
volatile uint8_t rud_vr; //

 // for eeprom config
int8_t vr_notch_table[4+1+4] = {0, 32, 64, 96, 128, 160, 192, 224, 255};
int8_t vr_notch[3] = {4, 4, 4};

// rx
#define RX_WIDTH_MIN 900
#define RX_WIDTH_LOW_FULL 1000
#define RX_WIDTH_LOW_NORM 1100
#define RX_WIDTH_LOW_TRACK 1250
#define RX_WIDTH_MID 1500
#define RX_WIDTH_HIGH_TRACK 1750
#define RX_WIDTH_HIGH_NORM 1900
#define RX_WIDTH_HIGH_FULL 2000
#define RX_WIDTH_MAX 2100

volatile int16_t ail_in = RX_WIDTH_MID;
volatile int16_t ele_in = RX_WIDTH_MID;
volatile int16_t rud_in = RX_WIDTH_MID;
volatile int16_t ailr_in = RX_WIDTH_MID;
volatile int16_t aux_in = RX_WIDTH_LOW_FULL; // assume max RATE MODE gain if no aux_in
volatile int16_t aux2_in = RX_WIDTH_MID;
volatile int16_t thr_in = RX_WIDTH_LOW_FULL;
volatile int16_t flp_in = RX_WIDTH_MID;
volatile int16_t tmp_in = RX_WIDTH_MID;
int16_t ail_in2, ailr_in2, ele_in2, rud_in2, aux_in2, aux2_in2, thr_in2, flp_in2;

int16_t ail_in2_mid = RX_WIDTH_MID; // calibration sets stick-neutral-position offsets
int16_t ele_in2_mid = RX_WIDTH_MID; //
int16_t rud_in2_mid = RX_WIDTH_MID; //
int16_t ailr_in2_mid = RX_WIDTH_MID; //

// switch
int8_t ail_sw = false;
int8_t ele_sw = false;
int8_t rud_sw = false;
int8_t vtail_sw = false;
int8_t delta_sw = false;
int8_t aux_sw = false;

// servo
volatile int16_t ail_out;
volatile int16_t ele_out;
volatile int16_t rud_out;
volatile int16_t ailr_out;
volatile int16_t thr_out;
volatile int16_t flp_out;
int16_t ail_out2, ele_out2, rud_out2, ailr_out2, thr_out2, flp_out2;

int16_t mixer_out2_low_limit[4];
int16_t mixer_out2_high_limit[4];

// imu
int16_t gyro0[3] = {0, 0, 0}; // calibration sets zero-movement-measurement offsets
int16_t gyro[3] = {0, 0, 0}; // full scale = 16b-signed = +/-2000 deg/sec
int16_t accel[3] = {0, 0, 0}; // full scale = 16b-signed = 16g? (TODO)

int16_t correction[3] = {0, 0, 0}; // final correction values

// wing mode
enum WING_MODE wing_mode = WING_SINGLE_AIL;
enum MIXER_EPA_MODE mixer_epa_mode = MIXER_EPA_FULL;
enum CPPM_MODE cppm_mode = CPPM_NONE;
enum MOUNT_ORIENT mount_orient = MOUNT_NORMAL;
enum HOLD_AXES hold_axes = HOLD_AXES_AE_R;

const int8_t rx_chan_list_size = 8;
volatile int16_t *rx_chan[][rx_chan_list_size] = {
  {&rud_in, &ele_in, &thr_in, &ail_in, &aux_in, &ailr_in, &aux2_in, &flp_in}, // CPPM_RETA1a2F (FrSky)
  {&thr_in, &ail_in, &ele_in, &rud_in, &aux_in, &ailr_in, &aux2_in, &flp_in}, // CPPM_TAERF1a2F (JR/Spektrum)
  {&ail_in, &ele_in, &thr_in, &rud_in, &aux_in, &ailr_in, &aux2_in, &flp_in} // CPPM_AETR1a2F (Futaba)
};

// stabilization mode
enum STAB_MODE {STAB_UNDEF, STAB_RATE, STAB_HOLD};
enum STAB_MODE stab_mode = STAB_UNDEF; // undefined

#define VR_GAIN_MAX 127
#define STICK_GAIN_MAX 400 // [1100-1500] or [1900-1500] => [0-STICK_GAIN_MAX]
#define MASTER_GAIN_MAX 400 // [1500-1100] or [1500-1900] => [0-MASTER_GAIN_MAX]

/***************************************************************************************************************
 * TIMER1 AND MISC
 ***************************************************************************************************************/

volatile uint16_t timer1_high = 0;
volatile uint8_t timer1_ovf = 0;

ISR(TIMER1_OVF_vect)
{
  if (!++timer1_high)
    timer1_ovf++;
}

uint32_t micros1()
{
  uint16_t th, tl;
  uint8_t to;
#if 0
  // interrupt-disable version
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    tl = TCNT1; // tick=0.5us if F_CPU=16M, tick=1.0us if F_CPU=8M 
    th = timer1_high;
    to = timer1_ovf;
  }
#else
  // lock-free version. read twice, ensure low order counter did not overflow and 
  // high order bits remain unchanged
  while (true) {
    tl = TCNT1; // tick=0.5us if F_CPU=16M, tick=1.0us if F_CPU=8M 
    th = timer1_high;
    to = timer1_ovf;
    if (!((tl ^ TCNT1) & 0x8000) && th == timer1_high && to == timer1_ovf)
      break;    
  }  
#endif
#if F_CPU == F_16MHZ
  return ((uint32_t)to << 31) | ((uint32_t)th << 15) | (tl >> 1);
#else
  return ((uint32_t)th << 16) | tl;
#endif
}

void delay1(uint32_t ms)
{
  uint32_t us = ms * 1000; 
  uint32_t t = micros1();
  while ((int32_t)(micros1() - t) < us);
}

void init_clock()
{
  // set clock prescaler to /2 when (F_XTAL, F_CPU) = (16M, 8M) 
#if F_XTAL == F_16MHZ && F_CPU == F_8MHZ
  uint8_t saveSREG = SREG;
  cli();
  CLKPR = (1 << CLKPCE);
  CLKPR = (1 << CLKPS0);
  SREG = saveSREG;
#endif
}

int get_free_sram() 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval); 
}

bool boot_check(int8_t in_pin, int8_t out_pin)
{
  pinMode(in_pin, INPUT);
  pinMode(out_pin, OUTPUT);

  bool result = true;
  for (int8_t i=0; i<8; i++) {
    int8_t level = i & 1 ? HIGH : LOW;
    digitalWrite(out_pin, level);
    if (digitalRead(in_pin) != level) {
      result = false;
      break;
    }
  }
  pinMode(out_pin, INPUT);
  return result;
}


/***************************************************************************************************************
 * LED
 ***************************************************************************************************************/
/*
  4 LED message slots
  slot 0 for operation mode
  slot 1 for operation mode
  slot 2 for error condition
  slot 3 for error condition

  slot 0
  1 LONG = MIX_NORMAL
  2 LONG = MIX_DELTA
  3 LONG = MIX_VTAIL

  slot 1
  1 SHORT = rx calibrating
  2 SHORT = imu calibrating
  3 SHORT = both calibrating
  4 SHORT = STAB_HOLD mode (else STAB_RATE)

  slot 2
  5 SHORT = gyro init error
  20 VERY SHORT = low sram
  
  slot 3
  50 VERY SHORT = eeprom reset
  
*/

int8_t led_pulse_count[4];
int16_t led_pulse_msec[4];

void init_led()
{
  LED_DDR |= (1 << LED_BIT);
}

void set_led(int8_t i)
{
#if !defined(LED_TIMING)
  switch (i ^ LED_XOR) {
  case LED_ON: LED_PORT |= (1 << LED_BIT); break;
  case LED_OFF: LED_PORT &= ~(1 << LED_BIT); break;
  default: LED_PORT ^= (1 << LED_BIT); break;
  }  
#endif
}

void set_led_msg(int8_t slot, int8_t pulse_count, int16_t pulse_msec) {
  led_pulse_count[slot] = pulse_count;
  led_pulse_msec[slot] = pulse_msec;
}

void update_led(uint32_t now)
{
  static int8_t slot;
  static int8_t step;
  static uint32_t next_time;
  
  if ((int32_t)(next_time - now) > 0)
    return;

  if (step == 0) {
    slot = (slot + 1) & 0x3;
    step = led_pulse_count[slot] << 1;
    next_time = now + 1000000;
    return;
  }

  step--;
  set_led(step & 0x1 ? LED_ON : LED_OFF);
  next_time = now + ((uint32_t)led_pulse_msec[slot] << 10);
}

/***************************************************************************************************************
 * I2CLIGHT / MPU6050 / ITG3205
 ***************************************************************************************************************/

uint16_t i2c_errors = 0;

void i2c_init(int8_t pullup, uint32_t freq)
{
  if (pullup) {
    // scl and sda
    digitalWrite(SCL_PIN, HIGH);
    digitalWrite(SDA_PIN, HIGH);
  }
  TWSR = 0; // prescaler = 1
  TWBR = ((F_CPU / freq) - 16) >> 1; // baud rate 
}

void i2c_wait() 
{
  int16_t timeout = 255;
  while (!(TWCR & (1 << TWINT))) {
    if (!timeout--) {
      TWCR = 0; // disable twi
      i2c_errors++;
      break;
    }
  }
}

void i2c_start(uint8_t addr)
{
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // repeated start
  i2c_wait();
  TWDR = addr;
  TWCR = (1 << TWINT) | (1 << TWEN);
  i2c_wait();
}

void i2c_stop()
{
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void i2c_write(uint8_t data)
{
  TWDR = data;
  TWCR = (1 << TWINT) | (1 << TWEN);
  i2c_wait();
}

uint8_t i2c_read(int8_t ack)
{
  uint8_t data;
  TWCR = (1 << TWINT) | (1 << TWEN) | (ack ? (1 << TWEA) : 0);
  i2c_wait();
  data = TWDR;
  if (!ack) i2c_stop();
  return data;
}

void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t data)
{
  i2c_start(addr << 1); // write reg
  i2c_write(reg);
  i2c_write(data); // write data
  i2c_stop();
}

uint8_t i2c_read_reg(uint8_t addr, uint8_t reg)
{
  i2c_start(addr << 1); // write reg
  i2c_write(reg);
  i2c_start((addr << 1) | 1); // read data
  return i2c_read(true);
}

void i2c_read_buf(uint8_t addr, uint8_t *buf, int8_t size)
{
  i2c_start((addr << 1) | 1); // read data
  while (size--) {
    *buf++ = i2c_read(size > 0);
  }
}

void i2c_read_buf_reg(uint8_t addr, uint8_t reg, uint8_t *buf, int8_t size)
{
  i2c_start(addr << 1); // write reg
  i2c_write(reg);
  i2c_read_buf(addr, buf, size);
}

#define MPU6050_ADDR 0x68

int8_t mpu6050_init()
{
  i2c_write_reg(MPU6050_ADDR, 0x6B, 0x80); // reset
  delay1(50);
  i2c_write_reg(MPU6050_ADDR, 0x6B, 0x03); // clock=pll with z-gyro ref
  i2c_write_reg(MPU6050_ADDR, 0x1A, 0x0); // accel_bw/delay=260hz/0ms gyro_bw/delay/sample=256hz/0.98ms/8khz
  //i2c_write_reg(MPU6050_ADDR, 0x1A, 0x6); // accel_bw/delay=5hz/19ms gyro_bw/delay/sample=5hz/18.6ms/1khz
  i2c_write_reg(MPU6050_ADDR, 0x1B, 0x18); // fs=2000deg/s

  return ((i2c_read_reg(MPU6050_ADDR, 0x75) & 0x7E) >> 1) == 0x34; // chip id
}

void mpu6050_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t buf[6];
  i2c_read_buf_reg(MPU6050_ADDR, 0x43, buf, 6);
  *gx = (buf[0] << 8) | (buf[1]);
  *gy = (buf[2] << 8) | (buf[3]);
  *gz = (buf[4] << 8) | (buf[5]);
}

void mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az)
{
  uint8_t buf[6];
  i2c_read_buf_reg(MPU6050_ADDR, 0x3B, buf, 6);
  *ax = (buf[0] << 8) | (buf[1]);
  *ay = (buf[2] << 8) | (buf[3]);
  *az = (buf[4] << 8) | (buf[5]);
}

#define ITG3205_ADDR 0x68

int8_t itg3205_init()
{
  i2c_write_reg(ITG3205_ADDR, 0x3E, 0x80); // reset
  delay1(50);
  i2c_write_reg(ITG3205_ADDR, 0x3E, 0x03); // clock=pll with z-gyro ref
  i2c_write_reg(ITG3205_ADDR, 0x16, 0x18 | 0x6); // fs=2000deg/s | lpf=5hz sample=1khz

  return ((i2c_read_reg(ITG3205_ADDR, 0x00) & 0x7E) >> 1) == 0x34; // chip id
}

void itg3205_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t buf[6];
  i2c_read_buf_reg(ITG3205_ADDR, 0x1D, buf, 6);
  *gx = (buf[0] << 8) | (buf[1]);
  *gy = (buf[2] << 8) | (buf[3]);
  *gz = (buf[4] << 8) | (buf[5]);
}

/***************************************************************************************************************
 * ANALOG IN (VR)
 ***************************************************************************************************************/

volatile uint8_t *adc_portc[] = AIN_PORTC;

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
  int8_t i;
  for (i=0; i<6; i++) {
    if (adc_portc[i]) {
      DDRC &= ~(1 << i); // set to input mode
      PORTC &= ~(1 << i); // do not enable internal pullup
    }
  }

  ADMUX = (1 << REFS0) | (1 << ADLAR); // acc vref, adc left adjust, adc channel 0
  ADCSRB = 0;
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (0x07); // adc enable, manual trigger, interrupt enable, prescaler 128
  start_next_adc(0); 
}

/***************************************************************************************************************
 * DIGITAL IN (DIP SW)
 ***************************************************************************************************************/

int8_t *din_portb[] = DIN_PORTB;
int8_t *din_portc[] = DIN_PORTC;
int8_t *din_portd[] = DIN_PORTD;

void init_digital_in_port_list(int8_t **pport_list, volatile uint8_t *pDDR, volatile uint8_t *pPORT)
{
  for (int8_t i=0; i<8; i++) {
    if (pport_list[i]) {
      *pDDR &= ~(1 << i); // set to input mode
      *pPORT |= (1 << i); // enable internal pullup
    }
  }    
}

void init_digital_in_sw()
{
  init_digital_in_port_list(din_portb, &DDRB, &PORTB);
  init_digital_in_port_list(din_portc, &DDRC, &PORTC);
  init_digital_in_port_list(din_portd, &DDRD, &PORTD);
}

void read_switches()
{
  for (int8_t i=0; i<8; i++) {
    if (din_portb[i])
     *din_portb[i] = PINB & (1 << i);
    if (din_portc[i])
     *din_portc[i] = PINC & (1 << i);
    if (din_portd[i])
     *din_portd[i] = PIND & (1 << i);
  } 
}


/***************************************************************************************************************
 * DIGITAL IN (RX)
 ***************************************************************************************************************/

volatile int8_t rx_frame_sync; // true if rx_frame_sync_ref pulse has occurred
int8_t rx_frame_sync_ref; // PB<n> bit for non-CPPM, rx_chan[cppm_mode-2][<n>] var for CPPM
// non cppm mode
volatile int16_t *rx_portb[] = RX_PORTB;
volatile int16_t *rx_portd[] = RX_PORTD;

// PORTB PCINT0-PCINT7
inline void pcint0_vect()
{
  static uint16_t rise_time[8];
  static uint8_t last_pin;
  uint16_t now;
  uint8_t pin, last_pin2, diff, rise;

  now = TCNT1; // tick=0.5us if F_CPU=16M, tick=1.0us if F_CPU=8M 
  last_pin2 = last_pin;
  pin = PINB;
  last_pin = pin;
  sei();

  diff = pin ^ last_pin2;
  rise = pin & ~last_pin2;

  for (int8_t i = PCINT0; i <= PCINT7; i++) {
    if (rx_portb[i] && diff & (1 << i)) {
      if (rise & (1 << i)) {
        rise_time[i] = now;
      } else {
        uint16_t width = (now - rise_time[i]) >> (F_CPU == F_16MHZ ? 1 : 0);
        if (width >= RX_WIDTH_MIN && width <= RX_WIDTH_MAX) {
          *rx_portb[i] = width;
          if (i == rx_frame_sync_ref)
            rx_frame_sync = true;
        }
      }
    }
  }
}

#if defined(RX3S_V1) || defined(RX3S_V2)
// isr armed only if cppm enabled
ISR(TIMER1_CAPT_vect)
{
#if !defined(NO_CPPM)
  static int8_t ch0_synced = false;
  static uint16_t rise_time;
  static uint8_t ch;
  uint16_t now;
  uint16_t width;

  now = ICR1; // tick=0.5us if F_CPU=16M, tick=1.0us if F_CPU=8M 
  sei();

  width = (now - rise_time) >> (F_CPU == F_16MHZ ? 1 : 0);
  rise_time = now;
  if (width > 3000) {
    rx_frame_sync_ref = ch - 1;
    ch = 0;
    ch0_synced = true;
  } else if (ch0_synced && ch < rx_chan_list_size && width >= RX_WIDTH_MIN && width <= RX_WIDTH_MAX) {
    *rx_chan[cppm_mode-2][ch] = width;
    if (ch == rx_frame_sync_ref)
      rx_frame_sync = true;
    ch++;
  }
#endif // !NO_CPPM
}

#if !(defined(RX3S_V2) && defined(MOD_PCINT0))
ISR(PCINT0_vect) 
{
  pcint0_vect();
}
#else // !(defined(RX3S_V2) && defined(MOD_PCINT0))

// contributed by JohnRB
//*********************************************************************************/
//  Excessive delay in processing interrupts for port B bit changes causes        */
//  inaccurate pulse width measurements which may result in jitter seen/heard on  */
//  the attached servos.                                                          */
//                                                                                */
//  This version of the interrupt handler was an attempt to get the fastest       */
//  possible execution time with the minimal use of stack space.  This ISR does   */
//  not enable interrupts again until it completes (about 6.35us at 16MHz).	      */
//                                                                                */
//  The downside of using this option is increased use of Flash memory (about 150 */
//  bytes).	                                                                      */
//	                                                                              */
//  The original ISR is reentrant and runs disabled for about 1.6us less than this*/
//  version but takes about 23.5us to complete (at 16MHz).  The original is also  */
//  reentrant and each new invocation requires an additonal 25 bytes of stack     */
//  space.	                                                                      */
//	                                                                              */
//  Bottom Line - If stack space becomes critical and/or processor is starting to */
//  get overloaded, use this replacement ISR, otherwise just stick with the       */
//  original ISR.	                                                                */
//*********************************************************************************/
// 
// this ISR uses a fixed map of (AIL, ELE, RUD, AUX)_IN_PINs assigned to PB(0,1,2,3)

#define    AIL_PORT_BIT  0
#define    ELE_PORT_BIT  1
#define    RUD_PORT_BIT  2
#define    AUX_PORT_BIT  3	// Also AILR_PORT_BIT in DUAL AILERON mode

#define CHECK_CHANNEL(PORT_BIT, RISE_VAR, PORT_VAR) \
  if (diff & (1 << PORT_BIT))	{ \
    if (rise & (1 << PORT_BIT)) { \
      RISE_VAR = now; \
    } else { \
      width = (now - RISE_VAR) >> (F_CPU == F_16MHZ ? 1 : 0); \
      if (width >= RX_WIDTH_MIN && width <= RX_WIDTH_MAX) { \
        PORT_VAR = width; \
        if (rx_frame_sync_ref == PORT_BIT) \
          rx_frame_sync = true; \
      } \
    } \
  }

ISR(PCINT0_vect)
{
  static uint16_t ail_rise, ele_rise, rud_rise, aux_rise;
  static uint8_t last_pin;

  uint16_t now;        // current timer value
  uint8_t rise;	       // rising edge pulses
  uint8_t diff;        // portb changed bits     
  uint8_t last_pin2;   // temporary save for previous Port B value
  uint16_t width;      // work register to verify width

  now = TCNT1;         
  last_pin2 = last_pin;  // save previous Port B value
  last_pin = PINB;

  diff = last_pin ^ last_pin2;	// pins that just changed state
  rise = last_pin & ~last_pin2;	// pins that just went positive (start of pulse)

  // The following tests each input for change, 
  // at rise captures time and at fall calculates and validates the width & saves it.
  // If the channel is the reference port, indicate sync
  
  CHECK_CHANNEL(AIL_PORT_BIT, ail_rise, ail_in);
  CHECK_CHANNEL(ELE_PORT_BIT, ele_rise, ele_in);
  CHECK_CHANNEL(RUD_PORT_BIT, rud_rise, rud_in);
  // In DUAL AILERON mode A, ailr_in replaces aux_in and there is no gain control
  if (wing_mode == WING_DUAL_AIL && !rx3s_v2_wing_dual_ailB) {
    CHECK_CHANNEL(AUX_PORT_BIT, aux_rise, ailr_in); // DUAL AILERON mode A
  } else {
    CHECK_CHANNEL(AUX_PORT_BIT, aux_rise, aux_in); // All other modes 
  }
}
#endif // !(defined(RX3S_V2) && defined(MOD_PCINT0))

// PORTD PCINT16-PCINT23
ISR(PCINT2_vect)
{
  static uint16_t rise_time[8];
  static uint8_t last_pin;
  uint16_t now;
  uint8_t pin, last_pin2, diff, rise;

  now = TCNT1; // tick=0.5us if F_CPU=16M, tick=1.0us if F_CPU=8M 
  last_pin2 = last_pin;
  pin = PIND;
  last_pin = pin;
  sei();

  diff = pin ^ last_pin2;
  rise = pin & ~last_pin2;

  for (int8_t i = 0; i < 8; i++) {
    if (rx_portd[i] && diff & (1 << i)) {
      if (rise & (1 << i)) {
        rise_time[i] = now;
      } else {
        uint16_t width = (now - rise_time[i]) >> (F_CPU == F_16MHZ ? 1 : 0);
        if (width >= RX_WIDTH_MIN && width <= RX_WIDTH_MAX) {
          *rx_portd[i] = width;
        }
      }
    }
  }
}

#endif // defined(RX3S_V1) || defined(RX3S_V2)

#if defined(NANOWII)
// PE6 = aux2_in
inline void int6_vect_non_cppm() 
{
  static uint16_t rise_time;
  static uint8_t last_pin;
  uint16_t now;
  uint8_t pin, last_pin2, rise;

  now = TCNT1; // tick=0.5us if F_CPU=16M, tick=1.0us if F_CPU=8M 
  last_pin2 = last_pin;
  pin = PINE;
  last_pin = pin;
  sei();

  rise = pin & ~last_pin2;
  if (rise & (1 << 6)) {
    rise_time = now;
  } else {
    uint16_t width = (now - rise_time) >> (F_CPU == F_16MHZ ? 1 : 0);
    if (width >= RX_WIDTH_MIN && width <= RX_WIDTH_MAX) {
      aux2_in = width;
    }
  }
}

// PE6 = cppm_in
inline void int6_vect_cppm() 
{
#if !defined(NO_CPPM)
  static int8_t ch0_synced = false;
  static uint16_t rise_time;
  static uint8_t last_pin;
  static uint8_t ch;
  uint16_t now;
  uint8_t pin, last_pin2, rise;

  now = TCNT1; // tick=0.5us if F_CPU=16M, tick=1.0us if F_CPU=8M 
  last_pin2 = last_pin;
  pin = CPPM_PINREG;
  last_pin = pin;
  sei();

  rise = pin & ~last_pin2;

  if (rise & (1 << CPPM_PINBIT)) {
    uint16_t width = (now - rise_time) >> (F_CPU == F_16MHZ ? 1 : 0);
    rise_time = now;
    if (width > 3000) {
      rx_frame_sync_ref = ch - 1;
      ch = 0;
      ch0_synced = true;
    } else if (ch0_synced && ch < rx_chan_list_size && width >= RX_WIDTH_MIN && width <= RX_WIDTH_MAX) {
      *rx_chan[cppm_mode-2][ch] = width;
      if (ch == rx_frame_sync_ref)
        rx_frame_sync = true;
      ch++;
    }
  }
#endif // !NO_CPPM
}

ISR(INT6_vect) 
{ 
  if (cppm_mode != CPPM_NONE)
    int6_vect_cppm();
  else
    int6_vect_non_cppm();
}
 
ISR(PCINT0_vect) 
{
  pcint0_vect();
}
#endif // NANOWII


void init_digital_in_rx()
{
#if !defined(NO_CPPM)
  if (cppm_mode != CPPM_NONE) {
#if defined(NANOWII)
    // (CPPM_PINREG, CPPM_PINBIT) MUST be (PINE, 6)
    EICRB |= (1 << ISC60); // interrupt on pin change
    EIMSK |= (1 << INT6);
    DDRE &= ~(1 << CPPM_PINBIT);
    PORTE |= (1 << CPPM_PINBIT);
#else // NANOWII
    // (CPPM_PINREG, CPPM_PINBIT) MUST be (PINB, 0)  
    TIMSK1 |= (1 << ICIE1); // enable interrupt on ICP
    TCCR1B |= (1 << ICNC1) | (1 << ICES1); // enable noise canceler and interrupt on rising edge
#endif // NANOWII
    rx_frame_sync_ref = 3; // sync on rx_chan[][3] first, but isr will track the sync gap
    return;
  }
#endif // !NO_CPPM

  // PORTB RX
  PCICR |= (1 << PCIE0); // interrupt on pin change
  for (int8_t i=0; i<8; i++) {
    if (rx_portb[i]) {
      PCMSK0 |= 1 << (PCINT0 + i);
      DDRB &= ~(1 << i); // set input mode
      PORTB |= (1 << i); // enable internal pullup
    }
  }
#if defined(NANOWII)
  rx_frame_sync_ref = 3; // use ELE_IN (PB3) as ref channel. TODO: fix this hardcoding
#else
  rx_frame_sync_ref = 1; // use ELE_IN (PB1) as ref channel. TODO: fix this hardcoding
#endif

#if defined(NANOWII)
  // PE6
  EICRB |= (1 << ISC60); // interrupt on pin change
  EIMSK |= (1 << INT6);
  DDRE &= ~(1 << 6);
  PORTE |= (1 << 6);
#else // NANOWII
  // PORTD RX
  PCICR |= (1 << PCIE2); // interrupt on pin change
  for (int8_t i=0; i<8; i++) {
    if (rx_portd[i]) {
      PCMSK2 |= 1 << i;
      DDRD &= ~(1 << i); // set to input mode
      PORTD |= (1 << i); // enable internal pullup
    }
  }
#endif // NANOWII
}

/***************************************************************************************************************
 * DIGITAL OUT (SERVO)
 ***************************************************************************************************************/

volatile int16_t *pwm_out_var[] = PWM_OUT_VAR;
volatile int8_t pwm_out_pin[] = PWM_OUT_PIN;
volatile int8_t servo_busy = false;

ISR(TIMER1_COMPA_vect)
{
  static int8_t fall_ch = 0;
  static int8_t rise_ch = 0;
  uint16_t wait;
  uint16_t tcnt1;
 
  digitalWrite(pwm_out_pin[fall_ch], LOW);
  if (rise_ch >= 0) {
    tcnt1 = TCNT1;
    digitalWrite(pwm_out_pin[rise_ch], HIGH);
    wait = (*pwm_out_var[rise_ch] - 2) << (F_CPU == F_16MHZ ? 1 : 0);
    fall_ch = rise_ch;
    // two consecutive null entries to indicate end of list
    if (!pwm_out_var[++rise_ch] && !pwm_out_var[++rise_ch]) {
      rise_ch = -1;
    }
    OCR1A = tcnt1 + wait;
  } else {
    rise_ch = 0;
    servo_busy = false;
    TIMSK1 &= ~(1 << OCIE1A); // disable further interrupt on TCNT1 == OCR1A
  }
}

void init_digital_out()
{
  int8_t i = -1;
  while (true) {
    // two consecutive null entries to indicate end of list
    if (!pwm_out_var[++i] && !pwm_out_var[++i])
      break;
    pinMode(pwm_out_pin[i], OUTPUT);
  }
} 

/***************************************************************************************************************
 * PID
 ***************************************************************************************************************/

#define PID_PERIOD 10000
// relative weights kp:ki:kd
#define PID_KP_SHIFT 3 
#define PID_KI_SHIFT 6
#define PID_KD_SHIFT 2

struct _pid_param {
  int16_t kp[3]; // [0, 1000] 11b signed
  int16_t ki[3];
  int16_t kd[3];
  int32_t i_limit[3];
  int8_t output_shift;
};

struct _pid_state {
  // setpoint, input [-8192, 8191]
  int16_t setpoint[3];
  int16_t input[3];
  int16_t last_err[3];
  int32_t sum_err[3];
  int16_t output[3];
};

struct _pid_param pid_param_rate; // rate mode pid parameters
struct _pid_param pid_param_hold; // hold mode pid parameters
struct _pid_state pid_state; // pid engine state

void compute_pid(struct _pid_state *ppid_state, struct _pid_param *ppid_param) 
{
  for (int8_t i=0; i<3; i++) {
    int32_t err, sum_err, diff_err, pterm, iterm, dterm;
    err = ppid_state->input[i] - ppid_state->setpoint[i];
    // accumulate the error up to an i_limit threshold
    sum_err = ppid_state->sum_err[i] = constrain(ppid_state->sum_err[i] + err, -ppid_param->i_limit[i], ppid_param->i_limit[i]);
    diff_err = err - ppid_state->last_err[i]; // difference the error
    ppid_state->last_err[i] = err;    
        
    pterm = (ppid_param->kp[i] * err) >> PID_KP_SHIFT;
    iterm = (ppid_param->ki[i] * sum_err) >> PID_KI_SHIFT;
    dterm = (ppid_param->kd[i] * diff_err) >> PID_KD_SHIFT;
    ppid_state->output[i] = (pterm + iterm + dterm) >> ppid_param->output_shift;

#if defined(SERIAL_DEBUG) && 0
    if (i == 2) {
      Serial.print(ppid_state->input[i]); Serial.print('\t');
      Serial.print(err); Serial.print('\t');
      Serial.print(diff_err); Serial.print('\t');
      Serial.print(pterm); Serial.print('\t');
      Serial.print(iterm); Serial.print('\t');
      Serial.print(ppid_state->sum_err[i]); Serial.print('\t');
      Serial.print(dterm); Serial.print('\t');
      Serial.println(ppid_state->output[i]);
    }
#endif
  }
}

/***************************************************************************************************************
 * EEPROM
 ***************************************************************************************************************/

const uint16_t eeprom_stats_addr = 128;
const uint16_t eeprom_cfg1_addr = 0;
const uint16_t eeprom_cfg2_addr = 256+0;

uint8_t eeprom_compute_chksum(void *buf, int8_t len) 
{
  uint8_t chksum = 0;
  for (int8_t i=0; i<len; i++)
    chksum += ((uint8_t *)buf)[i];
  return (chksum ^ 0xff) + 1;
}

void eeprom_write_cfg(struct _eeprom_cfg *pcfg, uint16_t eeprom_addr) 
{    
  pcfg->chksum = eeprom_compute_chksum(pcfg, sizeof(*pcfg)-1);
  eeprom_write_block(pcfg, (void *)eeprom_addr, sizeof(*pcfg));   
}

int8_t eeprom_read_cfg(struct _eeprom_cfg *pcfg, uint16_t eeprom_addr, uint8_t expect_ver) 
{
  eeprom_read_block(pcfg, (void *)eeprom_addr, sizeof(*pcfg));
  if (pcfg->ver != expect_ver) {
    return -1;
  }
  if (eeprom_compute_chksum(pcfg, sizeof(*pcfg)) != 0) {
    return -2;
  }
  return 0;
}

void eeprom_copy_to_cfg(struct _eeprom_cfg *pcfg)
{
  int8_t i;        
  pcfg->wing_mode = wing_mode;
  for (i=0; i<3; i++) {   
    pcfg->vr_notch[i] = vr_notch[i];
  }
  pcfg->mixer_epa_mode = mixer_epa_mode;
  pcfg->cppm_mode = cppm_mode;
  pcfg->mount_orient = mount_orient;
  pcfg->hold_axes = hold_axes;
}

void eeprom_copy_from_cfg(struct _eeprom_cfg *pcfg)
{
  int8_t i;
  wing_mode = pcfg->wing_mode;
  for (i=0; i<3; i++) {   
    vr_notch[i] = pcfg->vr_notch[i];
  }
  ail_vr = vr_notch_table[vr_notch[0] + 4];
  ele_vr = vr_notch_table[vr_notch[1] + 4];
  rud_vr = vr_notch_table[vr_notch[2] + 4];
  mixer_epa_mode = pcfg->mixer_epa_mode;
  cppm_mode = pcfg->cppm_mode;
  mount_orient = pcfg->mount_orient;
  hold_axes = pcfg->hold_axes;
}


/***************************************************************************************************************
 * IMU SENSOR
 ***************************************************************************************************************/

void read_imu()
{
  int16_t gx, gy, gz, tmp; 
#if defined(USE_I2CDEVLIB)
#if defined(USE_MPU6050)
  accelgyro.getRotation(&gx, &gy, &gz);
#elif defined(USE_ITG3200)
  itg3205_gyro.getRotation(&gx, &gy, &gz);
#endif
#endif

#if defined(USE_I2CLIGHT)
#if defined(USE_MPU6050)
  mpu6050_read_gyro(&gx, &gy, &gz);
#elif defined(USE_ITG3200)
  itg3205_read_gyro(&gx, &gy, &gz);
#endif
#endif
  GYRO_ORIENTATION(gx, gy, gz);
  switch (mount_orient) {
  case MOUNT_NORMAL: break;
  case MOUNT_ROLL_90_LEFT: 
    tmp = gyro[1];
    gyro[1] = gyro[2];
    gyro[2] = -tmp;
    break;    
  case MOUNT_ROLL_90_RIGHT: 
    tmp = gyro[1];
    gyro[1] = -gyro[2];
    gyro[2] = tmp;
    break;    
  }
}

void init_imu() 
{
#if defined(USE_I2CDEVLIB)  
#if defined(USE_MPU6050)
  Wire.begin();
  accelgyro.initialize();
  if (!accelgyro.testConnection()) {
    set_led_msg(2, 5, LED_SHORT);
  }
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  accelgyro.setDLPFMode(MPU6050_DLPF_BW_256);
  accelgyro.setClockSource(MPU6050_CLOCK_PLL_ZGYRO); 
#elif defined(USE_ITG3200)
  Wire.begin();
  itg3205_gyro.initialize();
  itg3205_gyro.setClockSource(ITG3200_CLOCK_PLL_XGYRO); 
  itg3205_gyro.setFullScaleRange(ITG3200_FULLSCALE_2000); 
  itg3205_gyro.setDLPFBandwidth(ITG3200_DLPF_BW_5); 
//  gyro.setDLPFBandwidth(ITG3200_DLPF_BW_256); 
  if (!itg3205_gyro.testConnection()) {
    set_led_msg(2, 5, LED_SHORT);
  }
#endif
#endif

#if defined(USE_I2CLIGHT)
#if defined(USE_MPU6050)
  i2c_init(true, F_I2C);
  if (!mpu6050_init()) {
    set_led_msg(2, 5, LED_SHORT);
  }
#elif defined(USE_ITG3200)
  i2c_init(true, F_I2C);
  if (!itg3205_init()) {
    set_led_msg(2, 5, LED_SHORT);
  }
#endif
#endif
}

/***************************************************************************************************************
 * CALIBRATION
 ***************************************************************************************************************/

int8_t calibration_wag_count;
uint32_t last_calibration_wag_time;
 
struct _calibration {
  int8_t done;
  int16_t num_elements; // 3 for imu, 4 for rx
  int16_t low[4];
  int16_t high[4];
  int32_t sum[4];
  int16_t mean[4];
  int16_t num_samples;
};

void calibrate_set_led(struct _calibration *prx_cal, struct _calibration *pimu_cal) {
  set_led_msg(1, (pimu_cal->done ? 0 : 2) + (prx_cal->done ? 0 : 1), LED_SHORT);
} 

void calibrate_init_stat(struct _calibration *pcal, int8_t num_elements) 
{
  pcal->done = false;
  pcal->num_elements = num_elements;
  for (int8_t i=0; i < pcal->num_elements; i++) {
    pcal->low[i] = 32767;
    pcal->high[i] = -32768;
    pcal->sum[i] = 0;
  }
  pcal->num_samples = 0;
}

void calibrate_update_stat(struct _calibration *pcal, int16_t *psample) 
{
  for (int8_t i=0; i < pcal->num_elements; i++) {
    pcal->low[i] = min(pcal->low[i], psample[i]);
    pcal->high[i] = max(pcal->high[i], psample[i]);
    pcal->sum[i] += psample[i];
  }
} 

void calibrate_compute_mean(struct _calibration *pcal) 
{
  for (int8_t i=0; i < pcal->num_elements; i++) {
    pcal->mean[i] = pcal->sum[i] / pcal->num_samples;
  }
} 

int8_t calibrate_check_stat(struct _calibration *pcal, int16_t range)
{
  int8_t good = 0;
  for (int8_t i=0; i < pcal->num_elements; i++) {
    if (pcal->high[i] - pcal->low[i] < range)
      good++;
  }
  return (good == pcal->num_elements);
}

void calibrate_print_stat(struct _calibration *pcal)
{
#if defined(SERIAL_DEBUG) && 0
  Serial.println(pcal->num_samples);
  for (int8_t i=0; i < pcal->num_elements; i++) {  
    Serial.print(pcal->low[i]); Serial.print('\t');
    Serial.print(pcal->mean[i]); Serial.print('\t');
    Serial.println(pcal->high[i]);
  }
#endif
}

void calibrate_rx(struct _calibration *prx_cal)
{
  int16_t sample[4];
  int8_t is_calibrating = true;
  
  sample[0] = ail_in2;
  sample[1] = ele_in2;
  sample[2] = rud_in2;
  sample[3] = ailr_in2;
  calibrate_update_stat(prx_cal, sample);
  
  if (++prx_cal->num_samples == 100) {
    if (calibrate_check_stat(prx_cal, 20)) {
      calibrate_compute_mean(prx_cal);
      ail_in2_mid = prx_cal->mean[0];
      ele_in2_mid = prx_cal->mean[1];
      rud_in2_mid = prx_cal->mean[2];
      ailr_in2_mid = prx_cal->mean[3];
#if defined(SERIAL_DEBUG) && 0
      Serial.println("RX");
      calibrate_print_stat(prx_cal);
#endif
      prx_cal->done = true; // done
    } else {
      calibrate_init_stat(prx_cal, 4); // reset and try again
    }
  }
}

void calibrate_imu(struct _calibration *pimu_cal)
{  
  int16_t sample[3];
  int8_t is_calibrating = true;

  sample[0] = gyro[0];
  sample[1] = gyro[1];
  sample[2] = gyro[2];
  calibrate_update_stat(pimu_cal, sample);
  
  if (++pimu_cal->num_samples == 300) {
    if (calibrate_check_stat(pimu_cal, 50)) {
      calibrate_compute_mean(pimu_cal);
      for (int8_t i=0; i<3; i++)
        gyro0[i] = pimu_cal->mean[i];
#if defined(SERIAL_DEBUG) && 0
      Serial.println("GY");
      calibrate_print_stat(pimu_cal);
#endif
      pimu_cal->done = true; // done
    } else {
      calibrate_init_stat(pimu_cal, 3); // reset and try again
    }
  }
}


/***************************************************************************************************************
 * SERIAL
 ***************************************************************************************************************/

#if defined(SERIAL_DEBUG) && 0
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
    switch (state) {
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
            delay1(50);
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
 * RX IN - MIXER - SERVO OUT
 ***************************************************************************************************************/

void copy_rx_in()
{
  // lock-free method to copy isr-owned *_in vars to *_in2 vars
  int16_t tmp;
  ail_in2 = (tmp = ail_in) == ail_in ? tmp : ail_in2;
  ele_in2 = (tmp = ele_in) == ele_in ? tmp : ele_in2;
  rud_in2 = (tmp = rud_in) == rud_in ? tmp : rud_in2;
  ailr_in2 = (tmp = ailr_in) == ailr_in ? tmp : ailr_in2;
  aux_in2 = (tmp = aux_in) == aux_in ? tmp : aux_in2;
  aux2_in2 = (tmp = aux2_in) == aux2_in ? tmp : aux2_in2;
//  aux2_in2 = (tmp = ailr_in) == ailr_in ? tmp : aux2_in2; // johnrb
  thr_in2 = (tmp = thr_in) == thr_in ? tmp : thr_in2;
  flp_in2 = (tmp = flp_in) == flp_in ? tmp : flp_in2;
  
  if (wing_mode == WING_SINGLE_AIL)
    ailr_in2 = ail_in2;
}

void apply_mixer_change(int16_t *change) 
{
  // *_in2 => [change] => *_out2
  
  // for delta/vtail
  // tx = MID + (stick[x] + change[x]) * mult, where mult = 1/1, 3/2, 5/4
  
  // mixer
  int16_t tmp0, tmp1, tmp2;
  switch (wing_mode) {
  case WING_SINGLE_AIL:
  case WING_DUAL_AIL:
    ail_out2 = ail_in2 + change[0];
    ailr_out2 = ailr_in2 + change[0];
    ele_out2 = ele_in2 + change[1];
    rud_out2 = rud_in2 + change[2];
    break;
  case WING_DELTA:
    tmp0 =  ail_in2 + change[0];
    tmp1 =  ele_in2 + change[1];
    // apply 100% (1/1)
    //ail_out2 = ((tmp0 + tmp1) >> 1);
    //ele_out2 = ((tmp0 - tmp1) >> 1) + RX_WIDTH_MID;
    // apply 125% (5/4)
    tmp2 = tmp0 + tmp1;
    ail_out2 = (tmp2 >> 1) + (tmp2 >> 3) - (RX_WIDTH_MID >> 2);
    tmp2 = tmp0 - tmp1;
    ele_out2 = (tmp2 >> 1) + (tmp2 >> 3) + RX_WIDTH_MID;
    // apply 150% (3/2)
    //tmp2 = tmp0 + tmp1;
    //ail_out2 = tmp2 - (tmp2 >> 2) - (RX_WIDTH_MID >> 1);
    //tmp2 = tmp0 - tmp1;
    //ele_out2 = tmp2 - (tmp2 >> 2) + RX_WIDTH_MID;
    rud_out2 = rud_in2 + change[2];
    break;
  case WING_VTAIL:
    ail_out2 = ail_in2 + change[0];
    ailr_out2 = ailr_in2 + change[0];
    tmp1 =  ele_in2 + change[1];
    tmp2 =  rud_in2 + change[2];
    // apply 125% (5/4)
    tmp0 = tmp2 + tmp1;
    ele_out2 = (tmp0 >> 1) + (tmp0 >> 3) - (RX_WIDTH_MID >> 2);
    tmp0 = tmp2 - tmp1;
    rud_out2 = (tmp0 >> 1) + (tmp0 >> 3) + RX_WIDTH_MID;    
    break;
  }

  // throttle and flap pass through
  thr_out2 = thr_in2;  
  flp_out2 = flp_in2;  
}

void set_mixer_limits(int16_t low, int16_t high)
{
  for (int8_t i=0; i<4; i++) {
    mixer_out2_low_limit[i] = low;
    mixer_out2_high_limit[i] = high;
  }
}

void apply_mixer() 
{
  // *_in2 => [correction] => *_out2
  
  static int16_t * const pout2[]  = {&ail_out2, &ele_out2, &rud_out2, &ailr_out2};
  int8_t i;
  
  // if tracking servo limits, apply mixer with zero correction to determine output from rx alone (pass through)
  if (mixer_epa_mode == MIXER_EPA_TRACK) {
    const int16_t zero_correction[4] = {0, 0, 0, 0};
    apply_mixer_change((int16_t *)zero_correction);
    for (i=0; i<4; i++) {
      mixer_out2_low_limit[i] = min(*pout2[i], mixer_out2_low_limit[i]);
      mixer_out2_high_limit[i] = max(*pout2[i], mixer_out2_high_limit[i]);
    }
  }
  
  // apply mixer with actual correction and clamp servo output to limits
  apply_mixer_change(correction);
  for (i=0; i<4; i++) {
    *pout2[i] = constrain(*pout2[i], mixer_out2_low_limit[i], mixer_out2_high_limit[i]);
  }
}

void start_servo_frame()
{
  // copy *_out2 vars to isr-owned *_out vars
  servo_busy = true;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    ail_out = ail_out2;
    ele_out = ele_out2;
    rud_out = rud_out2;
    ailr_out = ailr_out2;
    thr_out = thr_out2;
    flp_out = flp_out2;
    
    TIMSK1 |= (1 << OCIE1A); // enable interrupt on TCNT1 == OCR1A
    TIFR1 |= (1 << OCF1A); // clear any pending interrupt
    OCR1A = TCNT1 + 4; // force interrupt condition
  } // reenable global interrupts
}

/***************************************************************************************************************
 * DUMP SENSORS
 ***************************************************************************************************************/

#if defined(DUMP_SENSORS)  
void dump_sensors()
{
  uint32_t t;
  uint32_t last_rx_time = 0;
  int8_t servo_sync = false;
  int16_t servo_out = RX_WIDTH_MID;
  int8_t servo_dir = 20;

  while (true) {
    t = micros1();

    if (rx_frame_sync || (int32_t)(t - last_rx_time) > 30000) {
      rx_frame_sync = false;
      copy_rx_in();
      servo_sync = true;
      last_rx_time = t;
    }

    Serial.print("  RX "); 
    Serial.print(ail_in2); Serial.print(' ');
    Serial.print(ele_in2); Serial.print(' ');
    Serial.print(rud_in2); Serial.print(' ');
    Serial.print(ailr_in2); Serial.print(' ');
    Serial.print(aux_in2); Serial.print(' ');
    Serial.print(aux2_in2); Serial.print(' ');
    Serial.print(thr_in2); Serial.print(' ');
    Serial.print(flp_in2); Serial.print('\t');
  
    uint8_t ail_vr2, ele_vr2, rud_vr2;
    ail_vr2 = ail_vr;
    ele_vr2 = ele_vr;
    rud_vr2 = rud_vr;
    start_next_adc(0);

    Serial.print("VR "); 
    Serial.print(ail_vr2, HEX); Serial.print(' ');
    Serial.print(ele_vr2, HEX); Serial.print(' ');
    Serial.print(rud_vr2, HEX); Serial.print('\t');
    
    read_switches();
    Serial.print("SW "); 
    Serial.print((bool)ail_sw); Serial.print(' ');
    Serial.print((bool)ele_sw); Serial.print(' ');
    Serial.print((bool)rud_sw); Serial.print(' ');
    Serial.print((bool)vtail_sw); Serial.print(' ');
    Serial.print((bool)delta_sw); Serial.print(' ');
    Serial.print((bool)aux_sw); Serial.print('\t');
  
    read_imu();
    Serial.print("GY "); 
    Serial.print(gyro[0]); Serial.print('\t');
    Serial.print(gyro[1]); Serial.print('\t');
    Serial.print(gyro[2]); Serial.print('\t');

    servo_out += servo_dir;
    if (servo_out < RX_WIDTH_LOW_FULL || servo_out > RX_WIDTH_HIGH_FULL) {
      servo_out = constrain(servo_out, RX_WIDTH_LOW_FULL, RX_WIDTH_HIGH_FULL);
      servo_dir = -servo_dir;
    }
    ail_out2 = ele_out2 = rud_out2 = ailr_out2 = thr_out2 = flp_out2 = servo_out;
//    ail_out2 = ele_out2 = rud_out2 = ailr_out2 = thr_out2 = flp_out2 = RX_WIDTH_MID;
    if (servo_sync && !servo_busy) {
      servo_sync = false;
      start_servo_frame();
    } 

    Serial.print("MISC "); 
    Serial.print(i2c_errors); Serial.print(' ');
    Serial.print(wing_mode); Serial.print(' ');
    Serial.print(mixer_epa_mode); Serial.print(' ');
    Serial.print(cppm_mode); Serial.print(' ');
    Serial.print(mount_orient); Serial.print(' ');
    Serial.print(get_free_sram()); Serial.print(' ');
    Serial.print(servo_out); Serial.print(' ');
    Serial.println();

    set_led(LED_INVERT);
    delay1(50);
  }
}
#endif

/***************************************************************************************************************
* STICK CONFIGURATION
***************************************************************************************************************/
// telephone pad mapping
// 1 2 3
// 4 5 6
// 7 8 9 

 struct _stick_zone {
  int8_t zx, zy;
  uint8_t curr, prev; // zone [1-9] telephone pad mapping
  uint8_t move; // 0x<prev><curr>
  bool zx_rev, zy_rev;
  bool zx_sided, zy_sided;
};
 
void stick_zone_init(struct _stick_zone *psz)
{
  psz->zx = psz->zy = 1; // neutral zone
  psz->curr = 5; // center
  psz->zx_rev = psz->zy_rev = false; // rev=false => top/left == narrow AIL/ELE pulse
  psz->zx_sided = psz->zy_sided = false; // have not found a side yet
}

int8_t stick_zone(int16_t pwm, bool rev)
{
  int8_t zone;
  if (pwm <= 1200) zone = 0;
  else if (pwm >= 1300 && pwm <= 1700) zone = 1;
  else if (pwm >= 1800) zone = 2;
  else zone = -1; // in the hysteresis region
  
  if (zone >= 0 && rev)
    zone = 2 - zone;
  return zone;
}

bool stick_zone_update(struct _stick_zone *psz)
{  
  int8_t tmp;
  bool moved;

  psz->prev = psz->curr;
  psz->zx = (tmp = stick_zone(ail_in2, psz->zx_rev)) >= 0 ? tmp : psz->zx; // hysteresis
  psz->zy = (tmp = stick_zone(ele_in2, psz->zy_rev)) >= 0 ? tmp : psz->zy;
    
  // we assume that the first corner visited bottom left (position 7)
  // so stick on the left if the AIL pulse is at either horizontal end
  if (!psz->zx_sided && (psz->zx == 0 || psz->zx == 2)) {
    if (psz->zx == 2) {
      psz->zx_rev = true;
      psz->zx = 2 - psz->zx;
    }
    psz->zx_sided = true;
  }
  // and stick is on the bottom if the ELE pulse is at either vertical end
  if (!psz->zy_sided && (psz->zy == 0 || psz->zy == 2)) {
    if (psz->zy == 0) {
      psz->zy_rev = true;
      psz->zy = 2 - psz->zy;
    }
    psz->zy_sided = true;
  }

  psz->curr = psz->zy * 3 + psz->zx + 1;
  // telephone pad mapping
  // 1 2 3
  // 4 5 6
  // 7 8 9 
  moved = (psz->curr != psz->prev);
  if (moved) {
    psz->move = (psz->prev << 4) | psz->curr; // move = 0x[prev digit][curr digit]
#if defined(SERIAL_DEBUG) && 0
      Serial.print("stick_zone "); 
      Serial.print(psz->zx); Serial.print(' ');
      Serial.print(psz->zy); Serial.print(' ');
      Serial.print(psz->zx_rev); Serial.print(' ');
      Serial.print(psz->zy_rev); Serial.print(' ');
      Serial.print(psz->move, HEX);
      Serial.println();
#endif
  }
  return moved;
}

void stick_config(struct _stick_zone *psz)
{
// 1= wing_mode (see enum WING_MODE)
// 2,3,4= [roll|pitch|yaw]_gain (see vr_notch_table[])
// 5= mixer_epa_mode (see enum MIXER_EPA_MODE)
// 6= cppm_mode (see enum CPPM_MODE)
// 7= mount_orient (see enum MOUNT_ORIENT)
// 8= exit

// note: be careful about off-by-one errors in this function

  const int8_t param_ymin[] = {WING_SINGLE_AIL, -4, -4, -4, MIXER_EPA_FULL , CPPM_NONE,     MOUNT_NORMAL       , 1};
  const int8_t param_ymax[] = {WING_DUAL_AIL  , +4, +4, +4, MIXER_EPA_TRACK, CPPM_AETR1a2F, MOUNT_ROLL_90_RIGHT, 2};
  const int8_t param_xcount = sizeof(param_ymin)/sizeof(param_ymin[0]);
  int8_t param_yval[param_xcount];

  const int16_t servo_swing = 300;
  const int32_t servo_interval[] = {100000L, 400000L};

  int8_t x = 0; // 0-based, x=0 => wing_mode
  int8_t servo_sync = false;
  uint32_t last_servo_update_time=0;
  int32_t wait_interval = servo_interval[1];
  struct _eeprom_cfg eeprom_cfg;
  
  // read eeprom
  eeprom_read_cfg(&eeprom_cfg, eeprom_cfg1_addr, eeprom_cfg_ver);
  param_yval[0] = (int8_t) eeprom_cfg.wing_mode;
  param_yval[1] = eeprom_cfg.vr_notch[0];
  param_yval[2] = eeprom_cfg.vr_notch[1];
  param_yval[3] = eeprom_cfg.vr_notch[2];
  param_yval[4] = (int8_t) eeprom_cfg.mixer_epa_mode;
  param_yval[5] = (int8_t) eeprom_cfg.cppm_mode;
  param_yval[6] = (int8_t) eeprom_cfg.mount_orient;
  param_yval[7] = 1; // exit option
  
  int8_t update_x = (x + 1) << 1;
  int8_t update_y = param_yval[x] << 1;
  while (true) {
    uint32_t t = micros1();
  
    if ((int32_t)(t - last_servo_update_time) > wait_interval) {
      if (update_x > 0) {
        ele_out2 = RX_WIDTH_MID + (update_x & 1 ? 0 : servo_swing);
        ail_out2 = RX_WIDTH_MID;
        wait_interval = servo_interval[update_x & 1];
        update_x--;
      } else {
        if (update_y != 0) {
          int8_t sign = update_y > 0 ? 1 : -1;
          int16_t signed_change = sign > 0 ? servo_swing : -servo_swing;
          ail_out2 = RX_WIDTH_MID + (update_y & 1 ? 0 : signed_change);
          wait_interval = servo_interval[update_y & 1];
          update_y -= sign;
         }
       }
      last_servo_update_time = t;
    }
    
    if (rx_frame_sync) {
      rx_frame_sync = false;
      copy_rx_in();
      servo_sync = true;
    }
    if (servo_sync && !servo_busy) {
      servo_sync = false;
      start_servo_frame();
    } 

    if (update_x == 0 && /*update_y == 0 &&*/ stick_zone_update(psz)) {
      switch (psz->move) {
        case 0x54: x = max(x - 1, 0); update_x = (x + 1) << 1; break; // left
        case 0x56: x = min(x + 1, param_xcount - 1); update_x = (x + 1) << 1; break; // right
        case 0x52: param_yval[x] = min(param_yval[x] + 1, param_ymax[x]); update_y = param_yval[x] << 1; break; // up
        case 0x58: param_yval[x] = max(param_yval[x] - 1, param_ymin[x]); update_y = param_yval[x] << 1; break; // down
      }

      // if updating x (index) servo then update y (value) servo as well
      if (update_x > 0) {
        update_y = param_yval[x] << 1;
      }

      // check for exit option
      if (x == (param_xcount - 1) && param_yval[x] == 2) {
        break;
      }
 
  #if defined(SERIAL_DEBUG) && 0
      Serial.print("param "); 
      Serial.print(psz->prev, HEX); Serial.print(' ');
      Serial.print(psz->curr, HEX); Serial.print(' ');
      Serial.print(x); Serial.print(' ');
      Serial.print(param_yval[x]); Serial.print(' ');
      Serial.println();
  #endif      
    }
  }

  // write eeprom;
  eeprom_cfg.wing_mode = (enum WING_MODE) param_yval[0];
  eeprom_cfg.vr_notch[0] = param_yval[1];
  eeprom_cfg.vr_notch[1] = param_yval[2];
  eeprom_cfg.vr_notch[2] = param_yval[3];
  eeprom_cfg.mixer_epa_mode = (enum MIXER_EPA_MODE) param_yval[4];
  eeprom_cfg.cppm_mode = (enum CPPM_MODE) param_yval[5];
  eeprom_cfg.mount_orient = (enum MOUNT_ORIENT) param_yval[6];
  eeprom_write_cfg(&eeprom_cfg, eeprom_cfg1_addr);
  eeprom_write_cfg(&eeprom_cfg, eeprom_cfg2_addr);
}


/***************************************************************************************************************
 * SETUP
 ***************************************************************************************************************/

void setup() 
{
  int8_t i;

  init_clock();
  init_led();
  if (get_free_sram() < 128)
    set_led_msg(2, 20, LED_VERY_SHORT); 

#if defined(SERIAL_DEBUG) || defined(DUMP_SENSORS)
  Serial.begin(115200L);
#endif
  
#if defined(RX3S_V1) || defined(RX3S_V2) // test should be if atmega168/328
  // clear wd reset bit and disable wdt in case it was enabled due to stick config reboot
  MCUSR &= ~(1 << WDRF);
  wdt_disable();
#endif // RX3S_V1 || RX3S_V2

  // set up default RATE pid parameters
  for (i=0; i<3; i++) {
    pid_param_rate.kp[i] = 500;
    pid_param_rate.ki[i] = 0;
    pid_param_rate.kd[i] = 500;
    pid_param_rate.i_limit[i] = 0;
  }
  pid_param_rate.output_shift = 8;

   // set up default HOLD pid parameters
  for (i=0; i<3; i++) {
    pid_param_hold.kp[i] = 500;
    pid_param_hold.ki[i] = 500;
    pid_param_hold.kd[i] = 500;
    pid_param_hold.i_limit[i] = 0;
  }
  pid_param_hold.output_shift = 8;
  
  // eeprom processing
  struct _eeprom_cfg eeprom_cfg1, eeprom_cfg2;
  struct _eeprom_stats eeprom_stats;
  
  uint8_t ret = boot_check(EEPROM_RESET_IN_PIN, EEPROM_RESET_OUT_PIN) ? 0xff : 0x00;
    
  eeprom_read_block(&eeprom_stats, (void *)eeprom_stats_addr, sizeof(eeprom_stats));
  if (ret || (eeprom_stats.device_id != DEVICE_ID || eeprom_stats.device_ver != DEVICE_VER)) {
    // initialize eeprom stats
    eeprom_stats.device_id = DEVICE_ID;
    eeprom_stats.device_ver = DEVICE_VER;
    eeprom_stats.eeprom_cfg1_err = 0;
    eeprom_stats.eeprom_cfg2_err = 0;
    eeprom_stats.eeprom_cfg12_reset = 0;
  }

  if (eeprom_read_cfg(&eeprom_cfg1, eeprom_cfg1_addr, eeprom_cfg_ver) < 0)
    ret |= 0x01;
  if (eeprom_read_cfg(&eeprom_cfg2, eeprom_cfg2_addr, eeprom_cfg_ver) < 0)
    ret |= 0x10;

  calibration_wag_count = 3;
  switch (ret) {
  case 0x00: // both copy 1 and 2 good
    break;
  case 0x01: // copy 1 bad, copy 2 good
    eeprom_write_cfg(&eeprom_cfg2, eeprom_cfg1_addr);
    eeprom_stats.eeprom_cfg1_err++;
    eeprom_cfg1 = eeprom_cfg2;
    break;
  case 0x10: // copy 1 good, copy 2 bad
    eeprom_write_cfg(&eeprom_cfg1, eeprom_cfg2_addr);
    eeprom_stats.eeprom_cfg2_err++;
    break;
  default: // both copy 1 and 2 bad (ret=0x11), or boot_check (ret=0xff)
    eeprom_cfg1.ver = eeprom_cfg_ver;
    eeprom_copy_to_cfg(&eeprom_cfg1);
    eeprom_write_cfg(&eeprom_cfg1, eeprom_cfg1_addr);
    eeprom_write_cfg(&eeprom_cfg1, eeprom_cfg2_addr);
    eeprom_stats.eeprom_cfg12_reset++;
    calibration_wag_count = 9;
    break;
  }
  eeprom_copy_from_cfg(&eeprom_cfg1);
  
  if (ret != 0x00) {
    // update stats to eeprom if at least one cfg was written
    eeprom_write_block(&eeprom_stats, (void *)eeprom_stats_addr, sizeof(eeprom_stats));
  }
  
  if (ret == 0xff) {
    // boot_check initiated, halt and flash led indefinitely
    while (true) {
      set_led(LED_INVERT);
      delay(100);
    }      
  }

#if !defined(NO_ONEWIRE)
  // check for program box, reboot on return if connected
  // cannot disable TIMER0 or init TIMER1 before this call
  if (ow_loop()) {
    wdt_enable(WDTO_1S);
    while (true);
  }
#endif // !NO_ONEWIRE
  
  // init TIMER1
  TCCR1A = 0; // normal counting mode
  TCCR1B = (1 << CS11); // clkio/8
  TIMSK1 = (1 << TOIE1); // enable overflow interrupt
  // TIMSK1 |= (1 << OCIE1A); // enable interrupt on TCNT1 == OCR1A

  // disable TIMER0
  TCCR0B &= ~((1 << CS00) | (1 << CS01) | (1 << CS02)); // clock stopped
  // TIMSK0 &= ~(1 << TOIE0); // disable overflow interrupt
  
  
  // set mixer limits based on configuration
  switch (mixer_epa_mode) {
  case MIXER_EPA_FULL: // 1000-2000
    set_mixer_limits(RX_WIDTH_LOW_FULL, RX_WIDTH_HIGH_FULL);
    break;
  case MIXER_EPA_NORM: // 1100-1900
    set_mixer_limits(RX_WIDTH_LOW_NORM, RX_WIDTH_HIGH_NORM);
    break;
  case MIXER_EPA_TRACK: // 1250-1750 initially and track
    set_mixer_limits(RX_WIDTH_LOW_TRACK, RX_WIDTH_HIGH_TRACK);
    break;  
  }

  // init digital in for dip switches to read config settings
  init_digital_in_sw(); // sw
  read_switches();
  
  // device-specific modes and pin assignment differences
  
  const enum WING_MODE dip_sw_to_wing_mode_map[] = {
    WING_DUAL_AIL, // 0=rev / 0=rev
    WING_VTAIL, // 0=rev / 1=norm
    WING_DELTA, // 1=norm / 0=rev
    WING_SINGLE_AIL // 1=norm / 1=norm
  };
  
#if defined(RX3S_V1)

  wing_mode = dip_sw_to_wing_mode_map[(ele_sw ? 2 : 0) | (rud_sw ? 1 : 0)];

  switch (wing_mode) {
  case WING_SINGLE_AIL:
  case WING_DELTA:
  case WING_VTAIL:
    // PD7 7 AUX_IN instead of AILR_OUT
    pwm_out_var[3] = NULL; // disable ailr_out
    pwm_out_pin[3] = -1; //
    rx_portd[7] = &aux_in; // enable aux_in
    break;
  case WING_DUAL_AIL:
    // PB3 11 AUX_IN instead of MOSI
    // PB4 12 AILR_IN instead of MISO
    rx_portb[3] = &aux_in; // enable aux_in
    rx_portb[4] = &ailr_in; // enable ailr_in
    break;
  }
        
#if !defined(NO_CPPM)
  if (cppm_mode != CPPM_NONE) {
    // PB0 8 CPPM_IN instead of AIL_IN
    // PB2 9 FLP_OUT instead of ELE_IN
    // PB2 10 THR_OUT instead of RUD_IN
    rx_portb[0] = NULL; // disable ail_in
    rx_portb[1] = NULL; // disable ele_in
    rx_portb[2] = NULL; // disable rud_in
    pwm_out_var[4] = &thr_out; // enable thr_out
    pwm_out_pin[4] = THR_OUT_PIN; //
    pwm_out_var[5] = &flp_out; // enable flp_out
    pwm_out_pin[5] = FLP_OUT_PIN; //
  }
#endif // !NO_CPPM
#endif // RX3S_V1

#if defined(RX3S_V2)

  wing_mode = dip_sw_to_wing_mode_map[(vtail_sw ? 2 : 0) | (delta_sw ? 1 : 0)];
  
  if (wing_mode == WING_DUAL_AIL) {
    if (ail_sw) {
      // WING_DUAL_AIL mode A
      // PB3 11 AILR_IN instead of AUX_IN
      rx_portb[3] = &ailr_in; // replace aux_in
      rx3s_v2_wing_dual_ailB = false;
    } else {
      // WING_DUAL_AIL mode B
      // PD2 2 AILR_IN instead of ELE_SW
      din_portd[2] = NULL; // disable ele_sw
      rx_portd[2] = &ailr_in; // enable ailr_in
      rx3s_v2_wing_dual_ailB = true;
    }
  }

#if !defined(NO_CPPM)
  if (cppm_mode != CPPM_NONE) {
    // PB0 8 CPPM_IN instead of AIL_IN
    // PB2 10 THR_OUT instead of RUD_IN
    rx_portb[0] = NULL; // disable ail_in
    rx_portb[1] = NULL; // disable ele_in
    rx_portb[2] = NULL; // disable rud_in
    pwm_out_var[4] = &thr_out; // enable thr_out
    pwm_out_pin[4] = THR_OUT_PIN; //
    pwm_out_var[5] = &flp_out; // enable flp_out
    pwm_out_pin[5] = FLP_OUT_PIN; //
  }
#endif // !NO_CPPM
#endif // RX3S_V2

#if defined(NANOWII)
#if !defined(NO_CPPM)
  if (cppm_mode != CPPM_NONE) {
    // PE6 7 CPPM_IN instead of THR/AUX2_IN
    // PC6 5 THR_OUT instead of M
    // PD7 6 FLP_OUT instead of M
    pwm_out_var[4] = &thr_out; // enable thr_out
    pwm_out_pin[4] = THR_OUT_PIN; //
    pwm_out_var[5] = &flp_out; // enable flp_out
    pwm_out_pin[5] = FLP_OUT_PIN; //
  }
#endif // !NO_CPPM
#endif // NANOWII

  set_led_msg(0, wing_mode, LED_LONG);

  init_analog_in(); // vr
  init_digital_in_rx(); // rx
  init_digital_out(); // servo
  init_imu(); // gyro/accelgyro

  copy_rx_in(); // init *_in2 vars
  apply_mixer(); // init *_out2 vars

#if defined(DUMP_SENSORS)  
  dump_sensors();
#endif
  //struct _stick_zone sz;
  //stick_config(&sz);
}

/***************************************************************************************************************
 * LOOP
 ***************************************************************************************************************/

 void loop() 
{ 
  int8_t i;
  uint32_t t = micros1();
  uint32_t last_rx_time = t;
  uint32_t last_imu_time = t;
  uint32_t last_pid_time = t;
  uint32_t last_vr_time = t;
  int8_t servo_sync = false;

  int16_t vr_gain[3]= {VR_GAIN_MAX, VR_GAIN_MAX, VR_GAIN_MAX};
  int16_t stick_gain[3];
  int16_t master_gain;

#if !defined(NO_STICKCONFIG)  
  uint32_t stick_config_check_time = t;
  struct _stick_zone stick_zone;
  const uint8_t stick_config_seq[] = {0x78, 0x89, 0x98, 0x87, 0x78, 0x89, 0x98, 0x87, 0xff}; // 7-9-7-9-7
  int8_t stick_config_seq_i = 0;
  int8_t stick_configurable = true;

  stick_zone_init(&stick_zone); // stick zone setup
#endif // !NO_STICKCONFIG
  
  struct _calibration rx_cal;
  struct _calibration imu_cal;
    
  // calibration setup  
  calibrate_init_stat(&rx_cal, 4);
  calibrate_init_stat(&imu_cal, 3);
  last_calibration_wag_time = t;

again:
  t = micros1();
  update_led(t);

  // update rx frame data with rx ISR received reference channel or after timeout
  if (rx_frame_sync || (int32_t)(t - last_rx_time) > 30000) {
    rx_frame_sync = false;
    copy_rx_in();
    if (!rx_cal.done) {
      calibrate_rx(&rx_cal);
      calibrate_set_led(&rx_cal, &imu_cal);
    }
    servo_sync = true;    
    last_rx_time = t;
  }

  // sync servo frame with rx frame immediately when servo ISR is idle
  if (servo_sync && !servo_busy) {
    servo_sync = false;
    apply_mixer();
    start_servo_frame();
  }   

  // schedule imu read just after servo pulse start, if needed.
  // takes around 1ms at 100khz, 250us at 400khz
  if ((int32_t)(t - last_imu_time) > PID_PERIOD) {
    read_imu();
    if (!imu_cal.done) {
      calibrate_imu(&imu_cal);
      calibrate_set_led(&rx_cal, &imu_cal);
    }
    last_imu_time = t;
  }

#if !defined(NO_STICKCONFIG)
  if (stick_configurable && stick_zone_update(&stick_zone)) {
    stick_config_seq_i = (stick_zone.move == stick_config_seq[stick_config_seq_i]) ? stick_config_seq_i + 1 : 0;
    if (stick_config_seq[stick_config_seq_i] == 0xff) {
      stick_config(&stick_zone); // completed sequence, enter stick config mode
      // RESET SYSTEM ON RETURN    
      wdt_enable(WDTO_1S);
      while (true);
    }      
    if ((int32_t)(t - stick_config_check_time) > 15000000L) // 15 sec to enter config mode
      stick_configurable = false;
  }
#endif // !NO_STICKCONFIG

  // short circuit rest of the loop if calibration not done, do not compute correction[]
  if (!rx_cal.done || !imu_cal.done) {
    goto again;
  }
   
  if ((int32_t)(t - last_pid_time) > PID_PERIOD) {
    // determine how much sticks are off center (from neutral)
    int16_t ail_stick_pos = abs(((ail_in2 - ail_in2_mid) + (ailr_in2 - ailr_in2_mid)) >> 1);
    int16_t ele_stick_pos = abs(ele_in2 - ele_in2_mid);
    int16_t rud_stick_pos = abs(rud_in2 - rud_in2_mid);

    // vr_gain[] [-128, 128] from VRs or config
    
    // stick_gain[] [1100, <ail*|ele|rud>_in2_mid, 1900] => [0%, 100%, 0%] = [0, STICK_GAIN_MAX, 0]
    stick_gain[0] = STICK_GAIN_MAX - constrain(ail_stick_pos, 0, STICK_GAIN_MAX);
    stick_gain[1] = STICK_GAIN_MAX - constrain(ele_stick_pos, 0, STICK_GAIN_MAX);
    stick_gain[2] = STICK_GAIN_MAX - constrain(rud_stick_pos, 0, STICK_GAIN_MAX);
    
    // master gain [1500-1100] or [1500-1900] => [0, MASTER_GAIN_MAX] 
    master_gain = constrain(abs(aux_in2 - RX_WIDTH_MID), 0, MASTER_GAIN_MAX);     
        
    // stabilization mode
    enum STAB_MODE stab_mode2 = (aux_in2 <= RX_WIDTH_MID) ? STAB_RATE : STAB_HOLD;
    if (stab_mode2 != stab_mode) {
      stab_mode = stab_mode2;
      // reset attitude error on mode change
      for (i=0; i<3; i++) 
        pid_state.sum_err[i] = 0;
      set_led_msg(1, (stab_mode == STAB_RATE) ? 0 : 4, LED_SHORT);
    }
  
    // commanded angular rate (could be from [ail|ele|rud]_in2, note direction/sign)
    
    if (stab_mode == STAB_RATE) {
      // zero roll rate, stabilization only
      for (i=0; i<3; i++) 
        pid_state.setpoint[i] = 0;
    }
    
    if (stab_mode == STAB_HOLD) {
      // stick controlled roll rate
      // max stick == 400, if "<< 4" then 400*16 = 6400 => 6400/8192*500 = 391deg/s (32768 == 2000deg/s)
      int16_t sp[3];
      sp[0] = ((ail_in2 - ail_in2_mid) + (ailr_in2 - ailr_in2_mid)) << (2-1);
      sp[1] = (ele_in2 - ele_in2_mid) << 2;
      sp[2] = (rud_in2 - rud_in2_mid) << 2;
      for (i=0; i<3; i++) {
        pid_state.setpoint[i] = vr_gain[i] < 0 ? sp[i] : -sp[i];      
      }

      for (i=0; i<3; i++) {
        // 2000 deg/s == 32768 units, so 1 deg/(PID_PERIOD=10ms) == 32768/20 units
        pid_param_hold.i_limit[i] = ((int32_t)30 * (32768 / 2 / (PID_PERIOD / 1000)) * stick_gain[i]) >> 9; 
      }
    }
    
    // measured angular rate (from the gyro and apply calibration offset)
    pid_state.input[0] = constrain(gyro[0] - gyro0[0], -8192, 8191);
    pid_state.input[1] = constrain(gyro[1] - gyro0[1], -8192, 8191);
    pid_state.input[2] = constrain(gyro[2] - gyro0[2], -8192, 8191);        
    
    // apply PID control
    compute_pid(&pid_state, (stab_mode == STAB_RATE) ? &pid_param_rate : &pid_param_hold);

    // apply vr_gain, stick_gain and master_gain
    for (i=0; i<3; i++) {
      // vr_gain [-128,0,127]/128, stick_gain [0,400,0]/512, master_gain [400,0,400]/512
      correction[i] = ((((int32_t)pid_state.output[i] * vr_gain[i] >> 7) * stick_gain[i]) >> 9) * master_gain >> 9;
    }
    
    // calibration wag on all surfaces if needed
    if (calibration_wag_count > 0) {
      if ((int32_t)(t - last_calibration_wag_time) > 200000L) {
        calibration_wag_count--;
        last_calibration_wag_time = t;
      }
      for (i=0; i<3; i++)
        correction[i] += (calibration_wag_count & 1) ? 150 : -150;    
    }    
    
#if defined(SERIAL_DEBUG) && 0
    Serial.print(correction[0]); Serial.print('\t');
    Serial.print(correction[1]); Serial.print('\t');
    Serial.print(correction[2]); Serial.println('\t');
#endif
    last_pid_time = t;
  }

  if ((int32_t)(t - last_vr_time) > 500123) {
    // sample all adc channels
    uint8_t ail_vr2, ele_vr2, rud_vr2;
    ail_vr2 = ail_vr;
    ele_vr2 = ele_vr;
    rud_vr2 = rud_vr;

    // vr_gain[] [-128, 0, 127] => [-100%, 0%, 100%] = [-128, 0, 127]
    vr_gain[0] = (int16_t)ail_vr2 - 128;
    vr_gain[1] = (int16_t)ele_vr2 - 128;
    vr_gain[2] = (int16_t)rud_vr2 - 128;
    
    // deadband filter
    const int8_t vr_deadband = 8;
    for (i=0; i<3; i++) {
      if (vr_gain[i] > -vr_deadband && vr_gain[i] < vr_deadband) 
        vr_gain[i] = 0; 
    }

    start_next_adc(0);
    last_vr_time = t;
  }

  goto again; // the dreaded "goto" statement :O
}
