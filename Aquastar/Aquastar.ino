/* Aquastar ****************************************************************************************************/

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include "FlightStab.h"

#define LED_PIN 13

/*
 Aquastar pin mapping
 PB0  8 LCD_D4         PC0 14/A0               PD0 0 RXD
 PB1  9 LCD_D5         PC1 15/A1               PD1 1 TXD
 PB2 10 LCD_D6         PC2 16/A2 LCD_RS        PD2 2 UP_BUTTON
 PB3 11 LCD_D7 (MOSI)  PC3 17/A3 LCD_RW        PD3 3 LEFT_BUTTON
 PB4 12 (MISO)         PC4 18/A4 LCD_EN (SDA)  PD4 4 RIGHT_BUTTON
 PB5 13 (SCK)          PC5 19/A5 (SCL)         PD5 5 DOWN_BUTTON
 PB6 14 (XTAL1)        PC6 (RESET)             PD6 6 
 PB7 15 (XTAL2)                                PD7 7 
*/

// BUTTONS

enum BUTTON_STATE {BUTTON_UP, BUTTON_DOWN, BUTTON_DOWN_AUTO};
enum BUTTON_EVENT {BUTTON_NONE, BUTTON_RELEASE, BUTTON_PRESS};

BUTTON_STATE button_state[] = {BUTTON_UP, BUTTON_UP, BUTTON_UP, BUTTON_UP};
BUTTON_EVENT button_event[] = {BUTTON_NONE, BUTTON_NONE, BUTTON_NONE, BUTTON_NONE};
const uint8_t button_pin[] = {2, 3, 4, 5}; // up, left, right, down

// LCD

LiquidCrystal lcd(A2, A3, A4, 8, 9, 10, 11); // (rs, rw, enable, d4, d5, d6, d7)

// string table

prog_char wing_mode[] PROGMEM = "WING MODE";
prog_char roll_gain[] PROGMEM = "ROLL GAIN";
prog_char pitch_gain[] PROGMEM = "PITCH GAIN";
prog_char yaw_gain[] PROGMEM = "YAW GAIN";
prog_char mixer_epa_mode[] PROGMEM = "EPA MODE";
prog_char cppm_mode[] PROGMEM = "CPPM MODE";
prog_char mount_orient[] PROGMEM = "MOUNT ORIENT";
prog_char config_exit[] PROGMEM = "EXIT";

prog_char wing_single_ail[] PROGMEM = "Single Aileron";
prog_char wing_delta[] PROGMEM = "Delta";
prog_char wing_vtail[] PROGMEM = "VTail";
prog_char wing_dual_ail[] PROGMEM = "Dual Ailerons";

prog_char mixer_epa_full[] PROGMEM = "Full 1000-2000";
prog_char mixer_epa_norm[] PROGMEM = "Norm 1100-1900";
prog_char mixer_epa_track[] PROGMEM = "Tracking";

prog_char cppm_none[] PROGMEM = "None";
prog_char cppm_open9x[] PROGMEM = "RETA1a2F";

prog_char mount_normal[] PROGMEM = "Normal";
prog_char mount_roll90left[] PROGMEM = "Roll 90deg Left";
prog_char mount_roll90right[] PROGMEM = "Roll 90deg Right";

prog_char config_update[] PROGMEM = "Update EEPROM ^";

prog_char *menu_heading[] PROGMEM = {
  wing_mode,
  roll_gain,
  pitch_gain,
  yaw_gain,
  mixer_epa_mode,
  cppm_mode,
  mount_orient,
  config_exit,
};

prog_char *menu_item[][4] PROGMEM = {
  {wing_single_ail,
   wing_delta,
   wing_vtail,
   wing_dual_ail},
   {},
   {},
   {},
  {mixer_epa_full,
   mixer_epa_norm,
   mixer_epa_track},
  {cppm_none,
   cppm_open9x},
  {mount_normal,
   mount_roll90left,
   mount_roll90right},
  {config_update},
};

const int8_t menu_count[] = {4, 0, 0, 0, 3, 2, 3, 1};


/***************************************************************************************************************
* BUTTONS
***************************************************************************************************************/

bool button_read()
{
  bool has_event = false;
  uint32_t t = micros();
  static uint32_t button_down_time[4];
  static uint32_t button_debounce_time[4]; // time of last button state change
  const int32_t button_debounce_delay = 50000; // debounce interval
  const int32_t button_auto_delay = 800000; // time between first press and auto repeat start
  const int32_t button_auto_period = 200000; // time between auto repeat events
  
  for (int8_t i=0; i<4; i++) {
    button_event[i] = BUTTON_NONE;    
    BUTTON_STATE b = digitalRead(button_pin[i]) ? BUTTON_UP : BUTTON_DOWN;
    
    switch ((b << 4) | button_state[i]) {
    
    // release up
    case ((BUTTON_UP << 4) | BUTTON_DOWN):
    case ((BUTTON_UP << 4) | BUTTON_DOWN_AUTO):
      if ((int32_t)(t - button_debounce_time[i]) > button_debounce_delay) {
        button_event[i] = BUTTON_RELEASE;
        button_state[i] = BUTTON_UP;
        button_debounce_time[i] = t;
      }
      break;
      
    // press down
    case ((BUTTON_DOWN << 4) | BUTTON_UP):
      if ((int32_t)(t - button_debounce_time[i]) > button_debounce_delay) {
        button_event[i] = BUTTON_PRESS;
        button_state[i] = BUTTON_DOWN;
        button_down_time[i] = t;
        button_debounce_time[i] = t;
      }
      break;

      // holding down till auto repeat start
    case ((BUTTON_DOWN << 4) | BUTTON_DOWN):
      if ((int32_t)(t - button_down_time[i]) > button_auto_delay) {
        button_event[i] = BUTTON_PRESS;
        button_state[i] = BUTTON_DOWN_AUTO;
        button_down_time[i] = t;
      }
      break;      
      
    // holding down during auto repeat
    case ((BUTTON_DOWN << 4) | BUTTON_DOWN_AUTO):
      if ((int32_t)(t - button_down_time[i]) > button_auto_period) {
        button_event[i] = BUTTON_PRESS;
        button_down_time[i] = t;
      }
      break;
    }
    
    has_event = has_event || (button_event[i] != BUTTON_NONE);
  }
  return has_event;
}

void button_init()
{
  for (int8_t i=0; i<4; i++) {
    pinMode(button_pin[i], INPUT);
    digitalWrite(button_pin[i], HIGH);
  }
}

/***************************************************************************************************************
* SERIAL MESSAGING
***************************************************************************************************************/

void serial_write_byte(uint8_t b) {
  Serial.write(b);
  Serial.read(); // drain the received echo due to loopback
  delay(5); // delay for the one-wire library to ready the next receive
  // TODO: reduce delay?
}

bool send_msg(void *buf, int8_t buf_len) {
  uint8_t chksum = buf_len;
  serial_write_byte('$');
  serial_write_byte(buf_len);
  for (int8_t i=0; i<buf_len; i++) {
    chksum += ((uint8_t *)buf)[i];
    serial_write_byte(((uint8_t *)buf)[i]);
  }
  serial_write_byte((chksum ^ 0xff) + 1);
}

bool recv_msg(void *buf, int8_t buf_len, int16_t timeout_ms) {
  enum RECV_STATE {IDLE, HEADER, LENGTH} state = IDLE;
  uint32_t t = micros();
  
  while ((int32_t)(micros() - t) < timeout_ms * 1000) {
    uint8_t ch, len, chksum, i;

    if (!Serial.available())
      continue;
        
    ch = Serial.read();    
    switch (state) {
    case IDLE: 
      if (ch == '$') {
        state = HEADER; // found HEADER
      }
      break;
    case HEADER:
      if (ch <= buf_len) {
        // found valid LENGTH
        len = ch;
        chksum = ch;
        i = 0;
        state = LENGTH;
      } else {
        state = IDLE;
      }
      break;
    case LENGTH:
      chksum += ch;
      if (len-- != 0) {
        ((uint8_t *)buf)[i++] = ch;
      } else {
        // received expected length
        if (chksum == 0) {
          // good payload
          return true;
        } else {
          // bad checksum
          state = IDLE;
        }
      }
      break;
    }
  }
  
  return false;
}

/***************************************************************************************************************
* SETUP/LOOP
***************************************************************************************************************/

void setup()
{
  // clear wd reset bit and disable wdt in case avrootloader enabled it
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(115200L);  

  // init buttons
  button_init();
  
  // init lcd
  lcd.begin(16, 2);
  lcd.clear();    
  lcd.blink();
  lcd.setCursor(0, 0);
  lcd.print("Open FlightStab");
  lcd.setCursor(0, 1);
  lcd.print("Programming Box");

  delay(1000);

#if 0  
  for (int8_t i=0; i<7; i++) {
    lcd.clear();    
    lcd.setCursor(0, 0);
    lcd.print((const __FlashStringHelper*) pgm_read_word(&menu_heading[i]));
    for (int8_t j=0; j<menu_count[i]; j++) {
      lcd.setCursor(0, 1);
      lcd.print((const __FlashStringHelper*) pgm_read_word(&menu_item[i][j]));
      lcd.print("................");
      delay(2000);
    }
  }
#endif
  
}

int16_t button_count[] = {0, 0, 0, 0};

struct _ow_msg ow_msg;

void loop()
{
  uint32_t t = micros();
  uint32_t last_msg_time = t;
  uint32_t last_lcd_time = t;
  int16_t sync_ok=0, sync_err=0;
  
again:
  t = micros();

  if ((int32_t)(t - last_msg_time) > 500000L) {
    ow_msg.cmd = OW_SYNC;
    send_msg(&ow_msg, sizeof(ow_msg));
    if (recv_msg(&ow_msg, sizeof(ow_msg), 1000)) {
      sync_ok++;
    } else {
      sync_err++;
    }  
    last_msg_time = t;
  }
  
  if ((int32_t)(t - last_lcd_time) > 1000000L) {
    lcd.clear();    
    lcd.setCursor(0, 0);
    lcd.print("ok=");
    lcd.print(sync_ok);
    lcd.print(" err=");
    lcd.print(sync_err);
    last_lcd_time = t;
  }
  
#if 0  
  if (button_read()) {
    for (int8_t i=0; i<4; i++) {

      if (button_event[i] == BUTTON_PRESS) {
        button_count[i]++;

        lcd.clear();    
        lcd.setCursor(0, 0);
        lcd.print("button=");
        lcd.print(i);
        lcd.print(" state=");
        lcd.print(button_state[i]);
        lcd.setCursor(0, 1);
        lcd.print("count=");
        lcd.print(button_count[i]);
        
        Serial.print("button=");        
        Serial.print(i);
        Serial.print(" state=");
        Serial.print(button_state[i]);
        Serial.print(" count=");
        Serial.println(button_count[i]); 
      }
    }
  }
#endif
  
  
  goto again;
}