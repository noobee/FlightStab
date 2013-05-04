/* Aquastar ****************************************************************************************************/

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include "FlightStab.h"

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

// button pins
const uint8_t button_pin[] = {2, 3, 4, 5}; // up, left, right, down

// lcd pins
LiquidCrystal lcd(A2, A3, A4, 8, 9, 10, 11); // (rs, rw, enable, d4, d5, d6, d7)

// lcd custom characters
byte up_arrow[8] = {
  B00100,
  B01110,
  B10101,
  B00100,
  B00100,
  B00100,
  B00000,
};

byte down_arrow[8] = {
  B00000,
  B00100,
  B00100,
  B00100,
  B10101,
  B01110,
  B00100,
};

// string tables in flash

#define CHAR_DEGREE \xdf
#define CHAR_RIGHT_ARROW \x7e
#define STR(s) #s
#define XSTR(s) STR(s)

// device names
prog_char undef_device[] PROGMEM = "Undefined";
prog_char rx3s_v1[] PROGMEM = "RX3S V1";
prog_char rx3s_v2v3[] PROGMEM = "RX3S V2/V3";
prog_char nanowii[] PROGMEM = "NanoWii";

prog_char *device_name[] PROGMEM = { // must match enum DEVICE_IDS in eeprom_stats.device_id
  undef_device,
  rx3s_v1,
  rx3s_v2v3,
  nanowii
};

enum SCREEN_PAGES {
  STATUS_PAGE, 
  WING_MODE_PAGE, 
  ROLL_GAIN_PAGE, 
  PITCH_GAIN_PAGE, 
  YAW_GAIN_PAGE, 
  EPA_MODE_PAGE, 
  CPPM_MODE_PAGE, 
  MOUNT_ORIENT_PAGE, 
  HOLD_AXES_PAGE, 
  EEPROM_PAGE
};

// screen pages
prog_char status[] PROGMEM = "STATUS";
prog_char wing_mode[] PROGMEM = "WING MODE";
prog_char roll_gain[] PROGMEM = "ROLL GAIN";
prog_char pitch_gain[] PROGMEM = "PITCH GAIN";
prog_char yaw_gain[] PROGMEM = "YAW GAIN";
prog_char mixer_epa_mode[] PROGMEM = "EPA MODE";
prog_char cppm_mode[] PROGMEM = "CPPM MODE";
prog_char mount_orient[] PROGMEM = "MOUNT ORIENT";
prog_char hold_axes[] PROGMEM = "HOLD AXES";
prog_char eeprom[] PROGMEM = "EEPROM";

prog_char status_device_id[] PROGMEM = "ID=";
prog_char status_device_ver[] PROGMEM = "VER= ";
prog_char status_device_eeprom[] PROGMEM = "1/2/R= ";

prog_char wing_single_ail[] PROGMEM = "Single Aileron";
prog_char wing_delta[] PROGMEM = "Delta";
prog_char wing_vtail[] PROGMEM = "VTail";
prog_char wing_dual_ail[] PROGMEM = "Dual Ailerons";

prog_char mixer_epa_full[] PROGMEM = "Full 1000-2000";
prog_char mixer_epa_norm[] PROGMEM = "Norm 1100-1900";
prog_char mixer_epa_track[] PROGMEM = "Tracking";

prog_char cppm_none[] PROGMEM = "None";
prog_char cppm_RETA1a2f[] PROGMEM = "RETA1a2F";
prog_char cppm_TAER1a2F[] PROGMEM = "TAER1a2F";
prog_char cppm_AETR1a2F[] PROGMEM = "AETR1a2F";

prog_char mount_normal[] PROGMEM = "Normal";
prog_char mount_roll90left[] PROGMEM = "Roll 90" XSTR(CHAR_DEGREE) " Left";
prog_char mount_roll90right[] PROGMEM = "Roll 90" XSTR(CHAR_DEGREE) " Right";

prog_char hold_axes_aer[] PROGMEM = "[AER]";
prog_char hold_axes_ae_r[] PROGMEM = "[AE][R]";
prog_char hold_axes_a_e_r[] PROGMEM = "[A][E][R]";

prog_char eeprom_instr[] PROGMEM = "Up/Dn then " XSTR(CHAR_RIGHT_ARROW);
prog_char eeprom_update_cfg[] PROGMEM = "Update Config " XSTR(CHAR_RIGHT_ARROW);
prog_char eeprom_erase_cfg[] PROGMEM = "Erase Config " XSTR(CHAR_RIGHT_ARROW);
prog_char eeprom_erase_stats[] PROGMEM = "Erase Stats " XSTR(CHAR_RIGHT_ARROW);

prog_char *menu_heading[] PROGMEM = {
  status,
  wing_mode,
  roll_gain,
  pitch_gain,
  yaw_gain,
  mixer_epa_mode,
  cppm_mode,
  mount_orient,
  hold_axes,
  eeprom,
};

prog_char *menu_item[][5] PROGMEM = {
  {status_device_id,
   status_device_ver,
   status_device_eeprom},
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
   cppm_RETA1a2f,
   cppm_TAER1a2F,
   cppm_AETR1a2F},
  {mount_normal,
   mount_roll90left,
   mount_roll90right},
  {hold_axes_aer,
   hold_axes_ae_r,
   hold_axes_a_e_r},
  {eeprom_instr,
   eeprom_update_cfg,
   eeprom_erase_cfg,
   eeprom_erase_stats},
};

const int8_t menu_count[] = {3, 4, 0, 0, 0, 3, 4, 3, 3, 4};
const int8_t param_min[] = {1, WING_SINGLE_AIL, -4, -4, -4, MIXER_EPA_FULL,  CPPM_NONE,     MOUNT_NORMAL,        HOLD_AXES_AER,   1};
const int8_t param_max[] = {3, WING_DUAL_AIL,   +4, +4, +4, MIXER_EPA_TRACK, CPPM_AETR1a2F, MOUNT_ROLL_90_RIGHT, HOLD_AXES_A_E_R, 4};

/***************************************************************************************************************
* BUTTONS
***************************************************************************************************************/

enum BUTTON_ID {BUTTON_ID_UP, BUTTON_ID_LEFT, BUTTON_ID_RIGHT, BUTTON_ID_DOWN};
enum BUTTON_STATE {BUTTON_UP, BUTTON_DOWN, BUTTON_DOWN_AUTO};
enum BUTTON_EVENT {BUTTON_NONE, BUTTON_RELEASE, BUTTON_PRESS};
BUTTON_STATE button_state[] = {BUTTON_UP, BUTTON_UP, BUTTON_UP, BUTTON_UP};
BUTTON_EVENT button_event[] = {BUTTON_NONE, BUTTON_NONE, BUTTON_NONE, BUTTON_NONE};

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
* BITBANG SERIAL
***************************************************************************************************************/

void serial_write_byte(uint8_t b) {
  Serial.write(b);
  Serial.read(); // drain the echo received due to loopback
  delay(2); // delay for one-wire library on the target to ready the next receive
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
  
  while ((int32_t)(micros() - t) < (int32_t)timeout_ms * 1000L) {
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
* EEPROM CONFIG
***************************************************************************************************************/

void copy_from_cfg(int8_t *param, struct _eeprom_cfg *pcfg) {
  param[1] = (int8_t) pcfg->wing_mode;
  param[2] = pcfg->vr_notch[0];
  param[3] = pcfg->vr_notch[1];
  param[4] = pcfg->vr_notch[2];
  param[5] = (int8_t) pcfg->mixer_epa_mode;
  param[6] = (int8_t) pcfg->cppm_mode;
  param[7] = (int8_t) pcfg->mount_orient;
}

void copy_to_cfg(int8_t *param, struct _eeprom_cfg *pcfg) {
  pcfg->wing_mode = (WING_MODE) param[1];
  pcfg->vr_notch[0] = param[2];
  pcfg->vr_notch[1] = param[3];
  pcfg->vr_notch[2] = param[4];
  pcfg->mixer_epa_mode = (MIXER_EPA_MODE) param[5];
  pcfg->cppm_mode = (CPPM_MODE) param[6];
  pcfg->mount_orient = (MOUNT_ORIENT) param[7];
}

/***************************************************************************************************************
* SETUP/LOOP
***************************************************************************************************************/

void setup()
{
  // clear wd reset bit and disable wdt in case avrootloader enabled it
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  Serial.begin(115200L);  

  // init buttons
  button_init();
  
  // init lcd
  lcd.begin(16, 2);
  lcd.blink();
  lcd.createChar(0, up_arrow);
  lcd.createChar(1, down_arrow);
}

void loop()
{
  uint32_t t = micros();
  uint32_t last_msg_time = t;

  enum OW_STATE {
    OW_WAIT_CONNECT,
    OW_WAIT_STATS,
    OW_CONNECTED,
    OW_SEND
  } ow_state = OW_WAIT_CONNECT;

  struct _ow_msg ow_msg;
  struct _eeprom_stats eeprom_stats;

  int8_t param[sizeof(menu_count)/sizeof(menu_count[0])];
  int8_t page;
  bool update_lcd = true;
  
again:
  t = micros();

  // connection state machine
  if ((int32_t)(t - last_msg_time) > 250000L) { // every 250ms

    switch (ow_state) {
    case OW_WAIT_CONNECT: 
      ow_msg.cmd = OW_GET_CFG; 
      break;
    case OW_WAIT_STATS:
      ow_msg.cmd = OW_GET_STATS;
      break;
    case OW_CONNECTED: 
      ow_msg.cmd = OW_NULL;
      break;
    case OW_SEND:
      // ow_msg already set up
      break;
    }
    send_msg(&ow_msg, sizeof(ow_msg));
    
    if (recv_msg(&ow_msg, sizeof(ow_msg), 200)) { // wait 200ms for response
      switch (ow_state) {
      case OW_WAIT_CONNECT:
        copy_from_cfg(param, &ow_msg.u.eeprom_cfg);
        ow_state = OW_WAIT_STATS;
        break;
      case OW_WAIT_STATS:
        eeprom_stats = ow_msg.u.eeprom_stats;
        ow_state = OW_CONNECTED;
        // reset screen menu
        page = STATUS_PAGE;
        param[STATUS_PAGE] = param_min[STATUS_PAGE];
        param[EEPROM_PAGE] = param_min[EEPROM_PAGE];
        update_lcd = true;
        break;    
      case OW_SEND:
        ow_state = OW_WAIT_CONNECT; // restart state machine
        update_lcd = true;
        break;    
      }
    } else {
      if (ow_state != OW_WAIT_CONNECT) {
        update_lcd = true;
      }
      ow_state = OW_WAIT_CONNECT; // reset to initial state
    }
    last_msg_time = t;
  }

  // screen update
  if (update_lcd) {
    int8_t id, v;
    struct _eeprom_stats *p = &eeprom_stats;
    id = p->device_id;
    if (id < DEVICE_UNDEF || id > DEVICE_NANOWII)
      id = DEVICE_UNDEF;

    update_lcd = false;
    lcd.clear();
   
    switch (ow_state) {
    case OW_WAIT_CONNECT:
      lcd.print(F("Open FlightStab"));
      lcd.setCursor(0, 1);
      lcd.print(F("Program Box"));
      break;
    case OW_CONNECTED:
      // heading
      lcd.print((const __FlashStringHelper*) pgm_read_word(&menu_heading[page]));
      lcd.setCursor(14, 0);
      lcd.write(param[page] > param_min[page] ? byte(0) : ' '); // up arrow
      lcd.write(param[page] < param_max[page] ? byte(1) : ' '); // down arrow

      lcd.setCursor(0, 1);
      switch (page) {
      case STATUS_PAGE:
        lcd.print((const __FlashStringHelper*) pgm_read_word(&menu_item[page][param[page] - 1]));
        switch (param[page]) {
        case 1: // device_id
          lcd.print(p->device_id);
          lcd.print(F(" "));
          lcd.print((const __FlashStringHelper*) pgm_read_word(&device_name[id]));
          break;
        case 2: // device_ver
          lcd.print(p->device_ver); 
          break;
        case 3: // eeprom err and reset
          lcd.print(p->eeprom_cfg1_err);
          lcd.print(F("/"));
          lcd.print(p->eeprom_cfg2_err);
          lcd.print(F("/"));
          lcd.print(p->eeprom_cfg12_reset);
          break;
        }
        break;
      case ROLL_GAIN_PAGE:
      case PITCH_GAIN_PAGE:  
      case YAW_GAIN_PAGE:
        v = param[page];
        if (v > 0) lcd.print(F("+"));
        if (v == 0) lcd.print(F(" "));
        lcd.print(v);
        lcd.print(v < 0 ? F(" (Reverse)") : v > 0 ? F(" (Forward)") : F(" (OFF)"));
        break;
      default: // all other pages
        if ((param[page] >= param_min[page]) && (param[page] <= param_max[page])) {
          lcd.print((const __FlashStringHelper*) pgm_read_word(&menu_item[page][param[page] - 1]));
        } else {
          lcd.print(F("Invalid! ")); // out of range
          lcd.print(param[page]);
        }
        break;
      }
      break;
    }
  }

  // button handler
  if ((ow_state == OW_CONNECTED) && button_read()) {
  
    if (button_event[BUTTON_ID_LEFT] == BUTTON_PRESS) {
      if (page > 0)
        page--;
    }
    if (button_event[BUTTON_ID_RIGHT] == BUTTON_PRESS) {
    
      if (page == EEPROM_PAGE) {
        switch (param[page]) {
        case 2: // update cfg
          copy_to_cfg(param, &ow_msg.u.eeprom_cfg);
          ow_msg.u.eeprom_cfg.ver = eeprom_cfg_ver;
          ow_msg.cmd = OW_SET_CFG;
          ow_state = OW_SEND; 
          break;
        case 3: // erase cfg
          ow_msg.u.eeprom_cfg.ver = 0; // invalidate ver
          ow_msg.cmd = OW_SET_CFG;
          ow_state = OW_SEND; 
          break;
        case 4: // erase stats
          ow_msg.u.eeprom_stats.device_id = 0; // invalidate device_id
          ow_msg.cmd = OW_SET_STATS;
          ow_state = OW_SEND; 
          break;
        }
      }
      if (page < sizeof(menu_count)/sizeof(menu_count[0])-1) {
        page++;
      }      
    }
    if (button_event[BUTTON_ID_UP] == BUTTON_PRESS) {
      if (param[page] > param_min[page])
        param[page]--;
    }
    if (button_event[BUTTON_ID_DOWN] == BUTTON_PRESS) {
      if (param[page] < param_max[page])
        param[page]++;
    }
    
    update_lcd = true;
  }
  goto again;
}