/* Aquastar ****************************************************************************************************/

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include "FlightStab.h"

#define USE_SERIAL_LIGHT // use lightweight serial routines (atmega8 115200 poll-based)

/*
 Aquastar/DLUX pin mapping
 PB0  8 LCD_D4         PC0 14/A0               PD0 0 RXD
 PB1  9 LCD_D5         PC1 15/A1               PD1 1 TXD
 PB2 10 LCD_D6         PC2 16/A2 LCD_RS        PD2 2 UP_BUTTON
 PB3 11 LCD_D7 (MOSI)  PC3 17/A3 LCD_RW        PD3 3 LEFT_BUTTON
 PB4 12 (MISO)         PC4 18/A4 LCD_EN (SDA)  PD4 4 RIGHT_BUTTON (DLUX=DOWN_BUTTON)
 PB5 13 (SCK)          PC5 19/A5 (SCL)         PD5 5 DOWN_BUTTON (DLUX=RIGHT_BUTTON)
 PB6 14 (XTAL1)        PC6 (RESET)             PD6 6 
 PB7 15 (XTAL2)                                PD7 7 
*/

// button pins
#if defined(AQUASTAR)
const uint8_t button_pin[] = {2, 3, 4, 5}; // up, left, right, down
#endif 
#if defined(DLUX)
const uint8_t button_pin[] = {2, 3, 5, 4}; // up, left, right, down
#endif 

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

// eeprom cfg and stats
struct _eeprom_cfg cfg;
struct _eeprom_stats stats;

// device names
prog_char undef_device[] PROGMEM = " Undefined";
prog_char rx3s_v1[] PROGMEM = " RX3S V1";
prog_char rx3s_v2v3[] PROGMEM = " RX3S V2/V3";
prog_char nanowii[] PROGMEM = " NanoWii";
prog_char eagle_a3pro[] PROGMEM = " Eagle A3Pro";
prog_char rx3sm[] PROGMEM = " RX3SM";
prog_char mini_mwc[] PROGMEM = " Mini MWC";

prog_char *device_name[] PROGMEM = { // must match enum DEVICE_IDS in eeprom_stats.device_id
  undef_device,
  rx3s_v1,
  rx3s_v2v3,
  nanowii,
  eagle_a3pro,
  rx3sm,
  mini_mwc
};

// status
prog_char status[] PROGMEM = "STATUS";
prog_char status_device_id[] PROGMEM = "ID=";
prog_char status_device_ver[] PROGMEM = "VER=";
prog_char status_device_eeprom[] PROGMEM = "1E/2E/R=";

// eeprom
prog_char eeprom[] PROGMEM = "EEPROM";
prog_char eeprom_instr[] PROGMEM = "Up/Dn then " XSTR(CHAR_RIGHT_ARROW);
prog_char eeprom_update_cfg[] PROGMEM = "Update Config " XSTR(CHAR_RIGHT_ARROW);
prog_char eeprom_erase_cfg[] PROGMEM = "Erase Config " XSTR(CHAR_RIGHT_ARROW);
prog_char eeprom_erase_stats[] PROGMEM = "Erase Stats " XSTR(CHAR_RIGHT_ARROW);

// config modes
prog_char wing_mode[] PROGMEM = "WING MODE";
prog_char wing_use_dipsw[] PROGMEM = "DIP SW";
prog_char wing_single_ail[] PROGMEM = "Single Ail";
prog_char wing_delta[] PROGMEM = "Delta";
prog_char wing_vtail[] PROGMEM = "V-Tail";
prog_char wing_dual_ail[] PROGMEM = "Dual Ail";

prog_char mixer_epa_mode[] PROGMEM = "EPA MODE";
prog_char mixer_epa_full[] PROGMEM = "Full 1000-2000";
prog_char mixer_epa_norm[] PROGMEM = "Norm 1100-1900";
prog_char mixer_epa_track[] PROGMEM = "Tracking";

prog_char serialrx_order[] PROGMEM = "SERIALRX ORDER";
prog_char serialrx_none[] PROGMEM = "None";
prog_char serialrx_RETA1a2f[] PROGMEM = "RETA1a2F";
prog_char serialrx_TAER1a2F[] PROGMEM = "TAER1a2F";
prog_char serialrx_AETR1a2F[] PROGMEM = "AETR1a2F";

prog_char mount_orient[] PROGMEM = "MOUNT ORIENT";
prog_char mount_normal[] PROGMEM = "Normal";
prog_char mount_roll90left[] PROGMEM = "Roll 90" XSTR(CHAR_DEGREE) " Left";
prog_char mount_roll90right[] PROGMEM = "Roll 90" XSTR(CHAR_DEGREE) " Right";

prog_char stick_gain_throw[] PROGMEM = "STK-GAIN THROW";
prog_char stick_gain_throw_full[] PROGMEM = "Full";
prog_char stick_gain_throw_half[] PROGMEM = "Half";
prog_char stick_gain_throw_quarter[] PROGMEM = "Quarter";

prog_char max_rotate[] PROGMEM = "MAX STK-ROTATE";
prog_char max_rotate_vlow[] PROGMEM = "Very Low 0.25x";
prog_char max_rotate_low[] PROGMEM = "Low 0.5x";
prog_char max_rotate_med[] PROGMEM = "Medium 1x";
prog_char max_rotate_high[] PROGMEM = "High 2x";
prog_char max_rotate_vhigh[] PROGMEM = "Very High 4x";

prog_char rate_mode_stick_rotate[] PROGMEM = "RATE STK-ROTATE";
prog_char disabled[] PROGMEM = "Disabled";
prog_char enabled[] PROGMEM = "Enabled";

prog_char inflight_calibrate[] PROGMEM = "INFLIGHT CALIB";

prog_char vr_gain[] PROGMEM = "VR-GAIN A/E/R";

prog_char pid_rate[] PROGMEM = "RATE ";
prog_char pid_hold[] PROGMEM = "HOLD ";
prog_char pid_roll[] PROGMEM = "Ail ";
prog_char pid_pitch[] PROGMEM = "Ele ";
prog_char pid_yaw[] PROGMEM = "Rud ";

prog_char *param_text[][6] PROGMEM = {
  {status, 
   status_device_id, status_device_ver, status_device_eeprom},
  {wing_mode, 
   wing_use_dipsw, wing_single_ail, wing_delta, wing_vtail, wing_dual_ail},
  {mixer_epa_mode, 
   mixer_epa_full, mixer_epa_norm, mixer_epa_track},
  {serialrx_order, 
   serialrx_none, serialrx_RETA1a2f, serialrx_TAER1a2F, serialrx_AETR1a2F},
  {mount_orient, 
   mount_normal, mount_roll90left, mount_roll90right},
  {stick_gain_throw, 
   stick_gain_throw_full, stick_gain_throw_half, stick_gain_throw_quarter},
  {max_rotate, 
   max_rotate_vlow, max_rotate_low, max_rotate_med, max_rotate_high, max_rotate_vhigh},
  {rate_mode_stick_rotate, 
   disabled, enabled},
  {inflight_calibrate, 
   disabled, enabled},
  {vr_gain,
   },
  {pid_rate,
   pid_roll, pid_pitch, pid_yaw},
  {pid_hold,
   pid_roll, pid_pitch, pid_yaw},
  {eeprom, 
   eeprom_instr, eeprom_update_cfg, eeprom_erase_cfg, eeprom_erase_stats}
};

const int8_t param_min[] = {
  1, // status
  WING_USE_DIPSW, 
  MIXER_EPA_FULL,  
  SERIALRX_NONE,     
  MOUNT_NORMAL,    
  STICK_GAIN_THROW_FULL,
  MAX_ROTATE_VLOW,
  RATE_MODE_STICK_ROTATE_DISABLE,
  INFLIGHT_CALIBRATE_DISABLE,
  0, // vr_gain
  0, // pid_rate
  0, // pid_hold
  1 // eeprom
};

const int8_t param_max[] = {
  3, // status
  WING_DUAL_AIL, 
  MIXER_EPA_TRACK,  
  SERIALRX_AETR1a2F,     
  MOUNT_ROLL_90_RIGHT,    
  STICK_GAIN_THROW_QUARTER,
  MAX_ROTATE_VHIGH,
  RATE_MODE_STICK_ROTATE_ENABLE,
  INFLIGHT_CALIBRATE_ENABLE,
  0, // vr_gain
  0, // pid_rate
  0, // pid_hold
  4 // eeprom
};
  
int8_t param_val_status = 1;
int8_t param_val_eeprom = 1;

const int8_t kpid_map[] = {0, 1, 2, 0, 1, 2, 0, 1, 2}; // index = subpage
const int8_t axis_map[] = {0, 0, 0, 1, 1, 1, 2, 2, 2}; // index = subpage

struct _pid_param_array {
  int16_t *param[3]; // param[kpid][axis] = value
};

struct _pid_param_array pid_param_array_rate = {{cfg.pid_param_rate.kp, cfg.pid_param_rate.ki, cfg.pid_param_rate.kd}};
struct _pid_param_array pid_param_array_hold = {{cfg.pid_param_hold.kp, cfg.pid_param_hold.ki, cfg.pid_param_hold.kd}};  

int8_t * const param_pval[] = {
  &param_val_status,
  (int8_t*) &cfg.wing_mode,
  (int8_t*) &cfg.mixer_epa_mode,
  (int8_t*) &cfg.serialrx_order,
  (int8_t*) &cfg.mount_orient,
  (int8_t*) &cfg.stick_gain_throw,
  (int8_t*) &cfg.max_rotate,
  (int8_t*) &cfg.rate_mode_stick_rotate,
  (int8_t*) &cfg.inflight_calibrate,
  &cfg.vr_gain[0],
  (int8_t*) &pid_param_array_rate,
  (int8_t*) &pid_param_array_hold,
  &param_val_eeprom,
};
  
enum PARAM_TYPE {PARAM_STATUS, PARAM_ENUM, PARAM_VR_GAIN, PARAM_PID, PARAM_EEPROM};

enum PARAM_TYPE param_type[] = {
  PARAM_STATUS,
  PARAM_ENUM,
  PARAM_ENUM,
  PARAM_ENUM,
  PARAM_ENUM,
  PARAM_ENUM,
  PARAM_ENUM,
  PARAM_ENUM,
  PARAM_ENUM,
  PARAM_VR_GAIN,
  PARAM_PID,
  PARAM_PID,
  PARAM_EEPROM,
};

const int8_t param_num_pages = sizeof(param_type)/sizeof(param_type[0]);


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
  const int32_t button_auto_delay = 600000; // time between first press and auto repeat start
  const int32_t button_auto_period = 100000; // time between auto repeat events
  
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
* ATMEGA8 USART 115200 8N1 POLL-BASED
***************************************************************************************************************/

#if defined(USE_SERIAL_LIGHT)
bool serial2_available()
{
  return UCSRA & (1 << RXC); // check for data
}

uint8_t serial2_read()
{
  while (!serial2_available());
  return UDR;
}

void serial2_write(uint8_t b)
{
  while (!(UCSRA & (1 << UDRE))); // wait for ready
  UDR = b;
}

void serial2_init()
{
//  PORTD |= 0x03; // enable pullup
  UBRRH = 0; // 16 with U2X enabled means 115200 baud with 16MHz CPU: 16000000 / 115200 / 8 - 1 == 16
  UBRRL = 16; // seems to have too much error without U2X.
  UCSRA |= (1 << U2X);
  UCSRB = (1 << RXEN) | (1 << TXEN);
  UCSRC = (1 << URSEL) |(1 << UCSZ1) | (1 << UCSZ0); // 8N1
}
#endif

/***************************************************************************************************************
* MESSAGE TRANSPORT
***************************************************************************************************************/

void serial_write_byte(uint8_t b, bool last_byte) {
#if defined(USE_SERIAL_LIGHT)
  serial2_write(b);
  serial2_read();
#else
  Serial.write(b);
  Serial.read(); // drain the echo received due to loopback
#endif
  if (!last_byte)
    delay(2); // delay for one-wire library on the target to ready the next receive
}

bool send_msg(void *buf, int8_t buf_len) {
  uint8_t chksum = buf_len;
  serial_write_byte('$', false);
  serial_write_byte(buf_len, false);
  for (int8_t i=0; i<buf_len; i++) {
    chksum += ((uint8_t *)buf)[i];
    serial_write_byte(((uint8_t *)buf)[i], false);
  }
  serial_write_byte((chksum ^ 0xff) + 1, true);
}

bool recv_msg(void *buf, int8_t buf_len, int16_t timeout_ms) {
  enum RECV_STATE {IDLE, HEADER, LENGTH} state = IDLE;
  uint32_t t = micros();
  
  while ((int32_t)(micros() - t) < (int32_t)timeout_ms * 1000L) {
    uint8_t ch, len, chksum, i;

#if defined(USE_SERIAL_LIGHT)
    if (!serial2_available())
      continue;
    ch = serial2_read();    
#else
    if (!Serial.available())
      continue;
    ch = Serial.read();    
#endif
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
#if defined(USE_SERIAL_LIGHT)
  serial2_init();
#else
  Serial.begin(115200L);  
#endif

  // init buttons
  button_init();
  
  // init lcd
  lcd.begin(16, 2);
  lcd.blink();
#if 0
  lcd.createChar(0, up_arrow);
  lcd.createChar(1, down_arrow);
#endif

#if 0  
  for (int8_t i=0; i<3; i++) {
    cfg.pid_param_rate.kp[i] = 500;
    cfg.pid_param_rate.ki[i] = 500;
    cfg.pid_param_rate.kd[i] = 500;
    cfg.pid_param_hold.kp[i] = 1000;
    cfg.pid_param_hold.ki[i] = 1000;
    cfg.pid_param_hold.kd[i] = 1000;
    cfg.vr_gain[i] = -127;
  }  
  return;
  
again:
  for (int8_t i=0; i<param_num_pages; i++) {
    for (int8_t j=param_min[i]; j<=param_max[i]; j++) {
      lcd.clear();
      lcd.print((const __FlashStringHelper*) pgm_read_word(&param_text[i][0]));
      lcd.setCursor(0, 1);
      lcd.print((const __FlashStringHelper*) pgm_read_word(&param_text[i][j]));
      delay(1000);
    }
  }
  
  goto again;
#endif
}

void param_lcd_update(int8_t page, int8_t subpage) 
{
  int8_t v;
  int8_t kpid, axis;
  struct _pid_param_array *ppid;
  int8_t *pvr_gain;

  // heading
  lcd.print((const __FlashStringHelper*) pgm_read_word(&param_text[page][0]));
#if 0
  lcd.setCursor(14, 0);
  lcd.write(*param_pval[page] > param_min[page] ? byte(0) : ' '); // up arrow
  lcd.write(*param_pval[page] < param_max[page] ? byte(1) : ' '); // down arrow
#endif
  lcd.setCursor(0, 1);
  
  switch (param_type[page]) {
  case PARAM_STATUS:
    v = *param_pval[page];
    lcd.print((const __FlashStringHelper*) pgm_read_word(&param_text[page][v]));
    switch (v) {
    case 1: // device_id
      lcd.print(stats.device_id);
      lcd.print((const __FlashStringHelper*) pgm_read_word(&device_name[stats.device_id]));
      break;
    case 2: // device_ver
      lcd.print(stats.device_ver); 
      break;
    case 3: // eeprom err and reset
      lcd.print(stats.eeprom_cfg1_err);
      lcd.print('/');
      lcd.print(stats.eeprom_cfg2_err);
      lcd.print('/');
      lcd.print(stats.eeprom_cfg12_reset);
      break;
    }
    break;
  
  case PARAM_ENUM:
  case PARAM_EEPROM:
    v = *param_pval[page];
    if ((v >= param_min[page]) && (v <= param_max[page])) {
      lcd.print((const __FlashStringHelper*) pgm_read_word(&param_text[page][v]));
    } else {
      lcd.print(F("Inval ")); // out of range
      lcd.print(v);
    }
    break;

  case PARAM_VR_GAIN:
    pvr_gain = (int8_t *)param_pval[page];
    for (int8_t i=0; i<3; i++) {
      bool sel = (i == kpid_map[subpage]);
      if (sel) lcd.print('[');
      if (pvr_gain[i] == vr_gain_use_pot)
        lcd.print(F("POT"));
      else 
        lcd.print(pvr_gain[i]);      
      if (sel) lcd.print(']');
      if (i< 2)
        lcd.print('/');
    }
    break;
    
  case PARAM_PID:
    axis = axis_map[subpage];
    kpid = kpid_map[subpage];
    ppid = (struct _pid_param_array *)param_pval[page];
    lcd.clear();
    lcd.print((const __FlashStringHelper*) pgm_read_word(&param_text[page][0]));
    lcd.print((const __FlashStringHelper*) pgm_read_word(&param_text[page][axis + 1]));
    lcd.print(F("P/I/D"));
    lcd.setCursor(0, 1);
    for (int8_t i=0; i<3; i++) {
      bool sel = (i == kpid);
      if (sel) lcd.print('[');  
      lcd.print(ppid->param[i][axis]);
      if (sel) lcd.print(']');
      if (i< 2)
        lcd.print('/');
    }
    break;
  }
}


void loop()
{
  uint32_t t = micros();
  uint32_t last_msg_time = t;

  enum OW_STATE {
    OW_WAIT_CONNECT,
    OW_WAIT_STATS,
    OW_CONNECTED,
    OW_SEND,
    OW_BAD_CFG_VER
  } ow_state = OW_WAIT_CONNECT;

  struct _ow_msg ow_msg;
  int8_t ow_send_len;

  int8_t page=0, subpage=0;
  bool update_lcd = true;
  
again:
  t = micros();

  // connection state machine
  if ((int32_t)(t - last_msg_time) > (ow_state == OW_WAIT_CONNECT ? 50000L : 250000L)) { // 50ms to connect, else every 250ms

    int8_t msg_len = 1; // + 1 for ow_msg.cmd
    switch (ow_state) {
      case OW_WAIT_CONNECT: ow_msg.cmd = OW_GET_CFG; break;
      case OW_WAIT_STATS: ow_msg.cmd = OW_GET_STATS; break;
      case OW_CONNECTED: ow_msg.cmd = OW_NULL; break;
      case OW_SEND: msg_len += ow_send_len; break; // ow_msg already set up
      case OW_BAD_CFG_VER: ow_msg.cmd = OW_NULL; break;
    }
    send_msg(&ow_msg, msg_len);
    uint8_t resp_cmd = ow_msg.cmd | 0x80;
    
    if (recv_msg(&ow_msg, sizeof(ow_msg), 20) && (ow_msg.cmd == resp_cmd)) { // wait 20ms for response
      switch (ow_state) {
      case OW_WAIT_CONNECT:
        if (ow_msg.u.eeprom_cfg.ver != eeprom_cfg_ver) {
          ow_state = OW_BAD_CFG_VER;
          update_lcd = true;
          break;
        }      
        cfg = ow_msg.u.eeprom_cfg;
        ow_state = OW_WAIT_STATS;
        break;
      case OW_WAIT_STATS:
        stats = ow_msg.u.eeprom_stats;
        ow_state = OW_CONNECTED;
        page = subpage = 0;
        update_lcd = true;
        break;    
      case OW_SEND:
        ow_state = OW_WAIT_CONNECT; // restart state machine
        update_lcd = true;
        break;    
      }
    } else {
      // TODO: separate timeout from msg_err (eg. too large)
      if (ow_state != OW_WAIT_CONNECT) {
        update_lcd = true;
      }
      ow_state = OW_WAIT_CONNECT; // reset to initial state
    }
    last_msg_time = t;
  }

  if (update_lcd) {
    lcd.clear();
  
    switch (ow_state) {
    case OW_WAIT_CONNECT:
      lcd.print(F("OpenFlightStab"));
      lcd.setCursor(0, 1);
      lcd.print(F("CfgVer="));
      lcd.print(eeprom_cfg_ver);
//      lcd.print(F(" "));
//      lcd.print(sizeof(cfg));
      break;
    case OW_CONNECTED:
      param_lcd_update(page, subpage);
      break;
    case OW_BAD_CFG_VER:
      lcd.print(F("CfgVer Mismatch"));
      lcd.setCursor(0, 1);
      lcd.print(F("read="));
      lcd.print(ow_msg.u.eeprom_cfg.ver);
      lcd.print(F(" need="));
      lcd.print(eeprom_cfg_ver);
      break;
    }
    
    update_lcd = false;
  }

  // button handler
  if ((ow_state == OW_CONNECTED) && button_read()) {
    
    if (button_event[BUTTON_ID_LEFT] == BUTTON_PRESS) {
      enum PARAM_TYPE ptype = param_type[page];
      bool page_change = false;    

      if ((ptype == PARAM_VR_GAIN || ptype == PARAM_PID) && subpage > 0) {
        subpage--;
      } else {
        if (page > 0) {
          page--;
          if (param_type[page] == PARAM_VR_GAIN) 
            subpage = 2;
          else if (param_type[page] == PARAM_PID) 
            subpage = 8;
        }
      }      
    }
    
    if (button_event[BUTTON_ID_RIGHT] == BUTTON_PRESS) {
      enum PARAM_TYPE ptype = param_type[page];
      bool page_change = false;    
    
      if (ptype == PARAM_EEPROM) {
        switch (param_val_eeprom) {
        case 2: // update cfg
          ow_msg.u.eeprom_cfg = cfg; // copy cfg to message buffer
          ow_msg.cmd = OW_SET_CFG;
          ow_state = OW_SEND;
          ow_send_len = sizeof(ow_msg.u.eeprom_cfg);
          break;
        case 3: // erase cfg
          ow_msg.u.eeprom_cfg.ver = 0; // invalidate ver
          ow_msg.cmd = OW_SET_CFG;
          ow_state = OW_SEND; 
          ow_send_len = sizeof(ow_msg.u.eeprom_cfg);
          break;
        case 4: // erase stats
          ow_msg.u.eeprom_stats.device_id = 0; // invalidate device_id
          ow_msg.cmd = OW_SET_STATS;
          ow_state = OW_SEND; 
          ow_send_len = sizeof(ow_msg.u.eeprom_stats);
          break;
        }
      } else if ((ptype == PARAM_VR_GAIN && subpage < 2) || 
                 (ptype == PARAM_PID && subpage < 8)) {
        subpage++;
      } else {
        if (page < param_num_pages - 1) {
          page++;
          subpage = 0;
          // reset to safe option (1=eeprom_instr) when paging into eeprom page.
          // prevents user autorepeating right through it
          if (param_type[page] == PARAM_EEPROM)
            param_val_eeprom = 1;
        }
      }
    }
   
    int16_t diff = 0;
    int16_t diff2 = param_type[page] == PARAM_PID ? 10 : 1;

    if (button_event[BUTTON_ID_UP] == BUTTON_PRESS) {
      diff = diff2; // up to increase
    }
    if (button_event[BUTTON_ID_DOWN] == BUTTON_PRESS) {
      diff = -diff2; // down to decrease
    }

    if (diff != 0) {
      int8_t axis;
      int8_t kpid;
      int8_t *pv;
      struct _pid_param_array *ppid;
      
      switch (param_type[page]) {
        case PARAM_VR_GAIN:
          kpid = kpid_map[subpage];
          pv = (int8_t *)param_pval[page];
          pv[kpid] = constrain(pv[kpid] + diff, -128, 127);
          break;
        case PARAM_PID:
          axis = axis_map[subpage];
          kpid = kpid_map[subpage];
          ppid = (struct _pid_param_array *)param_pval[page];
          ppid->param[kpid][axis] = constrain(ppid->param[kpid][axis] + diff, 0, 1000);
          break;
        default:
          *param_pval[page] = constrain(*param_pval[page] + diff, param_min[page], param_max[page]);
          break;
      }
    }
    
    update_lcd = true;
  }

  goto again;  
}
