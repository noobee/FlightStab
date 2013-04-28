/* FlightStab **************************************************************************************************/

#include <Arduino.h>

// ONE-WIRE SERIAL


const uint8_t ow_pin = 12;

uint8_t ow_mask;
volatile uint8_t *ow_mode_reg;
volatile uint8_t *ow_port_reg;
volatile uint8_t *ow_pin_reg;

enum OW_COMMAND {OW_SYNC, OW_GET_CFG, OW_SET_CFG};
const char *ow_signature = "flightstab";


/***************************************************************************************************************
* ONE-WIRE SERIAL
***************************************************************************************************************/

// 16mhz clock
// 1/115200 == 139 ticks

const uint16_t ow_pulse_width_ticks = 139;

void ow_write(uint8_t b) 
{
  uint16_t w = (0xff00 | (uint16_t) b) << 1;
  uint8_t oldSREG = SREG;
  cli();  
  uint16_t t = TCNT1 + 16*4; // fixed 4us later to allow time to first while() check
  for (int8_t i=0; i<10; i++) {
    while ((int16_t)(t - TCNT1) > 0);
    if (w & 1) {
      *ow_mode_reg &= ~ow_mask; // input mode, external pull up to 1
    } else {
      *ow_mode_reg |= ow_mask; // output mode, internal pull down to 0
    }
    w >>= 1;
    t += ow_pulse_width_ticks;
  }
  SREG = oldSREG;
}

// return 0x8000 = timeout, 0x4000 = link down on entry, 0x00XX = received ch
uint16_t ow_read(int16_t timeout_ms) 
{
  uint16_t w, t;
  uint8_t oldSREG = SREG;
  cli();  
  do {

    // check for link already down on entry
    if (!(*ow_pin_reg & ow_mask)) {
      w = 0x4000;
      break;
    }

    // wait for start edge, or time out in timeout_ms
    while (timeout_ms-- > 0) {
      t = TCNT1 + 16000; // 1ms interval from current time
      while ((int16_t)(t - TCNT1) > 0) {
        if (!(*ow_pin_reg & ow_mask)) {
          // found start edge (high to low)
          timeout_ms = -1;
          break;
        }
      }
    }

    // timeout_ms == -1 if time out, -2 if found start pulse edge
    if (timeout_ms == -1) {
      w = 0x8000;
      break;
    }
    
    // delay 1.5 pulse widths - 3us, then read 8 bits consecutively
    w = 0;
    t = TCNT1 + ow_pulse_width_ticks*3/2 - 16*3;
    for (int8_t i=0; i<8; i++) {
      w >>= 1;
      while ((int16_t)(t - TCNT1) > 0);
      //PORTB ^= (1 << 5);
      w |= (*ow_pin_reg & ow_mask) ? 0x80 : 0;
      t += ow_pulse_width_ticks;
    }
  } while (0);
  SREG = oldSREG;
  return w;
}

void ow_init()
{
  // uses timer 1 at full clock speed (16mhz)
  TCCR1A = 0; // normal counting mode
  TCCR1B = (1 << CS10); // clkio

  ow_mask = digitalPinToBitMask(ow_pin);
  uint8_t port = digitalPinToPort(ow_pin);
  ow_mode_reg = portModeRegister(port);
  ow_port_reg = portOutputRegister(port);
  ow_pin_reg = portInputRegister(port);
    
  // set to [input mode + NO pull up]
  // require external pull up to HIGH. setting to output mode alone will pull pin down to LOW
  *ow_mode_reg &= ~ow_mask;
  *ow_port_reg &= ~ow_mask;
}

struct _ow_msg {
  uint8_t cmd;
  union {
    int8_t device_id; // OW_SYNC response
    // pid param
    // servo ?
  } u;
};


// returns true on good payload, false on timeout or link down

bool ow_recv_msg(void *buf, int8_t buf_len, int16_t timeout_ms) {
  enum OW_STATE {IDLE, HEADER, LENGTH} state = IDLE;
  
  while (true) {
    uint8_t ch, len, chksum, i;
    uint16_t w;

    w = ow_read(timeout_ms);
    if (w & 0xc000) {
      return false;
    }
    ch = w & 0xff;

    switch (state) {
    case IDLE: 
      if (ch == '$') {
        state = HEADER; // found HEADER
      }
      break;
    case HEADER:
      if (2 < ch && ch < buf_len) {
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
      ((uint8_t *)buf)[i++] = ch;
      chksum += ch;
      if (--len == 0) {
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
}

bool ow_send_msg(void *buf, int8_t buf_len) {
  uint8_t chksum=0;
  ow_write('$');
  ow_write(buf_len + 1); // +1 for checksum
  for (int8_t i=0; i<buf_len; i++) {
    chksum += ((uint8_t *)buf)[i];    
    ow_write(((uint8_t *)buf)[i]);
  }
  ow_write((chksum ^ 0xff) + 1);
}



void ow_loop() {
  struct _ow_msg ow_msg;
  
  while (true) {
    bool synced = false;
    
    if (!ow_recv_msg(&ow_msg, sizeof(ow_msg), synced ? 1100 : 500))
      break; 

    switch(ow_msg.cmd) {
    case OW_SYNC:
      synced = true;
      ow_msg.u.device_id = 0xa5;
      break;
    case OW_GET_CFG:
      break;
    case OW_SET_CFG:
      break;
    }

    ow_msg.cmd != 0x80;
    ow_send_msg(&ow_msg, sizeof(ow_msg));
  }

}