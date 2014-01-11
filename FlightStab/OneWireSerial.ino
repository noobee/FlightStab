/* FlightStab **************************************************************************************************/

/***************************************************************************************************************
* ONE-WIRE SERIAL
***************************************************************************************************************/

// required OW_* definitions
//#define OW_BIT ?
//#define OW_DDR DDR?
//#define OW_PORT PORT?
//#define OW_PINREG PIN?
//#define OW_FIRST_CONNECT_WAIT mmm	// Time in Milliseconds
//#define OW_HEARTBEAT_WAIT mmm		// Time in Milliseconds

#if !defined(OW_FIRST_CONNECT_WAIT)
// TODO(noobee): check why 600 and not lower? (previously 100)
#define OW_FIRST_CONNECT_WAIT 600 // in ms
#endif

#if !defined(OW_HEARTBEAT_WAIT)
#define OW_HEARTBEAT_WAIT 500 // in ms
#endif

const uint16_t ow_pulse_width_ticks = 139; // 1/115200 at 16mhz == 139 ticks

void ow_write(uint8_t b) 
{
  uint16_t w = (0xff00 | (uint16_t) b) << 1; // insert start and stop bits around 'b'
  uint8_t oldSREG = SREG;
  cli();  
  uint16_t t = TCNT1 + 16*4; // fixed 4us later to allow time to first while() check
  for (int8_t i=0; i<10; i++) {
    while ((int16_t)(t - TCNT1) > 0);
    if (w & 1) {
      OW_DDR &= ~(1 << OW_BIT); // set input mode, external pull up to 1
    } else {
      OW_DDR |= (1 << OW_BIT); // set output mode, internal pull down to 0
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
    if (!(OW_PINREG & (1 << OW_BIT))) {
      w = 0x4000;
      break;
    }

    // wait for start edge, or time out in timeout_ms
    while (timeout_ms-- > 0) {
      t = TCNT1 + 16000; // 1ms interval from current time
      while ((int16_t)(t - TCNT1) > 0) {
        if (!(OW_PINREG & (1 << OW_BIT))) {
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
      w |= (OW_PINREG & (1 << OW_BIT)) ? 0x80 : 0;
      t += ow_pulse_width_ticks;
    }
  } while (0);
  SREG = oldSREG;
  return w;
}

bool ow_send_msg(void *buf, int8_t buf_len) {
  uint8_t chksum = buf_len;
  ow_write('$');
  ow_write(buf_len);
  for (int8_t i=0; i<buf_len; i++) {
    chksum += ((uint8_t *)buf)[i];    
    ow_write(((uint8_t *)buf)[i]);
  }
  ow_write((chksum ^ 0xff) + 1);
}

// returns true on good payload, false on timeout or link down
bool ow_recv_msg(void *buf, int8_t buf_len, int16_t timeout_ms) {
  enum OW_STATE {IDLE, HEADER, LENGTH} state = IDLE;
  
  while (true) {
    uint8_t ch, len, chksum, i;
    uint16_t w;

    w = ow_read(timeout_ms);
    if (w & 0xc000) {
      // Serial.print("bad_conn=");
      // Serial.println(w, HEX);
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
          // Serial.println("good payload");
          return true;
        } else {
          // bad checksum
          // Serial.println("bad chksum");
          state = IDLE;
        }
      }
      break;
    }
  }
}

bool ow_loop() {
again:  
  bool connected = false;
  bool done = false;
  
  // uses timer 1 at full clock speed (16mhz)
  TCCR1A = 0; // normal counting mode
  TCCR1B = (1 << CS10); // clkio

  // set to [input mode + NO pull up]
  // require external pull up to HIGH. setting to output mode alone will pull pin down to LOW
  OW_DDR &= ~(1 << OW_BIT);
  OW_PORT &= ~(1 << OW_BIT);
  
  do {
    struct _ow_msg ow_msg;
		// wait up to M ms for first connect, then up to N ms for each heartbeat
    if (!ow_recv_msg(&ow_msg, sizeof(ow_msg), !connected ? OW_FIRST_CONNECT_WAIT : OW_HEARTBEAT_WAIT)) { 
      break; 
    }
    connected = true;
    
    // Serial.print("cmd=");
    // Serial.println(ow_msg.cmd, HEX);

    int8_t msg_len = 1; // + 1 for ow_msg.cmd
    switch(ow_msg.cmd) {
    case OW_NULL:
      break;
    case OW_GET_STATS:
      eeprom_read_block(&ow_msg.u.eeprom_stats, (void *)eeprom_stats_addr, sizeof(ow_msg.u.eeprom_stats));
      msg_len += sizeof(ow_msg.u.eeprom_stats);
      break;
    case OW_SET_STATS:
      eeprom_write_block(&ow_msg.u.eeprom_stats, (void *)eeprom_stats_addr, sizeof(ow_msg.u.eeprom_stats));
      done = true;
      break;
    case OW_GET_CFG:
      eeprom_read_cfg(&ow_msg.u.eeprom_cfg, eeprom_cfg1_addr, eeprom_cfg_ver);
      msg_len += sizeof(ow_msg.u.eeprom_cfg);
      break;
    case OW_SET_CFG:
      eeprom_write_cfg(&ow_msg.u.eeprom_cfg, eeprom_cfg1_addr);
      eeprom_write_cfg(&ow_msg.u.eeprom_cfg, eeprom_cfg2_addr);
      done = true;
      break;
    }
    // send response
    ow_msg.cmd |= 0x80;
    ow_send_msg(&ow_msg, msg_len); 
    
  } while (!done);
  
  //goto again;
  
  return connected;
}
