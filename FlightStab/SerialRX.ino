/* FlightStab **************************************************************************************************/

/***************************************************************************************************************
*
* SERIAL RX
* see http://paparazzi.github.io/docs/latest/stm32_2subsystems_2radio__control_2spektrum__arch_8c_source.html
*
***************************************************************************************************************/

#if !defined(NO_SERIALRX)

HardwareSerial *pSerial = &Serial1; // TODO: hardcoded for NanoWii for now

volatile int16_t *serialrx_chan[8] = {&ele_in, &rud_in, &thr_in, &ail_in, &aux_in, &ailr_in, &aux2_in, &flp_in}; // ERTA1a2F

void serialrx_init()
{
  switch (serialrx_mode) {
  case SERIALRX_SPEKTRUM:
    pSerial->begin(115200L);
    break;
  case SERIALRX_SBUS:
    pSerial->begin(100000L);
    break;
  }
}

void serialrx_update()
{
  static uint8_t buf[25];
  static int8_t index = 0;
  static uint32_t last_rx_time;
  static int8_t rshift = 0;
  uint32_t t;

  while (pSerial->available()) {
    uint8_t ch = pSerial->read();
    
    switch (serialrx_mode) {
    case SERIALRX_SPEKTRUM:
      t = micros1();
      // we assume loop() calls to serialrx_update() in << 7ms intervals
      if ((int32_t)(t - last_rx_time) > 7000) {
        index = 0; // found pause before new frame, resync
      }
      last_rx_time = t;
      buf[index++] = ch;
      if (index >= 15) {
        if (buf[2] & 0x80 == 0x00) { 
          // single frame type or 1st frame of two, contains "transmitter type"
          rshift = (buf[1] & 0x10) ? 11 : 10; // 11 or 10 bit data
        }
        if (rshift > 0) {
          // 10 bit == f  0 c3 c2 c1  c0 d9 d8 d7 d6 d5 d4 d3 d2 d1 d0
          // 11 bit == f c3 c2 c1 c0 d10 d9 d8 d7 d6 d5 d4 d3 d2 d1 d0        
          for (int8_t i=2; i<2+2*7; i+=2) {
            uint16_t w = ((uint16_t)buf[i] << 8) | (uint16_t)buf[i+1];
            int8_t chan = (w >> rshift) & 0xf;
            if (chan < 8) {
              *serialrx_chan[chan] = (w << (11 - rshift)) & 0x7ff - 1024 + 1500; // scale to 11 bits 1024 +/- 684
              
            }          
          }
        }  
        index = 0;
      }
      break;

    case SERIALRX_SBUS:
      if (index == 0 && ch != 0x0f) { // SBUS_SYNCBYTE
        break;
      }
      buf[index++] = ch;
      if (index >= 25) {
        *serialrx_chan[0] = (((((uint16_t)buf[1]  >> 0) | ((uint16_t)buf[2]  << 8)) & 0x7ff) >> 1) + 988;
        *serialrx_chan[1] = (((((uint16_t)buf[2]  >> 3) | ((uint16_t)buf[3]  << 5)) & 0x7ff) >> 1) + 988; 
        *serialrx_chan[2] = (((((uint16_t)buf[3]  >> 6) | ((uint16_t)buf[4]  << 2) | ((uint16_t)buf[5] << 10)) & 0x7ff) >> 1) + 988; 
        *serialrx_chan[3] = (((((uint16_t)buf[5]  >> 1) | ((uint16_t)buf[6]  << 7)) & 0x7ff) >> 1) + 988; 
        *serialrx_chan[4] = (((((uint16_t)buf[6]  >> 4) | ((uint16_t)buf[7]  << 4)) & 0x7ff) >> 1) + 988; 
        *serialrx_chan[5] = (((((uint16_t)buf[7]  >> 7) | ((uint16_t)buf[8]  << 1) | ((uint16_t)buf[9] << 9)) & 0x7ff) >> 1) + 988;
        *serialrx_chan[6] = (((((uint16_t)buf[9]  >> 2) | ((uint16_t)buf[10] << 6)) & 0x7ff) >> 1) + 988; 
        *serialrx_chan[7] = (((((uint16_t)buf[10] >> 5) | ((uint16_t)buf[11] << 3)) & 0x7ff) >> 1) + 988;
        
        index = 0;
      }
      break;
    }
  }
}

#endif // !NO_SERIALRX