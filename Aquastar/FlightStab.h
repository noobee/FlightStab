/* FlightStab **************************************************************************************************/

/* NOTE:
./Aquastar/FlightStab.h and ./FlightStab/FlightStab.h must be identical at all times for the 
aquastar programming box to be in sync with the flightstab firmware.
*/

#if !defined(FLIGHTSTAB_H)
#define FLIGHTSTAB_H

const int8_t eeprom_cfg_ver = 5;

enum DEVICE_IDS {DEVICE_UNDEF, DEVICE_RX3S_V1, DEVICE_RX3S_V2V3, DEVICE_NANOWII, DEVICE_EAGLE_A3PRO, DEVICE_RX3SM, DEVICE_MINI_MWC};

enum WING_MODE {WING_USE_DIPSW=1, WING_SINGLE_AIL, WING_DELTA, WING_VTAIL, WING_DUAL_AIL};
enum MIXER_EPA_MODE {MIXER_EPA_FULL=1, MIXER_EPA_NORM, MIXER_EPA_TRACK};
enum SERIALRX_ORDER {SERIALRX_NONE=1, SERIALRX_RETA1a2F, SERIALRX_TAER1a2F, SERIALRX_AETR1a2F};
enum MOUNT_ORIENT {MOUNT_NORMAL=1, MOUNT_ROLL_90_LEFT, MOUNT_ROLL_90_RIGHT};
enum STICK_GAIN_THROW {STICK_GAIN_THROW_FULL=1, STICK_GAIN_THROW_HALF=2, STICK_GAIN_THROW_QUARTER=3}; // values are important
enum MAX_ROTATE {MAX_ROTATE_VLOW=1, MAX_ROTATE_LOW=2, MAX_ROTATE_MED=3, MAX_ROTATE_HIGH=4, MAX_ROTATE_VHIGH=5}; // values are important
enum RATE_MODE_STICK_ROTATE {RATE_MODE_STICK_ROTATE_DISABLE=1, RATE_MODE_STICK_ROTATE_ENABLE};
enum INFLIGHT_CALIBRATE {INFLIGHT_CALIBRATE_DISABLE=1, INFLIGHT_CALIBRATE_ENABLE};

struct _eeprom_stats {
  int8_t device_id;
  uint32_t device_ver;
  int8_t eeprom_cfg1_err;
  int8_t eeprom_cfg2_err;
  int8_t eeprom_cfg12_reset;
};

struct _pid_param {
  int16_t kp[3]; // [0, 1000] 11b signed
  int16_t ki[3];
  int16_t kd[3];
  int8_t output_shift;
};

struct _eeprom_cfg {
  uint8_t ver;
  enum WING_MODE wing_mode; // overridden by DIP switches if available
  enum MIXER_EPA_MODE mixer_epa_mode;
  enum SERIALRX_ORDER serialrx_order; 
  enum MOUNT_ORIENT mount_orient;
  enum STICK_GAIN_THROW stick_gain_throw;
  enum MAX_ROTATE max_rotate;
  enum RATE_MODE_STICK_ROTATE rate_mode_stick_rotate;
  enum INFLIGHT_CALIBRATE inflight_calibrate;
  int8_t vr_gain[3];  
  struct _pid_param pid_param_rate;
  struct _pid_param pid_param_hold;
  uint8_t chksum;
};

const int8_t vr_gain_use_pot = -128;

enum OW_COMMAND {OW_NULL, OW_GET_STATS, OW_SET_STATS, OW_GET_CFG, OW_SET_CFG};

struct _ow_msg {
  uint8_t cmd;
  union {
    struct _eeprom_cfg eeprom_cfg; // OW_*_CFG
    struct _eeprom_stats eeprom_stats; // OW_*_STATS
  } u;
  uint8_t pad[32]; // pad to accommodate new ver cfg larger than existing one
};
 
#endif // FLIGHTSTAB_H