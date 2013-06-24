/* FlightStab **************************************************************************************************/

/* NOTE:
./Aquastar/FlightStab.h and ./FlightStab/FlightStab.h must be identical at all times for the 
aquastar programming box to be in sync with the flightstab firmware.
*/

#if !defined(FLIGHTSTAB_H)
#define FLIGHTSTAB_H

const int8_t eeprom_cfg_ver = 4;

enum DEVICE_IDS {DEVICE_UNDEF, DEVICE_RX3S_V1, DEVICE_RX3S_V2V3, DEVICE_NANOWII};

enum WING_MODE {WING_SINGLE_AIL=1, WING_DELTA, WING_VTAIL, WING_DUAL_AIL};
enum MIXER_EPA_MODE {MIXER_EPA_FULL=1, MIXER_EPA_NORM, MIXER_EPA_TRACK};
enum CPPM_MODE {CPPM_NONE=1, CPPM_RETA1a2F, CPPM_TAER1a2F, CPPM_AETR1a2F};
enum MOUNT_ORIENT {MOUNT_NORMAL=1, MOUNT_ROLL_90_LEFT, MOUNT_ROLL_90_RIGHT};
enum STICK_GAIN_THROW {STICK_GAIN_THROW_FULL=1, STICK_GAIN_THROW_HALF=2, STICK_GAIN_THROW_QUARTER=3};
enum MAX_ROTATE {MAX_ROTATE_49=1, MAX_ROTATE_98=2, MAX_ROTATE_196=3, MAX_ROTATE_391=4, MAX_ROTATE_782=5};

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
  int32_t i_limit[3];
  int8_t output_shift;
};

struct _eeprom_cfg {
  uint8_t ver;
  enum WING_MODE wing_mode; // overridden by DIP switches if available
  int8_t vr_notch[3]; // -4 to +4, will be deprecated
  bool vr_override_enable[3]; // true if use vr_override_gain[] to set vr_gain[]
  int8_t vr_override_gain[3];  
  enum MIXER_EPA_MODE mixer_epa_mode;
  enum CPPM_MODE cppm_mode; 
  enum MOUNT_ORIENT mount_orient;
  enum STICK_GAIN_THROW stick_gain_throw;
  enum MAX_ROTATE max_rotate;
  bool rate_mode_stick_rotate; // true if RATE mode also allows stick controlled rotation
  struct _pid_param pid_param_rate;
  struct _pid_param pid_param_hold;
  bool inflight_calibrate; // true to allow inflight rx calibration by toggling AUX between RATE/HOLD n times
  uint8_t chksum;
};


enum OW_COMMAND {OW_NULL, OW_GET_STATS, OW_SET_STATS, OW_GET_CFG, OW_SET_CFG};

struct _ow_msg {
  uint8_t cmd;
  union {
    struct _eeprom_cfg eeprom_cfg; // OW_*_CFG
    struct _eeprom_stats eeprom_stats; // OW_*_STATS
  } u;
};
 
#endif // FLIGHTSTAB_H