/* FlightStab **************************************************************************************************/

/* NOTE:
./Aquastar/FlightStab.h and ./FlightStab/FlightStab.h must be identical at all times for the 
aquastar programming box to be in sync with the flightstab firmware.
*/

#if !defined(FLIGHTSTAB_H)
#define FLIGHTSTAB_H

enum DEVICE_IDS {DEVICE_UNDEF, DEVICE_RX3S_V1, DEVICE_RX3S_V2V3, DEVICE_NANOWII};

enum WING_MODE {WING_SINGLE_AIL=1, WING_DELTA, WING_VTAIL, WING_DUAL_AIL};
enum MIXER_EPA_MODE {MIXER_EPA_FULL=1, MIXER_EPA_NORM, MIXER_EPA_TRACK};
enum CPPM_MODE {CPPM_NONE=1, CPPM_RETA1a2F, CPPM_TAER1a2F, CPPM_AETR1a2F};
enum MOUNT_ORIENT {MOUNT_NORMAL=1, MOUNT_ROLL_90_LEFT, MOUNT_ROLL_90_RIGHT};
enum HOLD_AXES {HOLD_AXES_AER=1, HOLD_AXES_AE_R, HOLD_AXES_A_E_R};

const int8_t eeprom_cfg_ver = 3;

struct _eeprom_stats {
  int8_t device_id;
  uint32_t device_ver;
  int8_t eeprom_cfg1_err;
  int8_t eeprom_cfg2_err;
  int8_t eeprom_cfg12_reset;
};

struct _eeprom_cfg {
  uint8_t ver;
  enum WING_MODE wing_mode; 
  int8_t vr_notch[3];
  enum MIXER_EPA_MODE mixer_epa_mode;
  enum CPPM_MODE cppm_mode; 
  enum MOUNT_ORIENT mount_orient;
  enum HOLD_AXES hold_axes;
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