/* FlightStab **************************************************************************************************/

/* NOTE:
./Aquastar/FlightStab.h and ./FlightStab/FlightStab.h must be identical at all times for the 
aquastar programming box to be in sync with the flightstab firmware.
*/

#if !defined(FLIGHTSTAB_H)
#define FLIGHTSTAB_H
enum WING_MODE {WING_SINGLE_AIL, WING_DELTA, WING_VTAIL, WING_DUAL_AIL};
enum MIXER_EPA_MODE {MIXER_EPA_FULL, MIXER_EPA_NORM, MIXER_EPA_TRACK};
enum CPPM_MODE {CPPM_NONE=0, CPPM_OPEN9X=1, CPPM_UNDEF};
enum MOUNT_ORIENT {MOUNT_NORMAL, MOUNT_ROLL_90_LEFT, MOUNT_ROLL_90_RIGHT};

const int8_t eeprom_cfg_ver = 2;
const uint16_t eeprom_cfg1_addr = 0;
const uint16_t eeprom_cfg2_addr = 0;

struct _eeprom_stats {
  uint32_t device_id;
  uint32_t device_version;
  int8_t eeprom_cfg1_err;
  int8_t eeprom_cfg2_err;
};

struct _eeprom_cfg {
  uint8_t ver;
  enum WING_MODE wing_mode; 
  int8_t vr_notch[3];
  enum MIXER_EPA_MODE mixer_epa_mode;
  enum CPPM_MODE cppm_mode; 
  enum MOUNT_ORIENT mount_orient;
  uint8_t chksum;
};

enum OW_COMMAND {OW_SYNC, OW_GET_CFG, OW_SET_CFG};

struct _ow_msg {
  uint8_t cmd;
  union {
    int8_t device_id; // OW_SYNC response
    struct _eeprom_cfg eeprom_cfg;
  } u;
};

#endif // FLIGHTSTAB_H