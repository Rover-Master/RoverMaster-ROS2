#pragma once

#include <cstdint>
#include <cstdio>

#include "MultiWii.h"

namespace MSP {
using namespace MultiWii;

// Reboot message contains no payload
// Call it with query() instead of send()
struct REBOOT_s {};

typedef Packet<68, PacketType::READABLE, REBOOT_s> REBOOT;

struct IDENT_s {
  uint8_t VERSION;
  uint8_t MULTITYPE;
  uint8_t MSP_VERSION;
  uint32_t capability;
};

typedef Packet<100, PacketType::READABLE, IDENT_s> IDENT;

struct STATUS_s {
  uint16_t cycleTime;
  uint16_t i2c_errors_count;
  uint16_t sensor;
  uint32_t flag;
  uint8_t global_conf_currentSet;
};

typedef Packet<101, PacketType::READABLE, STATUS_s> STATUS;

struct RAW_IMU_s {
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t gyrX;
  int16_t gyrY;
  int16_t gyrZ;
  int16_t magX;
  int16_t magY;
  int16_t magZ;
};

typedef Packet<102, PacketType::READABLE, RAW_IMU_s> RAW_IMU;

struct SERVO_s {
  uint16_t servo[8][2];
};

typedef Packet<103, PacketType::READABLE, SERVO_s> SERVO;

struct MOTOR_s {
  uint16_t motor[8];
};

typedef Packet<104, PacketType::READABLE, MOTOR_s> MOTOR;

struct RC_s {
  uint16_t ROLL;
  uint16_t PITCH;
  uint16_t YAW;
  uint16_t THROTTLE;
  uint16_t AUX1;
  uint16_t AUX2;
  uint16_t AUX3;
  uint16_t AUX4;
  uint16_t AUX5;
  uint16_t AUX6;
  uint16_t AUX7;
  uint16_t AUX8;
  uint16_t AUX9;
  uint16_t AUX10;
  uint16_t AUX11;
  uint16_t AUX12;
};

typedef Packet<105, PacketType::READABLE, RC_s> RC;

struct RAW_GPS_s {
  uint8_t GPS_FIX;
  uint8_t GPS_numSat;
  uint32_t GPS_coord_LAT;
  uint32_t GPS_coord_LON;
  uint16_t GPS_altitude;
  uint16_t GPS_speed;
  uint16_t GPS_ground_course;
};

typedef Packet<106, PacketType::READABLE, RAW_GPS_s> RAW_GPS;

struct COMP_GPS_s {
  uint16_t GPS_distanceToHome;
  uint16_t GPS_directionToHome;
  uint8_t GPS_update;
};

typedef Packet<107, PacketType::READABLE, COMP_GPS_s> COMP_GPS;

struct ATTITUDE_s {
  int16_t angx;
  int16_t angy;
  int16_t heading;
};

typedef Packet<108, PacketType::READABLE, ATTITUDE_s> ATTITUDE;

struct ALTITUDE_s {
  int32_t EstAlt;
  int16_t vario;
};

typedef Packet<109, PacketType::READABLE, ALTITUDE_s> ALTITUDE;

struct ANALOG_s {
  uint8_t vbat;
  uint16_t intPowerMeterSum;
  uint16_t rssi;
  uint16_t amperage;
};

typedef Packet<110, PacketType::READABLE, ANALOG_s> ANALOG;

struct RC_TUNING_s {
  uint8_t byteRC_RATE;
  uint8_t byteRC_EXPO;
  uint8_t byteRollPitchRate;
  uint8_t byteYawRate;
  uint8_t byteDynThrPID;
  uint8_t byteThrottle_MID;
  uint8_t byteThrottle_EXPO;
};

typedef Packet<111, PacketType::READABLE, RC_TUNING_s> RC_TUNING;

struct PID_s {
  uint8_t A_ROLL;
  uint8_t A_PITCH;
  uint8_t A_YAW;
  uint8_t A_ALT;
  uint8_t A_POS;
  uint8_t A_POSR;
  uint8_t A_NAVR;
  uint8_t A_LEVEL;
  uint8_t A_MAG;
  uint8_t A_VEL;
  uint8_t B_ROLL;
  uint8_t B_PITCH;
  uint8_t B_YAW;
  uint8_t B_ALT;
  uint8_t B_POS;
  uint8_t B_POSR;
  uint8_t B_NAVR;
  uint8_t B_LEVEL;
  uint8_t B_MAG;
  uint8_t B_VEL;
  uint8_t C_ROLL;
  uint8_t C_PITCH;
  uint8_t C_YAW;
  uint8_t C_ALT;
  uint8_t C_POS;
  uint8_t C_POSR;
  uint8_t C_NAVR;
  uint8_t C_LEVEL;
  uint8_t C_MAG;
  uint8_t C_VEL;
};

typedef Packet<112, PacketType::READABLE, PID_s> PID;

struct BOX_s {};

typedef Packet<113, PacketType::READABLE, BOX_s> BOX;

struct MISC_s {
  uint16_t intPowerTrigger1;
  uint16_t conf_minthrottle;
  uint16_t MAXTHROTTLE;
  uint16_t MINCOMMAND;
  uint16_t conf_failsafe_throttle;
  uint16_t plog_arm;
  uint32_t plog_lifetime;
  uint16_t conf_mag_declination;
  uint8_t conf_vbatscale;
  uint8_t conf_vbatlevel_warn1;
  uint8_t conf_vbatlevel_warn2;
  uint8_t conf_vbatlevel_crit;
};

typedef Packet<114, PacketType::READABLE, MISC_s> MISC;

struct MOTOR_PINS_s {
  uint8_t pin[8];
};

typedef Packet<115, PacketType::READABLE, MOTOR_PINS_s> MOTOR_PINS;

struct SET_RAW_RC_s {
  uint16_t ROLL;
  uint16_t PITCH;
  uint16_t YAW;
  uint16_t THROTTLE;
  uint16_t AUX1;
  uint16_t AUX2;
  uint16_t AUX3;
  uint16_t AUX4;
  uint16_t AUX5;
  uint16_t AUX6;
  uint16_t AUX7;
  uint16_t AUX8;
  uint16_t AUX9;
  uint16_t AUX10;
  uint16_t AUX11;
  uint16_t AUX12;
};

typedef Packet<200, PacketType::WRITABLE, SET_RAW_RC_s> SET_RAW_RC;

struct SET_RAW_GPS_s {
  uint8_t GPS_FIX;
  uint8_t GPS_numSat;
  uint32_t GPS_coord_LAT;
  uint32_t GPS_coord_LON;
  uint16_t GPS_altitude;
  uint16_t GPS_speed;
};

typedef Packet<201, PacketType::WRITABLE, SET_RAW_GPS_s> SET_RAW_GPS;

struct SET_PID_s {
  uint8_t A_ROLL;
  uint8_t A_PITCH;
  uint8_t A_YAW;
  uint8_t A_ALT;
  uint8_t A_POS;
  uint8_t A_POSR;
  uint8_t A_NAVR;
  uint8_t A_LEVEL;
  uint8_t A_MAG;
  uint8_t A_VEL;
  uint8_t B_ROLL;
  uint8_t B_PITCH;
  uint8_t B_YAW;
  uint8_t B_ALT;
  uint8_t B_POS;
  uint8_t B_POSR;
  uint8_t B_NAVR;
  uint8_t B_LEVEL;
  uint8_t B_MAG;
  uint8_t B_VEL;
  uint8_t C_ROLL;
  uint8_t C_PITCH;
  uint8_t C_YAW;
  uint8_t C_ALT;
  uint8_t C_POS;
  uint8_t C_POSR;
  uint8_t C_NAVR;
  uint8_t C_LEVEL;
  uint8_t C_MAG;
  uint8_t C_VEL;
};

typedef Packet<202, PacketType::WRITABLE, SET_PID_s> SET_PID;

struct SET_BOX_s {};

typedef Packet<203, PacketType::WRITABLE, SET_BOX_s> SET_BOX;

struct SET_RC_TUNING_s {
  uint8_t byteRC_RATE;
  uint8_t byteRC_EXPO;
  uint8_t byteRollPitchRate;
  uint8_t byteYawRate;
  uint8_t byteDynThrPID;
  uint8_t byteThrottle_MID;
  uint8_t byteThrottle_EXPO;
};

typedef Packet<204, PacketType::WRITABLE, SET_RC_TUNING_s> SET_RC_TUNING;

struct ACC_CALIBRATION_s {};

typedef Packet<205, PacketType::WRITABLE, ACC_CALIBRATION_s> ACC_CALIBRATION;

struct MAG_CALIBRATION_s {};

typedef Packet<206, PacketType::WRITABLE, MAG_CALIBRATION_s> MAG_CALIBRATION;

struct SET_MISC_s {
  uint16_t intPowerTrigger1;
  uint16_t conf_minthrottle;
  uint16_t MAXTHROTTLE;
  uint16_t MINCOMMAND;
  uint16_t conf_failsafe_throttle;
  uint16_t plog_arm;
  uint32_t plog_lifetime;
  uint16_t conf_mag_declination;
  uint8_t conf_vbatscale;
  uint8_t conf_vbatlevel_warn1;
  uint8_t conf_vbatlevel_warn2;
  uint8_t conf_vbatlevel_crit;
};

typedef Packet<207, PacketType::WRITABLE, SET_MISC_s> SET_MISC;

struct RESET_CONF_s {};

typedef Packet<208, PacketType::WRITABLE, RESET_CONF_s> RESET_CONF;

struct SET_HEAD_s {
  int16_t magHold;
};

typedef Packet<211, PacketType::WRITABLE, SET_HEAD_s> SET_HEAD;

struct SET_MOTOR_s {
  uint16_t motor[8];
};

typedef Packet<214, PacketType::WRITABLE, SET_MOTOR_s> SET_MOTOR;

struct BIND_s {};

typedef Packet<240, PacketType::WRITABLE, BIND_s> BIND;

struct EEPROM_WRITE_s {};

typedef Packet<250, PacketType::WRITABLE, EEPROM_WRITE_s> EEPROM_WRITE;

} // namespace MSP
