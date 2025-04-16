/*
  xsns_128_dps3xx.ino -  Infineon's XENSIV™ Digital Pressure Sensors (DPS3xx)
                         Pressure and temperature sensor support for Tasmota.

  Copyright (C) 2025  Kwertie01

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifdef USE_I2C
#ifdef USE_DPS3X
/*********************************************************************************************\
 * DPS310  - Pressure and Temperature
 * 
 * Source: Heiko Krupp and Adafruit Industries
 *
 * I2C Address: 0x76 or 0x77 
\*********************************************************************************************/

#define XSNS_128                128
#define XI2C_93                 93         // See I2CDEVICES.md

#define DPS_ADDR1               0x76
#define DPS_ADDR2               0x77

#define DPS310_CHIPID           0x10
#define DPS_REGISTER_CHIPID     0x0D

#define DPS_REGISTER_RESET      0x0C  // RESET Register to reset to power on defaults
#define DPS_CMND_RESET          0x09  // Parameter for softreset

#ifdef USE_I2C_BUS2
  #define DPS_MAX_SENSORS       4     // 2 busses
#else
  #define DPS_MAX_SENSORS       2
#endif

#include <Dps3xx.h"
Dps3xx Dps3xxPressureSensor = Dps3xx();

const char kDpsTypes[] PROGMEM = "DPS310";

typedef struct {
  uint8_t dps_address;    // I2C address
  uint8_t dps_bus;        // I2C bus
  char dps_name[7];       // Sensor name - "DPSXXX"
  uint8_t dps_type;
  uint8_t dps_model;
  float dps_temperature;
  float dps_pressure;
} dps_sensors_t;

uint8_t dps_addresses[] = { DPS_ADDR1, DPS_ADDR2 };

uint8_t dps_count = 0;

dps_sensors_t *dps_sensors = nullptr;




/********************************************************************************************/

void DpsDetect(void) {
  if (!dps_sensors) {
    dps_sensors = (dps_sensors_t*)calloc(DPS_MAX_SENSORS, sizeof(dps_sensors_t));
    AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: init dps_sensors DPS_MAX_SENSORS: %d"), DPS_MAX_SENSORS);
  }
  if (!dps_sensors) { return; }

  for (uint32_t i = 0; i < DPS_MAX_SENSORS; i++) {
    uint8_t bus = i >>1;
    uint8_t dps_type;
    uint8_t address;
    uint8_t chipidregister;
    AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: checking i=%d - bus=%d - dpsaddress=%02X"), i, bus, dps_addresses[i &1]); 

    if (!I2cSetDevice(dps_addresses[i &1], bus)) { continue; }
    else
    {
      address = dps_addresses[i &1];
      chipidregister = DPS_REGISTER_CHIPID;          
    }
    AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: found i=%d - bus=%d - address=%02X"), i, bus, address);
    dps_type = I2cRead8(address, chipidregister, bus);
    AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: DPS Type %02X"), dps_type); 
    if (dps_type) {
      dps_sensors[dps_count].dps_address = address;
      dps_sensors[dps_count].dps_bus = bus;
      dps_sensors[dps_count].dps_type = dps_type;
      dps_sensors[dps_count].dps_model = 0;

      bool success = false;
      switch (dps_type) {
        case DPS310_CHIPID:
          //success = Dps180Calibration(dps_count);
          success = true;
          break;
      }
      if (success) {
        GetTextIndexed(dps_sensors[dps_count].dps_name, sizeof(dps_sensors[dps_count].dps_name), dps_sensors[dps_count].dps_model, kDpsTypes);
        I2cSetActiveFound(dps_sensors[dps_count].dps_address, dps_sensors[dps_count].dps_name, dps_sensors[dps_count].dps_bus);
        dps_count++;
      }
    }
    AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: endif dps type i=%d"), i);
  }
  AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: end for"));
}

void DpsRead(void) {
  for (uint32_t dps_idx = 0; dps_idx < dps_count; dps_idx++) {
    switch (dps_sensors[dps_idx].dps_type) {
      case DPS310_CHIPID:
        //Dps310Read(dps_idx);
        break;
    }
  }
}



/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns128(uint32_t function) {
  if (!I2cEnabled(XI2C_93)) { return false; }

  bool result = false;

  if (FUNC_INIT == function) {
    DpsDetect();
  }
  else if (dps_count) {
    switch (function) {
      case FUNC_EVERY_SECOND:
        DpsRead();
        break;
      case FUNC_JSON_APPEND:
        //DpsShow(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        //DpsShow(0);
        break;
#endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_DPS3X
#endif  // USE_I2C
