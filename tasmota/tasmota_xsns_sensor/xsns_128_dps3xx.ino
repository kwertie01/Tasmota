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
 * Source: https://github.com/Infineon/arduino-xensiv-dps3xx
 *
 * I2C Address: 0x76 or 0x77 
\*********************************************************************************************/

#define XSNS_128              128
#define XI2C_93               93 // See I2CDEVICES.md

#define DPS310_ADDRESS1       0x77
#define DPS310_ADDRESS2       0x76

#ifdef USE_I2C_BUS2
  #define DPS310_MAX_SENSORS  4     // 2 busses
#else
  #define DPS310_MAX_SENSORS  2
#endif

#include <Dps3xx.h>
#include <Wire.h>
// DPS310 Object
Dps3xx DPS310Sensor = Dps3xx();

uint8_t dps310_addresses[] = { DPS310_ADDRESS1, DPS310_ADDRESS2 };
uint8_t dps_count = 0;
const char DPSTypes[] PROGMEM = "DPS310";

struct {
  int16_t oversampling = 7;
  char types[7] = "DPS310";
  uint8_t count  = 0;
} DPS310_cfg;

typedef struct {
  uint8_t dps_address;        // I2C address
  uint8_t dps_bus;            // I2C bus 
  char name[7];
  int16_t oversampling; 
  float temperature;
  float pressure;
  uint8_t valid;
} DPS310_sensor_t;

DPS310_sensor_t *DPS310_sensor = nullptr;

/*********************************************************************************************/

bool DPS310_Read(uint32_t DPS310_idx) {
  if (DPS310_sensor[DPS310_idx].valid) { DPS310_sensor[DPS310_idx].valid--; }
  int16_t res;
  float t;
  res = DPS310Sensor.measureTempOnce(t, DPS310_sensor[DPS310_idx].dps_address, DPS310_cfg.oversampling);
  AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: dpsread T address:%02X res:%d result:%d"), DPS310_sensor[DPS310_idx].dps_address, res, t);
  if (res != 0) {
    return false;
  }
  
  float p;
  res = DPS310Sensor.measurePressureOnce(p, DPS310_sensor[DPS310_idx].dps_address, DPS310_cfg.oversampling);
  AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: dpsread P address:%02X res:%d result:%d"), DPS310_sensor[DPS310_idx].dps_address, res, p);   
  if (res != 0) {
    return false;
  }

  DPS310_sensor[DPS310_idx].temperature = (float)ConvertTemp(t);
  DPS310_sensor[DPS310_idx].pressure = (float)ConvertPressure(p / 100);  // Conversion to hPa
  AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: dpsread address:%02X T:%f P:%f"), DPS310_sensor[DPS310_idx].dps_address, DPS310_sensor[DPS310_idx].temperature, DPS310_sensor[DPS310_idx].pressure);   

  DPS310_sensor[DPS310_idx].valid = SENSOR_MAX_MISS;
  return true;
}

/********************************************************************************************/

void DPS310_Detect(void) {
  if (!DPS310_sensor) {
    DPS310_sensor = (DPS310_sensor_t*)calloc(DPS310_MAX_SENSORS, sizeof(DPS310_sensor_t));
  }
  if (!DPS310_sensor) { return; }

  for (uint32_t i = 0; i < DPS310_MAX_SENSORS; i++) {
    uint8_t bus = i >>1;
    AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: DPS310 detect start i:%d address:%02X bus:%d"), i, dps310_addresses[i &1], bus);   

    if (!I2cSetDevice(dps310_addresses[i &1], bus)) { continue; }
    AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: DPS310 detected i:%d address:%02X bus:%d"), i, dps310_addresses[i &1], bus);   

    TwoWire &wire = I2cGetWire(bus);
    uint8_t initres = DPS310Sensor.begin(wire, dps310_addresses[i &1]);
    AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: DPS310 detected begin i:%d address:%02X bus:%d initres:%d"), i, dps310_addresses[i &1], bus, initres);   

    if (initres == 0) {
      DPS310_sensor[dps_count].dps_address = dps310_addresses[i &1];
      DPS310_sensor[dps_count].dps_bus = bus;
      DPS310_sensor[dps_count].oversampling = 7;
      DPS310_sensor[dps_count].valid = 0;  

      strcpy_P(DPS310_sensor[dps_count].name, PSTR("DPS310"));
      AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: DPS310 detected begin i:%d address:%02X bus:%d name:%s"), i, DPS310_sensor[dps_count].dps_address, DPS310_sensor[dps_count].dps_bus, DPS310_sensor[dps_count].name);   
      I2cSetActiveFound(DPS310_sensor[dps_count].dps_address, DPS310_sensor[dps_count].name, DPS310_sensor[dps_count].dps_bus);
      dps_count++;
      AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: DPS310 detected begin dps_count:%d"), dps_count);   
    }

  }
}

void DPS310_EverySecond(void) {
  for (uint32_t i = 0; i < dps_count; i++) {
    if (TasmotaGlobal.uptime &1) {
      if (!DPS310_Read(i)) {
        AddLogMissed(DPS310_sensor[dps_count].name, DPS310_sensor[dps_count].valid);
      }
    }
  }
}

void DPS310_Show(bool json) {
  for (uint32_t i = 0; i < dps_count; i++) {
    if (DPS310_sensor[i].valid) {
      char sensor_name[12];
      strlcpy(sensor_name, DPS310_sensor[dps_count].name, sizeof(DPS310_sensor[dps_count].name));
      if (dps_count > 1) {
        snprintf_P(sensor_name, sizeof(sensor_name), PSTR("%s%c%02X"), sensor_name, IndexSeparator(), DPS310_sensor[dps_count].dps_address); // DPS310-76, DPS310-77
      }

      char str_pressure[33];
      dtostrfd(DPS310_sensor[dps_count].pressure, Settings->flag2.pressure_resolution, str_pressure);

      if (json) {
        ResponseAppend_P(PSTR(",\"%s\":{\"" D_JSON_TEMPERATURE "\":%*_f,\"" D_JSON_PRESSURE "\":%s"),
          sensor_name, Settings->flag2.temperature_resolution, &DPS310_sensor[dps_count].temperature,  str_pressure);
        ResponseJsonEnd();
#ifdef USE_DOMOTICZ
        // Domoticz and knx only support one temp sensor
        if ((0 == TasmotaGlobal.tele_period) && (0 == i)) {
          DomoticzFloatSensor(DZ_TEMP, DPS310_sensor[dps_count].temperature);
        }
#endif // USE_DOMOTICZ
#ifdef USE_WEBSERVER
      } else {
        WSContentSend_Temp(sensor_name, DPS310_sensor[dps_count].temperature);
        WSContentSend_PD(HTTP_SNS_PRESSURE, sensor_name, str_pressure, PressureUnit().c_str());
#endif // USE_WEBSERVER
      }
    }
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns128(uint32_t function)
{
  if (!I2cEnabled(XI2C_93)) { return false; }

  bool result = false;

  if (FUNC_INIT == function) {
    DPS310_Detect();
  }
  else if (dps_count) {
    switch (function) {
      case FUNC_EVERY_SECOND:
      DPS310_EverySecond();
      break;
    case FUNC_JSON_APPEND:
      DPS310_Show(1);
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      DPS310_Show(0);
      break;
#endif // USE_WEBSERVER
    }
  }
  return result;
}

#endif // USE_DPS3X
#endif // USE_I2C
