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
#ifdef USE_DPS3Xa
/*********************************************************************************************\
 * DPS310  - Pressure and Temperature
 * 
 * Source: https://github.com/Infineon/arduino-xensiv-dps3xx
 *
 * I2C Address: 0x76 or 0x77 
\*********************************************************************************************/

#define XSNS_128a                128
#define XI2C_93a                 93         // See I2CDEVICES.md

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

#include <Dps3xx.h>

Dps3xx Dps3xxPressureSensor = Dps3xx();

const char DpsNames[] PROGMEM = "DPS310";

typedef struct {
  uint8_t dps_address;    // I2C address
  uint8_t dps_bus;        // I2C bus
  char dps_name[7] = "DPS310";       // Sensor name - "DPSXXX"
  uint8_t dps_type;
  uint8_t dps_model;
  float dps_temperature;
  float dps_pressure;
} dps_sensors_t;

uint8_t dps_addresses[] = { DPS_ADDR1, DPS_ADDR2 };

uint8_t dps_count = 0;

dps_sensors_t *dps_sensors = nullptr;

bool Dps310Init(uint8_t bmp_idx, uint8_t bus) {
  TwoWire &wire = I2cGetWire(dps_sensors[bmp_idx].dps_bus);
  Dps3xxPressureSensor.begin(wire, dps_sensors[bmp_idx].dps_address);
  float result;
  //uint8_t resinit = Dps3xxPressureSensor.begin(dps_sensors[bmp_idx].dps_address);
  //AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: dps init done resinit:%d"), resinit);  
  uint8_t pid = Dps3xxPressureSensor.getProductId();
  AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: dps init done ProductId:%d"), pid);
  int16_t res = Dps3xxPressureSensor.getSingleResult(result);
  AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: dps getSingleResult res:%d result:%f"), res, result);
  return true;
}

void Dps310Read(uint8_t dps_idx) {
  float temperature;
  float pressure;
  uint8_t oversampling = 7;
  int16_t ret;
  //AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: dps read"));
  ret = Dps3xxPressureSensor.measureTempOnce(temperature, oversampling);
  //AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: dps: %d ret:%d temp: %f"), dps_idx, ret, temperature);
  if (ret == 0)
  {
    dps_sensors[dps_count].dps_temperature = temperature;
    //AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: dps: %d temp: %f"), dps_idx, temperature);
  }
  ret = Dps3xxPressureSensor.measurePressureOnce(pressure, oversampling);
 // AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: dps: %d ret:%d pressure: %f"), dps_idx, ret, pressure);
  if (ret == 0)
  {
    dps_sensors[dps_count].dps_pressure = pressure;
    //AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: dps: %d pressure: %f"), dps_idx, pressure);    
  }
}

/********************************************************************************************/
void Dps1Detect(void) {
  float result;
  for (uint32_t i = 0; i < DPS_MAX_SENSORS; i++) {
    if (!I2cSetDevice(dps_addresses[i &1])) { continue; }

    uint8_t initres = Dps3xxPressureSensor.begin(dps_addresses[i &1]);
    AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: dpsdetect initres:%d"), initres);
    if (initres == 0) {
      int16_t res = Dps3xxPressureSensor.getSingleResult(result);
      AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: dpsdetect getSingleResult res:%d result:%f"), res, result);

      dps_sensors[dps_count].dps_address = dps_addresses[i &1];
      I2cSetActiveFound(dps_sensors[dps_count].dps_address, dps_sensors[dps_count].dps_name);
      AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: dpsdetect found i=%d - address=%02X"), i, dps_sensors[dps_count].dps_address);
      dps_count++;
    }
  }
}
void DpsDetect(void) {
  if (!dps_sensors) {
    dps_sensors = (dps_sensors_t*)calloc(DPS_MAX_SENSORS, sizeof(dps_sensors_t));
    AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: init dps_sensors DPS_MAX_SENSORS: %d"), DPS_MAX_SENSORS);
  }
  if (!dps_sensors) { return; }

  for (uint32_t i = 0; i < DPS_MAX_SENSORS; i++) {
    uint8_t bus = i >>1;
    uint8_t dps_type;
    uint8_t status;
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
          I2cWrite8(dps_sensors[dps_count].dps_address, BMP_REGISTER_RESET, BMP_CMND_RESET, dps_sensors[dps_count].dps_bus);
          delay(100);
          status = I2cRead8(address, 0x8, bus);
          AddLog(LOG_LEVEL_DEBUG, PSTR("I2C: DPS status %02X"), status); 
          delay(100);
          success = Dps310Init(dps_count, bus);
          break;
      }
      if (success) {
        GetTextIndexed(dps_sensors[dps_count].dps_name, sizeof(dps_sensors[dps_count].dps_name), dps_sensors[dps_count].dps_model, DpsNames);
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
        Dps310Read(dps_idx);
        break;
    }
  }
}

void DpsShow(bool json) {
  for (uint32_t dps_idx = 0; dps_idx < dps_count; dps_idx++) {
    if (dps_sensors[dps_idx].dps_type) {
      float dps_temperature = ConvertTemp(dps_sensors[dps_idx].dps_temperature);
      float dps_pressure = ConvertPressure(dps_sensors[dps_idx].dps_pressure);

      char name[16];
      strlcpy(name, dps_sensors[dps_idx].dps_name, sizeof(dps_sensors[dps_idx].dps_name));
      
      if (bmp_count > 1) {

        char pressure[33];
        dtostrfd(dps_pressure, Settings->flag2.pressure_resolution, pressure);

        if (json) {

          ResponseAppend_P(PSTR(",\"%s\":{\"" D_JSON_TEMPERATURE "\":%*_f%s,\"" D_JSON_PRESSURE "\":%s}"),
            name, 
            Settings->flag2.temperature_resolution, &dps_temperature, 
            "", 
            pressure);
#ifdef USE_WEBSERVER
        } else {
          WSContentSend_Temp(name, dps_temperature);
          WSContentSend_PD(HTTP_SNS_PRESSURE, name, pressure, PressureUnit().c_str());        
#endif  // USE_WEBSERVER
        }
      }
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
        DpsShow(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        DpsShow(0);
        break;
#endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_DPS3X
#endif  // USE_I2C
