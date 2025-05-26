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

#define XSNS_128                128
#define XI2C_93                 93     // See I2CDEVICES.md

#ifdef USE_I2C_BUS2
  #define DPS310_MAX_SENSORS    4      // 2 busses
#else
  #define DPS310_MAX_SENSORS    2
#endif

#define DPS310_ADDRESS1         0x77
#define DPS310_ADDRESS2         0x76
#define DPS310_CHIPID           0x10   // =DPS310

#define DPS_REGISTER_PRS_CFG    0x06   // Pressure Configuration
#define DPS_REGISTER_TMP_CFG    0x07   // Temperature Configuration
#define DPS_REGISTER_MEAS_CFG   0x08   // Sensor Operating Mode and Status
#define DPS_REGISTER_CFG_REG    0x09   // Interrupt and FIFO configuration
#define DPS_REGISTER_RESET      0x0C   // Soft Reset and FIFO flush
#define DPS_REGISTER_CHIPID     0x0D   // Product and Revision ID
#define DPS_REGISTER_COEF       0x10   // Calibration Coefficients. Register block start at 0x10 end at 0x21
#define DPS_REGISTER_COEF_SRCE  0x28   // Coefficient Source

#define DPS_CMND_RESET          0x09   // Parameter for softreset

#define DPS__MEASUREMENT_RATE_1 0
#define DPS__MEASUREMENT_RATE_2 1
#define DPS__MEASUREMENT_RATE_4 2
#define DPS__MEASUREMENT_RATE_8 3
#define DPS__MEASUREMENT_RATE_16 4
#define DPS__MEASUREMENT_RATE_32 5
#define DPS__MEASUREMENT_RATE_64 6
#define DPS__MEASUREMENT_RATE_128 7

#define DPS__OVERSAMPLING_RATE_1 DPS__MEASUREMENT_RATE_1
#define DPS__OVERSAMPLING_RATE_2 DPS__MEASUREMENT_RATE_2
#define DPS__OVERSAMPLING_RATE_4 DPS__MEASUREMENT_RATE_4
#define DPS__OVERSAMPLING_RATE_8 DPS__MEASUREMENT_RATE_8
#define DPS__OVERSAMPLING_RATE_16 DPS__MEASUREMENT_RATE_16
#define DPS__OVERSAMPLING_RATE_32 DPS__MEASUREMENT_RATE_32
#define DPS__OVERSAMPLING_RATE_64 DPS__MEASUREMENT_RATE_64
#define DPS__OVERSAMPLING_RATE_128 DPS__MEASUREMENT_RATE_128

enum Mode
{
    IDLE = 0x00,
    CMD_PRS = 0x01,
    CMD_TEMP = 0x02,
    CMD_BOTH = 0x03, // only for DPS422
    CONT_PRS = 0x05,
    CONT_TMP = 0x06,
    CONT_BOTH = 0x07
};

uint8_t dps310_addresses[] = { DPS310_ADDRESS1, DPS310_ADDRESS2 };
uint8_t dps_count = 0;

typedef struct {
  uint8_t dps_address;        // I2C address
  uint8_t dps_bus;            // I2C bus
  uint8_t dps_type;
  char dps_name[7];
  int16_t dps_oversampling_temperature;
  int16_t dps_oversampling_pressure;
  int16_t dps_coef_srce;  
  float dps_temperature;
  float dps_pressure;
  uint8_t dps_valid;
  float m_lastTempScal;
  int32_t m_c0Half;
  int32_t m_c1;
  int32_t m_c00;
  int32_t m_c10;
  int32_t m_c01;
  int32_t m_c11;
  int32_t m_c20;
  int32_t m_c21;
  int32_t m_c30;  
} DPS310_sensor_t;

DPS310_sensor_t *DPS310_sensor = nullptr;

const int32_t scaling_facts[8] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};


void standby(uint8_t dps_idx)
{
    // set device to idling mode
    I2cWrite8(DPS310_sensor[dps_idx].dps_address, DPS_REGISTER_MEAS_CFG, IDLE, DPS310_sensor[dps_idx].dps_bus); //(MEAS_CFG) - MEAS_CTRL 0x0 - Standby.
    delay(50);
    uint8_t status = I2cRead8(DPS310_sensor[dps_idx].dps_address, DPS_REGISTER_MEAS_CFG, DPS310_sensor[dps_idx].dps_bus);
}
void reset(uint8_t dps_idx)
{
    // Soft reset device
    I2cWrite8(DPS310_sensor[dps_idx].dps_address, DPS_REGISTER_RESET, DPS_CMND_RESET, DPS310_sensor[dps_idx].dps_bus);    
    delay(50);
    uint8_t status = I2cRead8(DPS310_sensor[dps_idx].dps_address, DPS_REGISTER_MEAS_CFG, DPS310_sensor[dps_idx].dps_bus);
}

void getTwosComplement(int32_t *raw, uint8_t length)
{
    if (*raw & ((uint32_t)1 << (length - 1)))
    {
        *raw -= (uint32_t)1 << length;
    }
}

int16_t configTemp(uint8_t tempMr, uint8_t tempOsr, uint8_t dps_idx)
{
    int16_t tmpcfg;
    uint8_t olddata;
    uint8_t newdata;
    uint8_t mask = 0x08;
    uint8_t shift = 3;

    tmpcfg = (DPS310_sensor[dps_idx].dps_coef_srce << 7) + (tempMr << 4) + tempOsr;
    I2cWrite8(DPS310_sensor[dps_idx].dps_address, DPS_REGISTER_TMP_CFG, tmpcfg, DPS310_sensor[dps_idx].dps_bus);    //set TMP_CFG register with temperature oversampling
  
    olddata = I2cRead8(DPS310_sensor[dps_idx].dps_address, DPS_REGISTER_CFG_REG, DPS310_sensor[dps_idx].dps_bus);
    if (tempOsr > 3) {newdata = 1U;}
    else {newdata = 0U;}
    I2cWrite8(DPS310_sensor[dps_idx].dps_address, DPS_REGISTER_CFG_REG, ((uint8_t)olddata & ~mask) | ((newdata << shift) & mask), DPS310_sensor[dps_idx].dps_bus); //(CFG_REG) - T_SHIFT enabled (bit 3).

    return 1;
}

int16_t configPressure(uint8_t prsMr, uint8_t prsOsr, uint8_t dps_idx)
{
    int16_t prscfg;
    uint8_t olddata;
    uint8_t newdata;
    uint8_t mask = 0x04;
    uint8_t shift = 2;

    prscfg = (prsMr << 4) + prsOsr;
    I2cWrite8(DPS310_sensor[dps_idx].dps_address, DPS_REGISTER_PRS_CFG, prscfg, DPS310_sensor[dps_idx].dps_bus);
  
    olddata = I2cRead8(DPS310_sensor[dps_idx].dps_address, DPS_REGISTER_CFG_REG, DPS310_sensor[dps_idx].dps_bus);
    if (prsOsr > 3) {newdata = 1U;}
    else {newdata = 0U;}
    I2cWrite8(DPS310_sensor[dps_idx].dps_address, DPS_REGISTER_CFG_REG, ((uint8_t)olddata & ~mask) | ((newdata << shift) & mask), DPS310_sensor[dps_idx].dps_bus); //(CFG_REG) - P_SHIFT enabled (bit 2).

    return 1;
}

float calcTemp(int32_t raw, uint8_t dps_idx)
{
    float temp = (float)raw;
    // scale temperature according to scaling table and oversampling
    temp /= scaling_facts[DPS310_sensor[dps_idx].dps_oversampling_temperature];
    // update last measured temperature
    // it will be used for pressure compensation
    DPS310_sensor[dps_idx].m_lastTempScal = temp;

    // Calculate compensated temperature
    temp = DPS310_sensor[dps_idx].m_c0Half + DPS310_sensor[dps_idx].m_c1 * temp;
    return temp;
}

float calcPressure(int32_t raw, uint8_t dps_idx)
{
    float prs = (float)raw;
    // scale pressure according to scaling table and oversampling
    prs /= scaling_facts[DPS310_sensor[dps_idx].dps_oversampling_pressure];
    // Calculate compensated pressure
    prs = DPS310_sensor[dps_idx].m_c00 + prs * (DPS310_sensor[dps_idx].m_c10 + prs * (DPS310_sensor[dps_idx].m_c20 + prs * DPS310_sensor[dps_idx].m_c30)) + DPS310_sensor[dps_idx].m_lastTempScal * (DPS310_sensor[dps_idx].m_c01 + prs * (DPS310_sensor[dps_idx].m_c11 + prs * DPS310_sensor[dps_idx].m_c21));
    return prs;
}

int16_t readcoeffs(uint8_t dps_idx)
{
    int32_t m_c0Half;
    int32_t m_c1;
    int32_t m_c00;
    int32_t m_c10;
    int32_t m_c01;
    int32_t m_c11;
    int32_t m_c20;
    int32_t m_c21;
    int32_t m_c30;  

    uint8_t buffer[18];
    
    // read COEF registers to buffer
    I2cReadBuffer(DPS310_sensor[dps_idx].dps_address, DPS_REGISTER_COEF, buffer, sizeof(buffer), DPS310_sensor[dps_idx].dps_bus);
    //int16_t ret = readBlock(coeffBlock, buffer);
    //if (!ret)
    //  return DPS__FAIL_INIT_FAILED;

    // compose coefficients from buffer content
    m_c0Half = ((uint32_t)buffer[0] << 4) | (((uint32_t)buffer[1] >> 4) & 0x0F);
    getTwosComplement(&m_c0Half, 12);
    // c0 is only used as c0*0.5, so c0_half is calculated immediately
    DPS310_sensor[dps_idx].m_c0Half = m_c0Half / 2U;

    // now do the same thing for all other coefficients
    m_c1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
    getTwosComplement(&m_c1, 12);
    DPS310_sensor[dps_idx].m_c1 = m_c1;
    m_c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (((uint32_t)buffer[5] >> 4) & 0x0F);
    getTwosComplement(&m_c00, 20);
    DPS310_sensor[dps_idx].m_c00 = m_c00;
    m_c10 = (((uint32_t)buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) | (uint32_t)buffer[7];
    getTwosComplement(&m_c10, 20);
    DPS310_sensor[dps_idx].m_c10 = m_c10;

    m_c01 = ((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9];
    getTwosComplement(&m_c01, 16);
    DPS310_sensor[dps_idx].m_c01 = m_c01;    

    m_c11 = ((uint32_t)buffer[10] << 8) | (uint32_t)buffer[11];
    getTwosComplement(&m_c11, 16);
    DPS310_sensor[dps_idx].m_c11 = m_c11;
    m_c20 = ((uint32_t)buffer[12] << 8) | (uint32_t)buffer[13];
    getTwosComplement(&m_c20, 16);
    DPS310_sensor[dps_idx].m_c20 = m_c20;
    m_c21 = ((uint32_t)buffer[14] << 8) | (uint32_t)buffer[15];
    getTwosComplement(&m_c21, 16);
    DPS310_sensor[dps_idx].m_c21 = m_c21;
    m_c30 = ((uint32_t)buffer[16] << 8) | (uint32_t)buffer[17];
    getTwosComplement(&m_c30, 16);
    DPS310_sensor[dps_idx].m_c30 = m_c30;

    return 1;
}

/*********************************************************************************************/
/*
Read Steps:
  write 0x02 to reg 0x08  (Sensor Operating Mode and Status (MEAS_CFG))
  read reg 0x03 - 0x05    (read raw temperature)
  calculate temperature
  write 0x01 to reg 0x08
  read reg 0x00 - 0x02    (read raw pressure)
  calculate pressure
  (loop)
*/
bool DPS310_Read(uint8_t dps_idx) {
  if (DPS310_sensor[dps_idx].dps_valid) { DPS310_sensor[dps_idx].dps_valid--; }

  uint8_t status;
  int32_t raw;
  int32_t raw1;
  float result;
  float resultp;

  I2cWrite8(DPS310_sensor[dps_idx].dps_address, DPS_REGISTER_MEAS_CFG, CMD_TEMP, DPS310_sensor[dps_idx].dps_bus); //(MEAS_CFG) - MEAS_CTRL 0x2 - Temperature measurement.
  raw = I2cRead24(DPS310_sensor[dps_idx].dps_address, 0x03,  DPS310_sensor[dps_idx].dps_bus); //Read 24 bits Temperature
  getTwosComplement(&raw, 24);
  result = calcTemp(raw, dps_idx);

  standby(dps_idx);

  I2cWrite8(DPS310_sensor[dps_idx].dps_address, DPS_REGISTER_MEAS_CFG, CMD_PRS, DPS310_sensor[dps_idx].dps_bus); //(MEAS_CFG) - MEAS_CTRL 0x1 - Pressure measurement.
  raw1 = I2cRead24(DPS310_sensor[dps_idx].dps_address, 0x00, DPS310_sensor[dps_idx].dps_bus); //Read 24 bits Temperature
  getTwosComplement(&raw1, 24);
  resultp = calcPressure(raw1, dps_idx);

  DPS310_sensor[dps_idx].dps_temperature = (float)result;
  DPS310_sensor[dps_idx].dps_pressure = (float)ConvertPressure(resultp / 100);  // Conversion to hPa
  
  DPS310_sensor[dps_idx].dps_valid = SENSOR_MAX_MISS;
  return true;
}

/********************************************************************************************/
/*
Init steps:
  read  reg 0x28        (TMP_COEF_SRCE)
  read  reg 0x10-0x21   (COEF)
  write reg 0x06        (Pressure Configuration (PRS_CFG))
  write reg 0x07        (Temperature Configuration(TMP_CFG))
  write reg 0x09        (Interrupt and FIFO configuration (CFG_REG))
*/
bool DPS310_Init(uint8_t dps_idx) {
  uint16_t ret;
  // Read Coefficient Source
  uint8_t TMP_COEF_SRCE = I2cRead8(DPS310_sensor[dps_idx].dps_address, DPS_REGISTER_COEF_SRCE, DPS310_sensor[dps_idx].dps_bus);
  TMP_COEF_SRCE &= 0x80;
  DPS310_sensor[dps_idx].dps_coef_srce = (TMP_COEF_SRCE >> 7);

  // Read Calibration Coefficients
  uint8_t res = readcoeffs(dps_idx);

  standby(dps_idx);
  // configure Pressure settings
  ret = configPressure(DPS__MEASUREMENT_RATE_4, DPS310_sensor[dps_idx].dps_oversampling_pressure, dps_idx);
  
  // configure Temperature settings
  ret = configTemp(DPS__MEASUREMENT_RATE_4, DPS310_sensor[dps_idx].dps_oversampling_temperature, dps_idx);
 
  standby(dps_idx);
  DPS310_sensor[dps_idx].dps_valid = SENSOR_MAX_MISS;

  return true;
}

void DPS310_Detect(void) {
  if (!DPS310_sensor) {
    DPS310_sensor = (DPS310_sensor_t*)calloc(DPS310_MAX_SENSORS, sizeof(DPS310_sensor_t));
  }
  if (!DPS310_sensor) { return; }

  for (uint32_t i = 0; i < DPS310_MAX_SENSORS; i++) {
    uint8_t bus = i >>1;
    uint8_t dps_type;
    uint8_t address;
    uint8_t chipidregister;
   
    if (!I2cSetDevice(dps310_addresses[i &1], bus)) { continue; }
    else
    {
      address = dps310_addresses[i &1];
      chipidregister = DPS_REGISTER_CHIPID;          
    }
    dps_type = I2cRead8(address, chipidregister, bus);
    if (dps_type) {
      DPS310_sensor[dps_count].dps_address = address;
      DPS310_sensor[dps_count].dps_bus = bus;
      DPS310_sensor[dps_count].dps_type = dps_type;
      DPS310_sensor[dps_count].dps_oversampling_temperature = DPS__OVERSAMPLING_RATE_8;   //default oversampling_temperature  8x
      DPS310_sensor[dps_count].dps_oversampling_pressure = DPS__OVERSAMPLING_RATE_128;    //default oversampling_pressure     128x
      DPS310_sensor[dps_count].dps_valid = 0;  
      strcpy_P(DPS310_sensor[dps_count].dps_name, PSTR("DPS310"));
      bool success = false;

      switch (dps_type) {
        case DPS310_CHIPID:
          success = DPS310_Init(dps_count);
          break;
      }
      
      if (success) {
        I2cSetActiveFound(DPS310_sensor[dps_count].dps_address, DPS310_sensor[dps_count].dps_name, DPS310_sensor[dps_count].dps_bus);
        dps_count++;
      }
    }
  }
}

void DPS310_EverySecond(void) {
  for (uint32_t i = 0; i < dps_count; i++) {
    if (TasmotaGlobal.uptime &1) {
      if (!DPS310_Read(i)) {
        AddLogMissed(DPS310_sensor[i].dps_name, DPS310_sensor[i].dps_valid);
      }
    }
  }
}

void DPS310_Show(bool json) {
  for (uint32_t i = 0; i < dps_count; i++) {
    if (DPS310_sensor[i].dps_valid) {
      char sensor_name[12];
      strlcpy(sensor_name, DPS310_sensor[i].dps_name, sizeof(DPS310_sensor[i].dps_name));
      if (dps_count > 1) {
        snprintf_P(sensor_name, sizeof(sensor_name), PSTR("%s%c%02X"), sensor_name, IndexSeparator(), DPS310_sensor[i].dps_address); // DPS310-76, DPS310-77
      }

      char str_pressure[33];
      dtostrfd(DPS310_sensor[i].dps_pressure, Settings->flag2.pressure_resolution, str_pressure);

      if (json) {
        ResponseAppend_P(PSTR(",\"%s\":{\"" D_JSON_TEMPERATURE "\":%*_f,\"" D_JSON_PRESSURE "\":%s"),
          sensor_name, Settings->flag2.temperature_resolution, &DPS310_sensor[i].dps_temperature,  str_pressure);
        ResponseJsonEnd();
#ifdef USE_DOMOTICZ
        // Domoticz and knx only support one temp sensor
        if ((0 == TasmotaGlobal.tele_period) && (0 == i)) {
          DomoticzFloatSensor(DZ_TEMP, DPS310_sensor[i].dps_temperature);
        }
#endif // USE_DOMOTICZ
#ifdef USE_WEBSERVER
      } else {
        WSContentSend_Temp(sensor_name, DPS310_sensor[i].dps_temperature);
        WSContentSend_PD(HTTP_SNS_PRESSURE, sensor_name, str_pressure, PressureUnit().c_str());
#endif // USE_WEBSERVER
      }
    }
  }
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/


const char kDPSCommands[] PROGMEM = "DPS310|"  // Prefix
  "Help|OversamplingTemp|OversamplingPress";

void (* const DPSCommand[])(void) PROGMEM = {
  &CmndDPSHelp, &CmndOversamplingTemperature, &CmndOversamplingPressure};


void CmndDPSHelp(void) {
  Response_P(PSTR("Available commands: DPS310Help (This help), DPS310OversamplingTemp [devicenr] [rate] (Get/Set Temperature Oversampling rate), DPS310OversamplingPress [devicenr] [rate] (Get/Set Pressure Oversampling rate). Possible values: devicenr=1-%d (for device 1 till %d), rate=1-7  (for Oversampling 1/2/4/8/16/32/64/128 times) "), dps_count, dps_count);
}

void CmndOversamplingTemperature(void) {
  uint32_t values[2] = { 0 };
  ParseParameters(2, values);

  if(XdrvMailbox.data_len >= 1 && XdrvMailbox.data_len < 6) {
    if (values[0] >= 1 && values[0] < (dps_count + 1)) {
      if (values[1] == 0) {
        //get value from single device
        char sensor_name[12];
        strlcpy(sensor_name, DPS310_sensor[values[0]-1].dps_name, sizeof(DPS310_sensor[values[0]-1].dps_name));
        snprintf_P(sensor_name, sizeof(sensor_name), PSTR("%s%c%02X"), sensor_name, IndexSeparator(), DPS310_sensor[values[0]-1].dps_address); // DPS310-76, DPS310-77
        Response_P(PSTR("{\"%s\": {\"OversamplingTemperature\":%i}}"), sensor_name, DPS310_sensor[values[0]-1].dps_oversampling_temperature);
        return;        
      }
      else
      {
        //set value for single device
        if (values[1] >= 1 && values[1] < 8) {
          DPS310_sensor[values[0]-1].dps_oversampling_temperature = values[1];
          // configure Temperature settings
          uint8_t ret = configTemp(DPS__MEASUREMENT_RATE_4, DPS310_sensor[values[0]-1].dps_oversampling_temperature, values[0]-1);
          Response_P(S_JSON_SENSOR_INDEX_SVALUE, XSNS_128, "Temperature Oversampling set");
          return;
        }
      }
      Response_P(S_JSON_SENSOR_INDEX_SVALUE, XSNS_128, "invalid Temperature Oversampling rate [1/2/4/8/16/32/64/128]");
      return; 
    }
  }
  else if(XdrvMailbox.data_len == 0) {
    //get value from all devices
    if (dps_count > 1) {
      char sensor_name[12];      
      Response_P(PSTR("{"));
      for (uint32_t i = 0; i < dps_count; i++) {
        strlcpy(sensor_name, DPS310_sensor[i].dps_name, sizeof(DPS310_sensor[i].dps_name));
        snprintf_P(sensor_name, sizeof(sensor_name), PSTR("%s%c%02X"), sensor_name, IndexSeparator(), DPS310_sensor[i].dps_address); // DPS310-76, DPS310-77
        ResponseAppend_P(PSTR("\"%s\": {\"OversamplingTemperature\":%i}"), sensor_name, DPS310_sensor[i].dps_oversampling_temperature);
        if (i <= (dps_count - 2)) { 
          ResponseAppend_P(PSTR(","));
        }
      }
      ResponseJsonEnd();    
    }
    else
    {
      Response_P(PSTR("{\"%s\": {\"OversamplingTemperature\":%i}}"), DPS310_sensor[0].dps_name, DPS310_sensor[0].dps_oversampling_temperature);
    }
    return;
  }
  Response_P(S_JSON_SENSOR_INDEX_SVALUE, XSNS_128, "invalid device nr [1-4]");
}

void CmndOversamplingPressure(void) {
  uint32_t values[2] = { 0 };
  ParseParameters(2, values);

  if(XdrvMailbox.data_len >= 1 && XdrvMailbox.data_len < 6) {
    if (values[0] >= 1 && values[0] < (dps_count + 1)) {
      if (values[1] == 0) {
        //get value from single device
        char sensor_name[12];
        strlcpy(sensor_name, DPS310_sensor[values[0]-1].dps_name, sizeof(DPS310_sensor[values[0]-1].dps_name));
        snprintf_P(sensor_name, sizeof(sensor_name), PSTR("%s%c%02X"), sensor_name, IndexSeparator(), DPS310_sensor[values[0]-1].dps_address); // DPS310-76, DPS310-77
        Response_P(PSTR("{\"%s\": {\"OversamplingPressure\":%i}}"), sensor_name, DPS310_sensor[values[0]-1].dps_oversampling_pressure);
        return;        
      }
      else
      {
        //set value for single device
        if (values[1] >= 1 && values[1] < 8) {
          DPS310_sensor[values[0]-1].dps_oversampling_pressure = values[1];
          // configure Pressure settings
          uint8_t ret = configPressure(DPS__MEASUREMENT_RATE_4, DPS310_sensor[values[0]-1].dps_oversampling_pressure, values[0]-1);
          Response_P(S_JSON_SENSOR_INDEX_SVALUE, XSNS_128, "Pressure Oversampling set");
          return;
        }
      }
      Response_P(S_JSON_SENSOR_INDEX_SVALUE, XSNS_128, "invalid Pressure Oversampling rate [1/2/4/8/16/32/64/128]");
      return; 
    }
  }
  else if(XdrvMailbox.data_len == 0) {
    //get value from all devices
    if (dps_count > 1) {
      char sensor_name[12];      
      Response_P(PSTR("{"));
      for (uint32_t i = 0; i < dps_count; i++) {
        strlcpy(sensor_name, DPS310_sensor[i].dps_name, sizeof(DPS310_sensor[i].dps_name));
        snprintf_P(sensor_name, sizeof(sensor_name), PSTR("%s%c%02X"), sensor_name, IndexSeparator(), DPS310_sensor[i].dps_address); // DPS310-76, DPS310-77
        ResponseAppend_P(PSTR("\"%s\": {\"OversamplingPressure\":%i}"), sensor_name, DPS310_sensor[i].dps_oversampling_pressure);
        if (i <= (dps_count - 2)) { 
          ResponseAppend_P(PSTR(","));
        }
      }
      ResponseJsonEnd();    
    }
    else
    {
      Response_P(PSTR("{\"%s\": {\"OversamplingPressure\":%i}}"), DPS310_sensor[0].dps_name, DPS310_sensor[0].dps_oversampling_pressure);
    }
    return;
  }
  Response_P(S_JSON_SENSOR_INDEX_SVALUE, XSNS_128, "invalid device nr [1-4]");  
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
    case FUNC_COMMAND:
      result = DecodeCommand(kDPSCommands, DPSCommand);
      break;
    }
  }
  return result;
}

#endif // USE_DPS3X
#endif // USE_I2C
