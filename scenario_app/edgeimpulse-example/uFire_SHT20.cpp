#include "uFire_SHT20.h"
#include "cmath"

#ifdef HIMAX_SHT20
#else
#include <Arduino.h>
#include <Wire.h>
#endif

#ifdef HIMAX_SHT20
bool uFire_SHT20::begin()
{
  return connected();
}
#else
bool uFire_SHT20::begin(uint8_t resolution, uint8_t address, TwoWire &wirePort)
{
   _address = address;
   _resolution = resolution;
  _i2cPort = &wirePort;


  return connected();
}
#endif

void uFire_SHT20::_reset()
{
  uint8_t regAddr[1] = {(uint8_t)SHT20_RESET};
  #ifdef HIMAX_SHT20
  hx_drv_i2cm_set_data((uint8_t)SHT20_I2C, regAddr, 1, 0, 0) ;
  hx_util_delay_ms(SOFT_RESET_DELAY);
  //hx_drv_uart_print("reset ok\n");
  #else  
  _i2cPort->beginTransmission(SHT20_I2C);
  _i2cPort->write(SHT20_RESET);
  _i2cPort->endTransmission();
  delay(SOFT_RESET_DELAY);
  #endif

  _onchip_heater = _DISABLE_ONCHIP_HEATER;
  _otp_reload = _DISABLE_OTP_RELOAD;
  uint8_t config;
  uint8_t regAddr2[1] = {(uint8_t)SHT20_READ_USER_REG};  
  #ifdef HIMAX_SHT20
	hx_drv_i2cm_set_data(SHT20_I2C,regAddr2, 1, 0, 0);
	hx_drv_i2cm_get_data(SHT20_I2C,NULL,0,&config,1);
  //hx_drv_uart_print("read register ok\n");
  #else
  _i2cPort->beginTransmission(SHT20_I2C);
  _i2cPort->write(SHT20_READ_USER_REG);
  _i2cPort->endTransmission();
  _i2cPort->requestFrom(SHT20_I2C, 1);
  config = _i2cPort->read();  
  #endif

  uint8_t regAddr1[1] = {(uint8_t)SHT20_WRITE_USER_REG};  
  config = ((config & _RESERVED_BITMASK) | _resolution | _onchip_heater | _otp_reload);
  #ifdef HIMAX_SHT20
  hx_drv_i2cm_set_data(SHT20_I2C,regAddr1, 1,&config , 1);
  //hx_drv_uart_print("write register ok\n");
  #else
  _i2cPort->beginTransmission(SHT20_I2C);
  _i2cPort->write(SHT20_WRITE_USER_REG);
  _i2cPort->write(config);
  _i2cPort->endTransmission();
  #endif
}

void uFire_SHT20::measure_all()
{
  // also measures temp/humidity
  vpd();
  dew_point();
}

float uFire_SHT20::temperature()
{
  uint8_t buffer[2];
  uint8_t regAddr[1] = {(uint8_t)SHT20_TEMP};
  #ifdef HIMAX_SHT20  
  _reset();
	hx_drv_i2cm_set_data(SHT20_I2C,regAddr, 1, 0, 0);
  hx_util_delay_ms(TEMPERATURE_DELAY);
	hx_drv_i2cm_get_data(SHT20_I2C,NULL,0,buffer,2);
  //hx_drv_uart_print("temperature ok\n");
  #else  
  uint8_t i; 
  _reset();
  _i2cPort->beginTransmission(SHT20_I2C);
  _i2cPort->write(SHT20_TEMP);
  _i2cPort->endTransmission();
  delay(TEMPERATURE_DELAY);
  _i2cPort->requestFrom(SHT20_I2C, 2);
  for (i = 0; i < 2; i++)
    buffer[i] = _i2cPort->read();
  #endif
  uint16_t value  = (buffer[0] << 8) | buffer[1];
  tempC = value * (175.72 / 65536.0)- 46.85;
  tempF = ((value * (175.72 / 65536.0)- 46.85)  * 1.8) + 32; 
  return tempC;
}

float uFire_SHT20::temperature_f()
{
  uint8_t buffer[2];
  uint8_t regAddr[1] = {(uint8_t)SHT20_TEMP};
  #ifdef HIMAX_SHT20  
  _reset();
	hx_drv_i2cm_set_data(SHT20_I2C,regAddr, 1, 0, 0);
  hx_util_delay_ms(TEMPERATURE_DELAY);  
	hx_drv_i2cm_get_data(SHT20_I2C,NULL,0,buffer,2);
  //hx_drv_uart_print("temperature ok\n");
  #else 
  uint8_t i;   
  _reset();
  _i2cPort->beginTransmission(SHT20_I2C);
  _i2cPort->write(SHT20_TEMP);
  _i2cPort->endTransmission();
  delay(TEMPERATURE_DELAY);
  _i2cPort->requestFrom(SHT20_I2C, 2);
  for (i = 0; i < 2; i++)
    buffer[i] = _i2cPort->read();
  #endif
  uint16_t value  = (buffer[0] << 8) | buffer[1];
  tempC = value * (175.72 / 65536.0)- 46.85;
  tempF = ((value * (175.72 / 65536.0)- 46.85)  * 1.8) + 32;
  return tempF;
}

float uFire_SHT20::humidity()
{
  uint8_t buffer[2];
  uint8_t regAddr[1] = {(uint8_t)SHT20_HUMID};
  #ifdef HIMAX_SHT20 
  _reset(); 
	hx_drv_i2cm_set_data(SHT20_I2C,regAddr, 1, 0, 0);
  hx_util_delay_ms(HUMIDITY_DELAY);
	hx_drv_i2cm_get_data(SHT20_I2C,NULL,0,buffer,2);
  //hx_drv_uart_print("humidity ok\n");
  #else  
  uint8_t i; 
  _reset();
  _i2cPort->beginTransmission(SHT20_I2C);
  _i2cPort->write(SHT20_HUMID);
  _i2cPort->endTransmission();
  delay(HUMIDITY_DELAY);
  _i2cPort->requestFrom(SHT20_I2C, 2);
  for (i = 0; i < 2; i++)
    buffer[i] = _i2cPort->read();
  #endif
  uint16_t value  = (buffer[0] << 8) | buffer[1];
  RH = value * (125.0 / 65536.0) - 6.0;

  return RH;
}

float uFire_SHT20::vpd()
{
  temperature();
  humidity();

  float es = 0.6108 * exp(17.27 * tempC / (tempC + 237.3));
  float ae = RH / 100 * es;
  vpd_kPa = es - ae;

  return vpd_kPa;
}

float uFire_SHT20::dew_point()
{
  temperature();
  humidity();

  float tem = -1.0 * tempC;
  float esdp = 6.112 * exp(-1.0 * 17.67 * tem / (243.5 - tem));
  float ed = RH / 100.0 * esdp;
  float eln = log(ed / 6.112);
  dew_pointC = -243.5 * eln / (eln - 17.67 );

  dew_pointF = (dew_pointC * 1.8) + 32;
  return dew_pointC;
}

bool uFire_SHT20::connected()
{
#ifdef HIMAX_SHT20 
  return true; 
#else
  _i2cPort->beginTransmission(SHT20_I2C);
  _i2cPort->write(SHT20_READ_USER_REG);
  _i2cPort->endTransmission();
  _i2cPort->requestFrom(SHT20_I2C, 1);
  uint8_t config = _i2cPort->read();

  if (config != 0xFF) {
    return true;
  }
  else {
    return false;
  }
#endif  
}