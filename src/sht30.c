//============================================================================= 
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland 
//============================================================================= 
// Project   :  SHT3x Sample Code (V1.1) 
// File      :  sht3x.c (V1.1) 
// Author    :  RFU 
// Date      :  6-Mai-2015 
// Controller:  STM32F100RB 
// IDE       :  μVision V5.12.0.0 
// Compiler  :  Armcc 
// Brief     :  Sensor Layer: Implementation of functions for sensor access. 
//============================================================================= 
 
//-- Includes ----------------------------------------------------------------- 
#include "sht30.h" 
#include "iic_app.h" 
#include "i2c.h" 
//-- Defines ------------------------------------------------------------------ 
// Generator polynomial for CRC 
#define POLYNOMIAL  0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001 
 
//============================================================================= 
// IO-Pins                            /* -- adapt the defines for your uC -- */ 
//----------------------------------------------------------------------------- 
// Reset on port B, bit 12 
//#define RESET_LOW()  (GPIOB->BSRR = 0x10000000) // set Reset to low 
//#define RESET_HIGH() (GPIOB->BSRR = 0x00001000) // set Reset to high 
 
// Alert on port B, bit 10 
//#define ALERT_READ   (GPIOB->IDR  & 0x0400)     // read Alert 
//============================================================================= 
 
//-- Global variables --------------------------------------------------------- 
static uint8_t _i2cAddress; // I2C Address 使用8位地址表示！！！！！，程序中不再左移1位
static iic_index_t  _iic_index;

float        g_temperature; // temperature [°C] 
float        g_humidity;    // relative humidity [%RH] 


//-- Static function prototypes ----------------------------------------------- 
static etError SHT3X_WriteAlertLimitData(float humidity, float temperature); 
static etError SHT3X_ReadAlertLimitData(float* humidity, float* temperature); 
static etError SHT3X_StartWriteAccess(void); 
static etError SHT3X_StartReadAccess(void); 
static void SHT3X_StopAccess(void); 
static etError SHT3X_WriteCommand(etCommands command); 
static etError SHT3X_Read2BytesAndCrc(uint16_t* data, etI2cAck finaleAckNack, 
                                      uint8_t timeout); 
static etError SHT3X_Write2BytesAndCrc(uint16_t data); 
static uint8_t SHT3X_CalcCrc(uint8_t data[], uint8_t nbrOfBytes); 
static etError SHT3X_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum); 
static float SHT3X_CalcTemperature(uint16_t rawValue); 
static float SHT3X_CalcHumidity(uint16_t rawValue); 
static uint16_t SHT3X_CalcRawTemperature(float temperature); 
static uint16_t SHT3X_CalcRawHumidity(float humidity); 
 
//----------------------------------------------------------------------------- 
void SHT3X_Init(uint8_t i2cAddress)          /* -- adapt the init for your uC -- */ 
{ 
	etError error; // error code 
	IicApp_Init(IIC1_INDEX);
	_iic_index = IIC1_INDEX;
  // init I/O-pins 
//  RCC->APB2ENR |= 0x00000008;  // I/O port B clock enabled 
//   
//  // Alert on port B, bit 10 
//  GPIOB->CRH   &= 0xFFFFF0FF;  // set floating input for Alert-Pin 
//  GPIOB->CRH   |= 0x00000400;  // 
//  
//  // Reset on port B, bit 12 
//  GPIOB->CRH   &= 0xFFF0FFFF;  // set push-pull output for Reset pin 
//  GPIOB->CRH   |= 0x00010000;  // 
//  RESET_LOW(); 
//   
//  I2c_Init(); // init I2C 
	SHT3X_SetI2cAdr(i2cAddress); 
//   
//  // release reset 
//  RESET_HIGH(); 
	
	
	error = SHT3X_StartPeriodicMeasurment(REPEATAB_HIGH, FREQUENCY_1HZ);
	if(error != NO_ERROR) 
		printf("ERROR: SHT3X_Init SHT3X_StartPeriodicMeasurment\r\n");
	
} 
 
//----------------------------------------------------------------------------- 
void SHT3X_SetI2cAdr(uint8_t i2cAddress) 
{ 
	_i2cAddress = i2cAddress; 
} 
 
//----------------------------------------------------------------------------- 
etError SHT3x_ReadSerialNumber(uint32_t* serialNumber) 
{ 
  etError error; // error code 
  uint16_t serialNumWords[2]; 
   
  error = SHT3X_StartWriteAccess(); 
   
  // write "read serial number" command 
  error |= SHT3X_WriteCommand(CMD_READ_SERIALNBR); 
  // if no error, start read access 
  if(error == NO_ERROR) error = SHT3X_StartReadAccess(); 
  // if no error, read first serial number word 
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&serialNumWords[0], ACK, 100); 
  // if no error, read second serial number word 
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&serialNumWords[1], NACK, 0); 
   
  SHT3X_StopAccess(); 
   
  // if no error, calc serial number as 32-bit integer 
  if(error == NO_ERROR) 
  { 
    *serialNumber = (serialNumWords[0] << 16) | serialNumWords[1]; 
  } 
   
  return error; 
} 
 
//----------------------------------------------------------------------------- 
etError SHT3X_ReadStatus(uint16_t* status) 
{ 
  etError error; // error code 
   
  error = SHT3X_StartWriteAccess(); 
   
  // if no error, write "read status" command 
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_READ_STATUS); 
  // if no error, start read access 
  if(error == NO_ERROR) error = SHT3X_StartReadAccess();  
  // if no error, read status 
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(status, NACK, 0);     
  SHT3X_StopAccess(); 
   
  return error; 
} 
 
//----------------------------------------------------------------------------- 
etError SHT3X_ClearAllAlertFlags(void) 
{ 
  etError error; // error code 
   
  error = SHT3X_StartWriteAccess(); 
   
  // if no error, write clear status register command 
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_CLEAR_STATUS); 
   
  SHT3X_StopAccess(); 
   
  return error; 
} 
 
//----------------------------------------------------------------------------- 
etError SHT3X_GetTempAndHumi(float* temperature, float* humidity, 
                             etRepeatability repeatability, etMode mode, 
                             uint8_t timeout) 
{ 
  etError error; 
                                
  switch(mode) 
  {     
    case MODE_CLKSTRETCH: // get temperature with clock stretching mode 
      error = SHT3X_GetTempAndHumiClkStretch(temperature, humidity, 
                                             repeatability, timeout); 
      break; 
    case MODE_POLLING:    // get temperature with polling mode 
      error = SHT3X_GetTempAndHumiPolling(temperature, humidity, 
                                          repeatability, timeout); 
      break; 
    default:               
      error = PARM_ERROR; 
      break; 
  } 
   
  return error; 
} 
 
 
//----------------------------------------------------------------------------- 
etError SHT3X_GetTempAndHumiClkStretch(float* temperature, float* humidity, 
                                       etRepeatability repeatability, 
                                       uint8_t timeout) 
{ 
	etError error;        // error code 
	uint16_t    rawValueTemp; // temperature raw value from sensor 
	uint16_t    rawValueHumi; // humidity raw value from sensor 

#if 1	
	error = SHT3X_StartWriteAccess(); 

	// if no error ... 
	if(error == NO_ERROR) 
	{ 
	// start measurement in clock stretching mode 
	// use depending on the required repeatability, the corresponding command     
	switch(repeatability) 
	{ 
	  case REPEATAB_LOW: 
		error = SHT3X_WriteCommand(CMD_MEAS_CLOCKSTR_L); 
		break; 
	  case REPEATAB_MEDIUM: 
		error = SHT3X_WriteCommand(CMD_MEAS_CLOCKSTR_M); 
		break; 
	  case REPEATAB_HIGH: 
		error = SHT3X_WriteCommand(CMD_MEAS_CLOCKSTR_H); 
		break; 
	  default: 
		error = PARM_ERROR; 
		break; 
	} 
	} 

	// if no error, start read access 
	if(error == NO_ERROR) error = SHT3X_StartReadAccess(); 
	// if no error, read temperature raw values 
	if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&rawValueTemp, ACK, timeout); 
	// if no error, read humidity raw values 
	if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&rawValueHumi, NACK, 0); 

	SHT3X_StopAccess(); 

	// if no error, calculate temperature in °C and humidity in %RH 
	if(error == NO_ERROR) 
	{ 
		*temperature = SHT3X_CalcTemperature(rawValueTemp); 
		*humidity = SHT3X_CalcHumidity(rawValueHumi); 
	} 
#else
	uint16_t word_addr;
	switch(repeatability) 
	{ 
	  case REPEATAB_LOW: 
		word_addr = CMD_MEAS_CLOCKSTR_L; 
		break; 
	  case REPEATAB_MEDIUM: 
		word_addr = CMD_MEAS_CLOCKSTR_M; 
		break; 
	  case REPEATAB_HIGH: 
		word_addr = CMD_MEAS_CLOCKSTR_H; 
		break; 
	  default: 
		error = PARM_ERROR; 
		break; 
	} 
	
	
	IicApp_Read_Bytes(_iic_index,_i2cAddress,uint8_t word_addr,uint8_t *dat,uint8_t len);
#endif
  return error; 
} 
 
//----------------------------------------------------------------------------- 
etError SHT3X_GetTempAndHumiPolling(float* temperature, float* humidity, 
                                    etRepeatability repeatability, 
                                    uint8_t timeout) 
{ 
	etError error;           // error code 
	uint16_t    rawValueTemp;    // temperature raw value from sensor 
	uint16_t    rawValueHumi;    // humidity raw value from sensor 

	error  = SHT3X_StartWriteAccess(); 

	// if no error ... 
	if(error == NO_ERROR) 
	{ 
	// start measurement in polling mode 
	// use depending on the required repeatability, the corresponding command 
	switch(repeatability) 
	{ 
	  case REPEATAB_LOW: 
		error = SHT3X_WriteCommand(CMD_MEAS_POLLING_L); 
		break; 
	  case REPEATAB_MEDIUM: 
		error = SHT3X_WriteCommand(CMD_MEAS_POLLING_M); 
		break; 
	  case REPEATAB_HIGH: 
		error = SHT3X_WriteCommand(CMD_MEAS_POLLING_H);     
		break; 
	  default: 
	   error = PARM_ERROR; 
		break; 
	} 
	} 

	// if no error, wait until measurement ready 
	if(error == NO_ERROR) 
	{ 
		// poll every 1ms for measurement ready until timeout 
		while(timeout--) 
		{ 
			// check if the measurement has finished 
			error = SHT3X_StartReadAccess(); 

			// if measurement has finished -> exit loop 
			if(error == NO_ERROR) break; 

			// delay 1ms 
			Delay1ms(1); 
		} 
	}
	// if no error, read temperature and humidity raw values 
	if(error == NO_ERROR) 
	{ 
		error |= SHT3X_Read2BytesAndCrc(&rawValueTemp, ACK, 0); 
		error |= SHT3X_Read2BytesAndCrc(&rawValueHumi, NACK, 0); 
	} 

	SHT3X_StopAccess(); 

	// if no error, calculate temperature in °C and humidity in %RH 
	if(error == NO_ERROR) 
	{ 
		*temperature = SHT3X_CalcTemperature(rawValueTemp); 
		*humidity = SHT3X_CalcHumidity(rawValueHumi); 
	} 

	return error; 
} 
 
//----------------------------------------------------------------------------- 
etError SHT3X_StartPeriodicMeasurment(etRepeatability repeatability, 
                                      etFrequency frequency) 
{ 
  etError error;        // error code 
   
	error = SHT3X_StartWriteAccess(); 
	if(error)
	{
		printf("ERROR: SHT3X_StartPeriodicMeasurment SHT3X_StartWriteAccess\r\n");
		return PARM_ERROR;
	}
  // if no error, start periodic measurement  
  if(error == NO_ERROR) 
  { 
    // use depending on the required repeatability and frequency, 
    // the corresponding command 
    switch(repeatability) 
    { 
      case REPEATAB_LOW: // low repeatability 
        switch(frequency) 
        { 
          case FREQUENCY_HZ5:  // low repeatability,  0.5 Hz 
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_05_L); 
           break;           
          case FREQUENCY_1HZ:  // low repeatability,  1.0 Hz 
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_1_L); 
            break;           
          case FREQUENCY_2HZ:  // low repeatability,  2.0 Hz 
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_2_L); 
            break;           
          case FREQUENCY_4HZ:  // low repeatability,  4.0 Hz 
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_4_L); 
            break;           
          case FREQUENCY_10HZ: // low repeatability, 10.0 Hz 
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_10_L); 
            break;           
          default: 
            error |= PARM_ERROR; 
            break; 
        } 
        break; 
         
      case REPEATAB_MEDIUM: // medium repeatability 
        switch(frequency) 
        { 
          case FREQUENCY_HZ5:  // medium repeatability,  0.5 Hz 
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_05_M); 
      break; 
          case FREQUENCY_1HZ:  // medium repeatability,  1.0 Hz 
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_1_M); 
      break;         
          case FREQUENCY_2HZ:  // medium repeatability,  2.0 Hz 
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_2_M); 
      break;         
          case FREQUENCY_4HZ:  // medium repeatability,  4.0 Hz 
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_4_M); 
      break;       
          case FREQUENCY_10HZ: // medium repeatability, 10.0 Hz 
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_10_M); 
      break; 
          default: 
            error |= PARM_ERROR; 
      break; 
        } 
        break; 
         
      case REPEATAB_HIGH: // high repeatability 
        switch(frequency) 
        { 
          case FREQUENCY_HZ5:  // high repeatability,  0.5 Hz 
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_05_H); 
            break; 
          case FREQUENCY_1HZ:  // high repeatability,  1.0 Hz 
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_1_H); 
            break; 
          case FREQUENCY_2HZ:  // high repeatability,  2.0 Hz 
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_2_H); 
            break; 
          case FREQUENCY_4HZ:  // high repeatability,  4.0 Hz 
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_4_H); 
            break; 
          case FREQUENCY_10HZ: // high repeatability, 10.0 Hz 
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_10_H); 
            break; 
          default: 
            error |= PARM_ERROR; 
            break; 
        } 
        break; 
      default: 
        error |= PARM_ERROR; 
        break; 
    } 
  } 
 
  SHT3X_StopAccess(); 
 
  return error; 
} 
 
//----------------------------------------------------------------------------- 
etError SHT3X_ReadMeasurementBuffer(float* temperature, float* humidity) 
{ 
	etError  error;        // error code 
	uint16_t     rawValueTemp; // temperature raw value from sensor 
	uint16_t     rawValueHumi; // humidity raw value from sensor 

	error = SHT3X_StartWriteAccess(); 
	if(error)
	{
		printf("ERROR: SHT3X_StartWriteAccess\r\n");
		return error;
	}
	// if no error, read measurements 
	if(error == NO_ERROR) 
		error = SHT3X_WriteCommand(CMD_FETCH_DATA); 
	if(error)
	{
		printf("ERROR: SHT3X_WriteCommand\r\n");
		return error;
	}
	if(error == NO_ERROR) 
		error = SHT3X_StartReadAccess();   
	if(error)
	{
		printf("ERROR: SHT3X_StartReadAccess\r\n");
		return error;
	}
	if(error == NO_ERROR) 
		error = SHT3X_Read2BytesAndCrc(&rawValueTemp, ACK, 0); 
	if(error)
	{
		printf("ERROR: SHT3X_Read2BytesAndCrc\r\n");
		return error;
	} 
	if(error == NO_ERROR) 
		error = SHT3X_Read2BytesAndCrc(&rawValueHumi, NACK, 0); 
	if(error)
	{
		printf("ERROR: SHT3X_Read2BytesAndCrc\r\n");
		return error;
	} 
	// if no error, calculate temperature in °C and humidity in %RH 
	if(error == NO_ERROR) 
	{ 
	*temperature = SHT3X_CalcTemperature(rawValueTemp); 
	*humidity = SHT3X_CalcHumidity(rawValueHumi); 
	} 

	SHT3X_StopAccess(); 

	return error; 
} 
 
//----------------------------------------------------------------------------- 
etError SHT3X_EnableHeater(void) 
{ 
  etError error; // error code 
 
  error = SHT3X_StartWriteAccess(); 
 
  // if no error, write heater enable command 
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_HEATER_ENABLE); 
 
  SHT3X_StopAccess(); 
 
  return error; 
} 
 
//----------------------------------------------------------------------------- 
etError SHT3X_DisableHeater(void) 
{ 
  etError error; // error code     
  error = SHT3X_StartWriteAccess(); 
 
  // if no error, write heater disable command 
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_HEATER_DISABLE); 
 
  SHT3X_StopAccess(); 
 
  return error; 
} 
 
 
//----------------------------------------------------------------------------- 
etError SHT3X_SetAlertLimits(float humidityHighSet,   float temperatureHighSet, 
                             float humidityHighClear, float temperatureHighClear, 
                             float humidityLowClear,  float temperatureLowClear, 
                             float humidityLowSet,    float temperatureLowSet) 
{ 
  etError  error;  // error code 
   
  // write humidity & temperature alter limits, high set 
  error = SHT3X_StartWriteAccess(); 
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_W_AL_LIM_HS); 
  if(error == NO_ERROR) error = SHT3X_WriteAlertLimitData(humidityHighSet, 
                                                          temperatureHighSet); 
  SHT3X_StopAccess(); 
 
  if(error == NO_ERROR) 
  { 
    // write humidity & temperature alter limits, high clear 
    error = SHT3X_StartWriteAccess(); 
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_W_AL_LIM_HC); 
    if(error == NO_ERROR) error = SHT3X_WriteAlertLimitData(humidityHighClear, 
                                                            temperatureHighClear); 
    SHT3X_StopAccess(); 
  } 
 
  if(error == NO_ERROR) 
  { 
    // write humidity & temperature alter limits, low clear 
    error = SHT3X_StartWriteAccess(); 
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_W_AL_LIM_LC); 
    if(error == NO_ERROR) error = SHT3X_WriteAlertLimitData(humidityLowClear, 
                                                            temperatureLowClear); 
    SHT3X_StopAccess(); 
  } 
   
  if(error == NO_ERROR) 
  { 
    // write humidity & temperature alter limits, low set 
    error = SHT3X_StartWriteAccess(); 
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_W_AL_LIM_LS); 
    if(error == NO_ERROR) error = SHT3X_WriteAlertLimitData(humidityLowSet, 
                                                            temperatureLowSet); 
    SHT3X_StopAccess(); 
  } 
 
  return error; 
} 
 
//----------------------------------------------------------------------------- 
etError SHT3X_GetAlertLimits(float* humidityHighSet,   float* temperatureHighSet, 
                             float* humidityHighClear, float* temperatureHighClear,     
                             float* humidityLowClear,  float* temperatureLowClear, 
                             float* humidityLowSet,    float* temperatureLowSet) 
{ 
  etError  error;  // error code 
   
  // read humidity & temperature alter limits, high set 
  error = SHT3X_StartWriteAccess(); 
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_R_AL_LIM_HS); 
  if(error == NO_ERROR) error = SHT3X_StartReadAccess(); 
  if(error == NO_ERROR) error = SHT3X_ReadAlertLimitData(humidityHighSet, 
                                                         temperatureHighSet); 
  SHT3X_StopAccess(); 
 
  if(error == NO_ERROR) 
  { 
    // read humidity & temperature alter limits, high clear 
    error = SHT3X_StartWriteAccess(); 
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_R_AL_LIM_HC); 
    if(error == NO_ERROR) error = SHT3X_StartReadAccess(); 
    if(error == NO_ERROR) error = SHT3X_ReadAlertLimitData(humidityHighClear, 
                                                           temperatureHighClear); 
    SHT3X_StopAccess(); 
  } 
 
  if(error == NO_ERROR) 
  { 
    // read humidity & temperature alter limits, low clear 
    error = SHT3X_StartWriteAccess(); 
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_R_AL_LIM_LC); 
    if(error == NO_ERROR) error = SHT3X_StartReadAccess(); 
    if(error == NO_ERROR) error = SHT3X_ReadAlertLimitData(humidityLowClear, 
                                                           temperatureLowClear); 
    SHT3X_StopAccess(); 
  } 
 
  if(error == NO_ERROR) 
  { 
    // read humidity & temperature alter limits, low set 
    error = SHT3X_StartWriteAccess(); 
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_R_AL_LIM_LS); 
    if(error == NO_ERROR) error = SHT3X_StartReadAccess(); 
    if(error == NO_ERROR) error = SHT3X_ReadAlertLimitData(humidityLowSet, 
                                                           temperatureLowSet); 
    SHT3X_StopAccess(); 
  } 
  
  return error; 
} 
                              
//----------------------------------------------------------------------------- 
uint8_t SHT3X_ReadAlert(void) 
{ 
  // read alert pin 
//  return (ALERT_READ != 0) ? TRUE : FALSE; 
	return TRUE;
} 
 
//----------------------------------------------------------------------------- 
etError SHT3X_SofloatReset(void) 
{ 
  etError error; // error code 
 
  error = SHT3X_StartWriteAccess(); 

  // write reset command 
  error |= SHT3X_WriteCommand(CMD_SOfloat_RESET); 
 
  SHT3X_StopAccess(); 
   
  // if no error, wait 50 ms afloater reset 
  if(error == NO_ERROR) Delay1ms(50);  
 
  return error; 
} 
 
//----------------------------------------------------------------------------- 
void SHT3X_HardReset(void) 
{ 
  // set reset low 
//  RESET_LOW(); 
 
  // wait 100 ms 
  Delay1ms(100); 
   
  // release reset 
//  RESET_HIGH(); 
   
  // wait 50 ms afloater reset 
  Delay1ms(50); 
} 
 
                              
//----------------------------------------------------------------------------- 
static etError SHT3X_WriteAlertLimitData(float humidity, float temperature) 
{ 
  etError  error;           // error code 
   
  int16_t rawHumidity; 
  int16_t rawTemperature; 
   
  if((humidity < 0.0f) || (humidity > 100.0f)  
  || (temperature < -45.0f) || (temperature > 130.0f)) 
  { 
    error = PARM_ERROR; 
  } 
  else 
  { 
    rawHumidity    = SHT3X_CalcRawHumidity(humidity); 
    rawTemperature = SHT3X_CalcRawTemperature(temperature); 
 
    error = SHT3X_Write2BytesAndCrc((rawHumidity & 0xFE00) | ((rawTemperature >> 7) & 0x001FF)); 
  } 
   
  return error; 
} 
 
//----------------------------------------------------------------------------- 
static etError SHT3X_ReadAlertLimitData(float* humidity, float* temperature) 
{ 
  etError  error;           // error code 
  uint16_t     data; 
   
  error = SHT3X_Read2BytesAndCrc(&data, NACK, 0); 
  
  if(error == NO_ERROR) 
  { 
    *humidity = SHT3X_CalcHumidity(data & 0xFE00); 
    *temperature = SHT3X_CalcTemperature(data << 7); 
  } 
   
  return error; 
} 
 
//----------------------------------------------------------------------------- 
static etError SHT3X_StartWriteAccess(void) 
{ 
	etError error; // error code 
 
  // write a start condition 
	IIC_Start(_iic_index); 
 
  // write the sensor I2C address with the write flag 
//  IIC_Send_Byte(_iic_index,_i2cAddress); //I2c_WriteByte(_i2cAddress << 1); 
 //	error = IIC_Wait_Ack(_iic_index);
	
	error = I2c_WriteByte(_iic_index,_i2cAddress);
	
  return error; 
} 
 
//----------------------------------------------------------------------------- 
static etError SHT3X_StartReadAccess(void) 
{ 
	etError error; // error code 
 
  // write a start condition 
	IIC_Start(_iic_index);  //I2c_StartCondition(); 
 
  // write the sensor I2C address with the read flag 
//	IIC_Send_Byte(_iic_index,_i2cAddress | 0x01); //error = I2c_WriteByte(_i2cAddress << 1 | 0x01); 
 
	
//	error = IIC_Wait_Ack(_iic_index);
	
	error = I2c_WriteByte(_iic_index,_i2cAddress | 0x01);
	
  return NO_ERROR;//error;  //
} 
 
//----------------------------------------------------------------------------- 
static void SHT3X_StopAccess(void) 
{ 
  // write a stop condition 
  //I2c_StopCondition(); 
	IIC_Stop(_iic_index);
} 
 
//----------------------------------------------------------------------------- 
static etError SHT3X_WriteCommand(etCommands command) 
{ 
	etError error; // error code 

	// write the upper 8 bits of the command to the sensor 
	error  = (etError)I2c_WriteByte(_iic_index , command >> 8); 
	if(error)
	{
		printf("ERROR: SHT3X_WriteCommand1 cmd = %#x\r\n",command);
		return PARM_ERROR;
	}	
	//error  = IIC_Send_Byte( _iic_index , command >> 8);
	// write the lower 8 bits of the command to the sensor 
	error |= (etError)I2c_WriteByte(_iic_index , command & 0xFF); 
	if(error)
	{
		printf("ERROR: SHT3X_WriteCommand2 cmd = %#x\r\n",command);
		return PARM_ERROR;
	}
	

	return error; 
} 
 
//----------------------------------------------------------------------------- 
static etError SHT3X_Read2BytesAndCrc(uint16_t* data, etI2cAck finaleAckNack, 
                                      uint8_t timeout) 
{     
	etError error;    // error code 
	uint8_t     bytes[2]; // read data array 
	uint8_t     checksum; // checksum byte 

	// read two data bytes and one checksum byte 
//	error = I2c_ReadByte(&bytes[0], ACK, timeout); 
//	if(error == NO_ERROR) 
//		error = I2c_ReadByte(&bytes[1], ACK, 0); 
//	if(error == NO_ERROR) 
//		error = I2c_ReadByte(&checksum, finaleAckNack, 0); 
	bytes[0] = IIC_Read_Byte(_iic_index,1);
	bytes[1] = IIC_Read_Byte(_iic_index,1);
	checksum = IIC_Read_Byte(_iic_index,0);   //0 表示 noack
	// verify checksum 
//	if(error == NO_ERROR) 
	
	printf("debug : bytes[0] = 0x%x,[1] = 0x%x,csum = 0x%x\r\n",bytes[0],bytes[1],checksum);
	
	error = SHT3X_CheckCrc(bytes, 2, checksum); 

	// combine the two bytes to a 16-bit value 
	*data = (bytes[0] << 8) | bytes[1]; 

	return error; 
} 
 
//----------------------------------------------------------------------------- 
static etError SHT3X_Write2BytesAndCrc(uint16_t data) 
{ 
	etError error;    // error code 
	uint8_t     bytes[2]; // read data array 
	uint8_t     checksum; // checksum byte 

	bytes[0] = data >> 8; 
	bytes[1] = data & 0xFF; 
	checksum = SHT3X_CalcCrc(bytes, 2); 

	// write two data bytes and one checksum byte 
	error = (etError)I2c_WriteByte(_iic_index,bytes[0]); // write data MSB 
	if(error == NO_ERROR) 
		error = (etError)I2c_WriteByte(_iic_index,bytes[1]); // write data LSB 
	if(error == NO_ERROR)
		error = (etError)I2c_WriteByte(_iic_index,checksum); // write checksum 

	return error; 
} 
 
//----------------------------------------------------------------------------- 
static uint8_t SHT3X_CalcCrc(uint8_t data[], uint8_t nbrOfBytes) 
{ 
	uint8_t bit;        // bit mask 
	uint8_t crc = 0xFF; // calculated checksum 
	uint8_t byteCtr;    // byte counter 

	// calculates 8-Bit checksum with given polynomial 
	for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) 
	{ 
		crc ^= (data[byteCtr]); 
		for(bit = 8; bit > 0; --bit) 
		{ 
		  if(crc & 0x80) 
			  crc = (crc << 1) ^ POLYNOMIAL; 
		  else           
			  crc = (crc << 1); 
		} 
	} 

	return crc; 
} 
 
//----------------------------------------------------------------------------- 
static etError SHT3X_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum) 
{ 
  uint8_t crc;     // calculated checksum 
  
  // calculates 8-Bit checksum 
  crc = SHT3X_CalcCrc(data, nbrOfBytes); 
   
  // verify checksum 
  if(crc != checksum) 
	  return CHECKSUM_ERROR; 
  else                
	  return NO_ERROR; 
} 
 
//----------------------------------------------------------------------------- 
static float SHT3X_CalcTemperature(uint16_t rawValue) 
{ 
  // calculate temperature [°C] 
  // T = -45 + 175 * rawValue / (2^16-1) 
  return 175.0f * (float)rawValue / 65535.0f - 45.0f; 
} 
 
//----------------------------------------------------------------------------- 
static float SHT3X_CalcHumidity(uint16_t rawValue) 
{ 
  // calculate relative humidity [%RH] 
  // RH = rawValue / (2^16-1) * 100 
  return 100.0f * (float)rawValue / 65535.0f; 
} 
 
//----------------------------------------------------------------------------- 
static uint16_t SHT3X_CalcRawTemperature(float temperature) 
{ 
  // calculate raw temperature [ticks] 
  // rawT = (temperature + 45) / 175 * (2^16-1) 
  return (temperature + 45.0f) / 175.0f * 65535.0f; 
} 
 
//----------------------------------------------------------------------------- 
static uint16_t SHT3X_CalcRawHumidity(float humidity) 
{ 
  // calculate raw relative humidity [ticks] 
  // rawRH = humidity / 100 * (2^16-1) 
  return humidity / 100.0f * 65535.0f; 
}


//获取温湿度的任务，1s读取1次
void get_sht30_tmp_task(void)
{
	etError   error;       // error code 
//	float        temperature; // temperature [°C] 
//	float        humidity;    // relative humidity [%RH] 
	// read measurment buffer 
	
	error = SHT3X_ReadMeasurementBuffer(&g_temperature, &g_humidity); 
	if(error == NO_ERROR) 
	{ 
		MY_PRINTF("%s %d temp = %f humi = %f\r\n",__FUNCTION__,__LINE__,g_temperature, g_humidity);
	//new temperature and humidity values 
		if(g_temperature < 30.0)
		{
			pwm_all_change(100);
		}
		else if(g_temperature < 35.0)
		{
			pwm_all_change(70);		
		}
		else if(g_temperature < 40.0)
		{
			pwm_all_change(50);		
		}
		else if(g_temperature < 45.0)
		{
			pwm_all_change(30);		
		}
		else if(g_temperature < 48.0)  //50度了
		{
			pwm_all_change(20);		
		}
		else if(g_temperature < 50.0)  //50度了
		{
			pwm_all_change(10);		
		}
		else //50度以上了
			pwm_all_change(0);
	}
	else
	{
		DBG_PRINTF("ERROR:SHT3X_ReadMeasurementBuffer\r\n");
	}
}


