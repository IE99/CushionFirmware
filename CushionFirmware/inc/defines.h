#ifndef _DEFINE_H_
#define _DEFINE_H_

/*#include <stdio.h>
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Config.h"
#include "sleep.h"
#include "sensor.h"
#include "SensorDemo_config.h"
#include "OTA_btl.h"  
#include "gatt_db.h"
#include "SEGGER_RTT.h"
#include "BlueNRG1_adc.h"
#include "clock.h"
#include <string.h>
#include "gp_timer.h"
#include <stdlib.h>
#include "osal.h"
#include <time.h>*/

#define BLE_SENSOR_VERSION_STRING "1.0.0" 
//#define ADC_READING_OFFSET		0.042 //ADC reading value is always 0.042 greater than actual values

#define CURRENT_PROGRAM_SIZE 64*1024
#define SPI_CS_Memory GPIO_Pin_1 //TODO: NEED to change these to low power pins that can stay high
#define SPI_CS_ADC GPIO_Pin_17

#define Valve1 GPIO_Pin_8
#define Valve2 GPIO_Pin_7
#define Valve3 GPIO_Pin_21
#define Valve4 GPIO_Pin_6
#define Valve5 GPIO_Pin_5
#define Valve6 GPIO_Pin_20
#define Valve7 GPIO_Pin_19
#define Valve8 GPIO_Pin_18
//#define Valve8 GPIO_Pin_22 //TODO Delete this idk what it is
#define ValveFinal GPIO_Pin_8 //TODO: Update

#define ValveControl GPIO_Pin_22
#define PVControl GPIO_Pin_23
#define SensorControl GPIO_Pin_24

#define BLUE_LED GPIO_Pin_25
#define RED_LED GPIO_Pin_4

void init(void);
void Batt_ADC_Configuration(void);
uint16_t ADC_Value(uint8_t input);

void InitGPIO(uint32_t myLED);
void InitGPIOInput(uint32_t myLED);

void GPIO_On(uint32_t myLED);
void GPIO_Off(uint32_t myLED);
void GPIO_Toggle(uint32_t myLED);

void Valve_Open(uint8_t pin);
void Valve_Close(uint8_t pin);

//static uint8_t wakeup_source = WAKEUP_IO11; // TODO: Is this used?
//static uint8_t wakeup_level = (WAKEUP_IOx_LOW << WAKEUP_IO11_SHIFT_MASK);
uint32_t Increment_Time(uint32_t diffMillis, uint32_t printInterval1);

void SPI_Master_Configuration(void);
uint8_t SPI_Read_Memory(uint32_t address);
void SPI_Write_Memory(uint32_t address, uint8_t byte, BOOL incrementAddress);
void SPI_WriteEnable_Memory(void);
void SPI_BulkErase_Memory(void);
void SPI_SectorErase_Memory(uint32_t address);
uint8_t SPI_ReadRegister_Memory(void);
void SPI_DeepPowerDown_Memory(void);
void SPI_DeepPowerRelease_Memory(void);
void SPI_Write_ADC_Register(uint8_t address, uint8_t byte);
uint8_t SPI_Read_ADC_Register(uint8_t address);
uint16_t SPI_Read_ADC_Value(void);
void SPI_SendByte(uint8_t byte);

void ADC_Reset(void);
void Read_ADC(void);

void Open_Valve_Small(uint8_t valve);
void Close_Valve_Small(uint8_t valve);
void Adjust_Valve_State(uint8_t state);
void Open_Valve_Large(void);
void Close_Valve_Large(void);
void Open_Valve_Small_Final(void);
void Close_Valve_Small_Final(void);
void Close_Valve(void);

void Pump_On(void);
void Pump_Off(void);

void Pressurize_Air_Bag(uint8_t bag, uint16_t desiredPressure, float multiplier);
void Depressurize_Air_Bag(uint8_t bag, uint16_t desiredPressure, float multiplier);
void Redistribute_Pressure(uint8_t targetBag, uint8_t otherBag, uint16_t desiredPressure);
void Initialize_AirBags(uint16_t startingPressure);
void Release_AirBags(void);

void LEDs_Set_Blue(BOOL LED_Blue);
void LEDs_Set_Red(BOOL LED_Red);
void LEDs_Flash(BOOL LED_Blue, BOOL LED_Red);

void WriteHeader_Memory(void);
void WritePressures_Memory(void);
uint32_t External_Find_Last_Entry(void);

//void WriteAddress_IntFlash(void);
void WriteAddress_IntFlash(uint32_t currentAddress);
void ReadAddress_IntFlash(void);
BOOL ReadDeviceNum_IntFlash(uint8_t* deviceNum);
void WriteDeviceNum_IntFlash(uint8_t* deviceNum);
//void ReadOffset_IntFlash(void);
//void WriteOffset_IntFlash(void);
void ReadMultipliers_IntFlash(void);
void WriteMultipliers_IntFlash(void);
void WriteLP_IntFlash(uint16_t lower);
uint16_t ReadLP_IntFlash(void);
void WriteUP_IntFlash(uint16_t upper);
uint16_t ReadUP_IntFlash(void);
void WriteTP_IntFlash(uint16_t target);
uint16_t ReadTP_IntFlash(void);
void WriteSP_IntFlash(uint8_t sample);
uint8_t ReadSP_IntFlash(void);
void WriteSP_IntFlash(uint8_t sample);
uint8_t ReadSP_IntFlash(void);
void WriteNS_IntFlash(uint8_t noSamples);
uint8_t ReadNS_IntFlash(void);
void WriteMode_IntFlash(uint8_t mode);
uint8_t ReadMode_IntFlash(void);

void DeInitialize(void);
void ReInitialize(void);

void SPI_Memory_Test(void);
void SPI_ADC_Test(void);
void BLE_DataDownloader_Test(void);

extern volatile BOOL disconnectedMode;
extern volatile BOOL longTermMode;
extern volatile uint16_t CurrentPressures[];
extern volatile uint8_t currentState;
extern volatile uint16_t connection_handle;
extern volatile BOOL isPressureReaderNotificationEnabled;
extern volatile BOOL isDownloadNotificationEnabled;
extern uint32_t currentAddress;
extern volatile uint32_t address;
extern volatile uint32_t BatteryReading;
extern volatile uint8_t sensorOffsets[];
extern volatile uint8_t multiplier_increase[];
extern volatile uint8_t multiplier_decrease[];

extern volatile uint16_t lowerBoundPressure;
extern volatile uint16_t upperBoundPressure;
extern volatile uint16_t targetPressure;
extern volatile uint8_t samplingPeriod;
extern volatile uint8_t numberOfSamples;
extern volatile uint8_t cushionMode;

extern volatile uint8_t year;
extern volatile uint8_t month;
extern volatile uint8_t day;
extern volatile uint8_t hour;
extern volatile uint8_t minute;
extern volatile uint8_t second;
extern volatile uint32_t startTimeTicks;
extern volatile uint32_t stopTimeTicks;
extern volatile int logCounter[];

typedef enum
{
    INCREASE,
    DECREASE
} pressureSTATE;

#endif
