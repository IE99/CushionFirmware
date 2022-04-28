
/*******************************************************************************
* File Name          : main.c
* Author             : Brendan Coutts / Ibrahim Elmallah
* Date               : August-2021
* Description        : Main program for pressure cushion project
*******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ble_const.h"
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "BlueNRG1_adc.h"
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Config.h"
#include "SensorDemo_config.h"
#include "SEGGER_RTT.h"
#include "clock.h"
#include "sleep.h"
#include "sensor.h"
#include "gatt_db.h"
#include "OTA_btl.h"

#include "defines.h"

#define 	device_num_address 			0x1005A800
#define		device_num_page 			0x35 // Device Number value stored in this page
#define 	offset_address 				0x1005B000
#define		offset_page 				0x36 // Pressure sensor ADC value offsets from nominal value stored in this page
#define 	multipliers_address 		0x1005B800
#define		multipliers_page 			0x37 // Time multiplier values stored in this page
// Settings stored in these pages
#define 	lowerPressure_address 		0x1005C000
#define		lowerPressure_page 			0x38
#define 	upperPressure_address 		0x1005C800
#define		upperPressure_page 			0x39
#define 	targetPressure_address 		0x1005D000
#define		targetPressure_page 		0x3A
#define 	samplePeriod_address 		0x1005D800
#define		samplePeriod_page 			0x3B
#define 	numSamples_address 			0x1005E000
#define		numSamples_page 			0x3C
#define 	mode_address 				0x1005F800
#define		mode_page 					0x3D

ADC_InitType sensorADC;
//uint8_t select;
volatile int logCounter[8];
volatile uint16_t CurrentPressures[8];
volatile uint8_t sensorOffsets[8];
volatile uint8_t multiplier_increase[8];
volatile uint8_t multiplier_decrease[8];

// Declarations of defined global variables
volatile BOOL longTermMode; // Long term sleep mode for low battery
volatile BOOL disconnectedMode; // Low power mode for when connection has not been established for a period
volatile uint8_t currentState;
volatile uint16_t connection_handle;
uint32_t currentAddress;
volatile BOOL isPressureReaderNotificationEnabled;
volatile BOOL isDownloadNotificationEnabled;

volatile uint16_t lowerBoundPressure;
volatile uint16_t upperBoundPressure;
volatile uint16_t targetPressure;
volatile uint8_t samplingPeriod;
volatile uint8_t numberOfSamples;
volatile uint8_t cushionMode;

volatile uint8_t year;
volatile uint8_t month;
volatile uint8_t day;
volatile uint8_t hour;
volatile uint8_t minute;
volatile uint8_t second;

volatile uint32_t address;
volatile uint32_t BatteryReading;
volatile uint32_t startTimeTicks;
volatile uint32_t stopTimeTicks;

int main(void) //TODO: Deal with gatt_db.h and sensor.h
{
	init();
	SEGGER_RTT_printf(0,"Initializations have been completed\n");

	//SPI_ADC_Test();
	//SPI_Memory_Test();
	//BLE_DataDownloader_Test();

	//uint32_t result = ADC_Value(ADC_Input_AdcPin2);
	//SEGGER_RTT_printf(0, "ADC BATT Read: %d.\n", result);

	// Checking which internal memory locations are empty
	/*int counter=0;
	for(uint32_t i = FLASH_START; i<= FLASH_END; i+=4){
		uint32_t data =  FLASH_ReadWord(i);
		if (data == 0xFFFFFFFF){
			SEGGER_RTT_printf(0,"i:%x\n",i);
			counter++;
		}
		if (counter == 20){
			break;
		}
	}*/

	while(1)
	{
		/* BLE Stack Tick */
		BTLE_StackTick();

		BlueNRG_Sleep(SLEEPMODE_NOTIMER,0,0);

		if (!longTermMode){
			APP_Tick();
		}

	}
}

/****************** BlueNRG-1 Sleep Management Callback ********************************/
// Note: SLEEPMODE_CPU_HALT is used for disconnectedMode, between air bag readjustments in autonomous mode,
//  or if Bluetooth connection has not been made while not in autonomous mode
//  SLEEPMODE_NOTIMER is used when battery level is low and device is put in very low power mode
SleepModes App_SleepMode_Check(SleepModes sleepMode)
{
	if (longTermMode) return SLEEPMODE_NOTIMER;
	if (disconnectedMode) return SLEEPMODE_CPU_HALT;
	return SLEEPMODE_RUNNING;
}

/***************************************************************************************/

//****************************************************************************
// Function Name  : init
// Description    : Runs all of the initialization code
// Input          : None
// Return         : None
//*****************************************************************************

void init(void) {
	/*Debugger Init*/
	SEGGER_RTT_Init();
	SEGGER_RTT_ConfigUpBuffer(0,NULL,NULL,0,SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

	SEGGER_RTT_printf(0,"The Device has began initialization\n");

	/* System Init */
	SystemInit();

	/* Identify BlueNRG1 platform */
	SdkEvalIdentification();

	/* Configure I/O communication channel */
	SdkEvalComUartInit(UART_BAUDRATE);

	/* SPI initialization */
	SPI_Master_Configuration();
	SEGGER_RTT_printf(0,"Configured as SPI Master\n");

	/* BlueNRG-1 stack init */
	uint8_t ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
	if (ret != BLE_STATUS_SUCCESS) {
		while(1);
	}

	/* SysTick initialization 1ms */
	SysTick_Config(SYST_CLOCK/1000 - 1);

	Clock_Init();

	/* Sensor Device BLE Init */
	ret = Sensor_DeviceInit();
	if (ret != BLE_STATUS_SUCCESS) {
		SEGGER_RTT_printf(0,"Error in BLE init\n");
		while(1);
	}

	/* ADC Initialization */
	Batt_ADC_Configuration();

	/* Start new conversion */
	ADC_Cmd(ENABLE);

	SEGGER_RTT_printf(0,"Initialized internal ADC\n");

	/* Initialize external ADC */
	ADC_Reset();
	SEGGER_RTT_printf(0,"Initialized external ADC\n");

	// GPIO Initialization
	InitGPIO(RED_LED|BLUE_LED|PVControl|ValveControl|Valve1|Valve2|Valve3|Valve4|Valve5|Valve6|Valve7|Valve8|SensorControl);

	GPIO_SetBits(RED_LED|BLUE_LED); // Turn on the LEDs
	GPIO_On(SensorControl); // Turn on the supply voltage for the sensors
	GPIO_Off(PVControl); // Turn off the pump to begin

	Close_Valve(); // Close valves to begin with

	longTermMode = FALSE; // Long term sleep mode for low battery
	disconnectedMode = FALSE; // Low power mode for when connection has not been established for a period
	currentState = 0;
	connection_handle = 0;
	currentAddress = 0;
	isPressureReaderNotificationEnabled = FALSE;
	isDownloadNotificationEnabled = FALSE;

	year = 0;
	month = 1;
	day = 1;
	hour = 1;
	minute = 0;
	second = 0;

	logCounter[0] = 0;
	logCounter[1] = 0;
	logCounter[2] = 0;
	logCounter[3] = 0;
	logCounter[4] = 0;
	logCounter[5] = 0;
	logCounter[6] = 0;
	logCounter[7] = 0;

	sensorOffsets[0] = 56;
	sensorOffsets[1] = 56;
	sensorOffsets[2] = 56;
	sensorOffsets[3] = 56;
	sensorOffsets[4] = 56;
	sensorOffsets[5] = 56;
	sensorOffsets[6] = 56;
	sensorOffsets[7] = 56;

	currentAddress = External_Find_Last_Entry(); // Get currentAddress

	// Read settings
	lowerBoundPressure = ReadLP_IntFlash();
	upperBoundPressure = ReadUP_IntFlash();
	targetPressure = ReadTP_IntFlash();
	samplingPeriod = ReadSP_IntFlash();
	numberOfSamples = ReadNS_IntFlash();
	cushionMode = ReadMode_IntFlash();

	ReadMultipliers_IntFlash(); // Get current multipliers

	SEGGER_RTT_printf(0,"Initialized GPIO's\n");

	// Start timer 3: Start battery level monitoring counter
	HAL_VTimerStart_ms(3, 60000);
	// Start notification timer
	HAL_VTimerStart_ms(0, 1000);

	if (cushionMode == 0) { // In autonomous mode, start readjustment timer
		SEGGER_RTT_printf(0,"In patient mode\n");
		HAL_VTimerStart_ms(2, samplingPeriod*60*1000); // Pressure readjustment counter
		DeInitialize();
	} else 	if (cushionMode == 1) {
		SEGGER_RTT_printf(0,"In admin mode\n");
		// Pressure readjustment counter will be turned on after connection is made in this mode

		//Start timer 1: Start connection monitoring counter in Admin mode
		// In autonomous mode, do not need connection monitoring counter
		//HAL_VTimerStart_ms(1, 60000); //TODO: Turn back on
	}

}

//****************************************************************************
// Function Name  : Batt_ADC_Configuration
// Description    : Configures the ADC to read battery voltage
// Input          : None
// Return         : None
//*****************************************************************************
void Batt_ADC_Configuration(void) {
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_ADC, ENABLE);

	/* Configure ADC */
	sensorADC.ADC_OSR = ADC_OSR_200;
	sensorADC.ADC_Input = ADC_Input_AdcPin2;
	sensorADC.ADC_ConversionMode = ADC_ConversionMode_Single;
	sensorADC.ADC_ReferenceVoltage = ADC_ReferenceVoltage_0V6;
	sensorADC.ADC_Attenuation = ADC_Attenuation_9dB54;

	ADC_Init(&sensorADC);

	/* Enable auto offset correction */
	ADC_AutoOffsetUpdate(ENABLE);
	ADC_Calibration(ENABLE);
}

//****************************************************************************
// Function Name  : ADC_Raw_Value
// Description    : Reads the ADC 8 times and returns an average reading
// Input          : input: Pin to read the voltage value
// Return         : ADC value * 1000
//*****************************************************************************
uint16_t ADC_Value(uint8_t input) {
	uint8_t counter = 0;
	float adc_value = 0;
	uint16_t adc_value_whole;
	while(counter < 16){
		if (ADC->CONF_b.CHSEL == input){
			if(ADC_GetFlagStatus(ADC_FLAG_EOC)) {
				/* Read converted data */
				 //uint16_t raw_data = ADC_GetRawData();
				 float converted_data = ADC_GetConvertedData(ADC_Input_AdcPin2, ADC_ReferenceVoltage_0V6);
				 //SEGGER_RTT_printf(0,"Raw value: 0x%x\n", raw_data);

				 adc_value += converted_data;
				 counter++;
			}
		}
		sensorADC.ADC_Input = input;
		/* Enable auto offset correction */
		ADC_AutoOffsetUpdate(ENABLE);
		ADC_Calibration(ENABLE);
		ADC_SelectInput(input);
		ADC_Cmd(ENABLE);
	}
	adc_value_whole = ((uint16_t)(adc_value*1000))>>4;
	adc_value_whole *= 3; // Real battery voltage passes through voltage divider

	SEGGER_RTT_printf(0,"Battery value: %d\n", adc_value_whole);
	return adc_value_whole;
}

//****************************************************************************
// Function Name  : InitGPIO
// Description    : Initializes a GPIO as output
// Input          : myLED: The pin to initialize
// Return         : None
//*****************************************************************************
void InitGPIO(uint32_t myLED) {
	GPIO_InitType GPIO_InitStructure;
	/* Enable the GPIO Clock */
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

	/* Configure the LED pin */
	GPIO_InitStructure.GPIO_Pin = myLED;
	GPIO_InitStructure.GPIO_Mode = GPIO_Output;
	GPIO_InitStructure.GPIO_Pull = ENABLE;
	GPIO_InitStructure.GPIO_HighPwr = ENABLE;
	GPIO_Init(&GPIO_InitStructure);

	/* Put the LED off */
	GPIO_Off(myLED);
}

//****************************************************************************
// Function Name  : InitGPIOInput
// Description    : Initializes a GPIO as input
// Input          : myLED: The pin to initialize
// Return         : None
//*****************************************************************************
void InitGPIOInput(uint32_t myLED)
{
	GPIO_InitType GPIO_InitStructure;
	/* Enable the GPIO Clock */
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

	/* Configure the LED pin */
	GPIO_InitStructure.GPIO_Pin = myLED;
	GPIO_InitStructure.GPIO_Mode = GPIO_Input;
	GPIO_InitStructure.GPIO_Pull = DISABLE;
	GPIO_InitStructure.GPIO_HighPwr = DISABLE;
	GPIO_Init(&GPIO_InitStructure);

	/* Put the LED off */
	GPIO_Off(myLED);
}

//****************************************************************************
// Function Name  : GPIO_On
// Description    : Turns on a GPIO
// Input          : myLED: The pin to turn on
// Return         : None
//*****************************************************************************
void GPIO_On(uint32_t myLED) {
	GPIO_WriteBit(myLED, Bit_SET);
}

//****************************************************************************
// Function Name  : GPIO_Off
// Description    : Turns off a GPIO
// Input          : myLED: The pin to turn off
// Return         : None
//*****************************************************************************
void GPIO_Off(uint32_t myLED) {
	GPIO_WriteBit(myLED, Bit_RESET);
}

//****************************************************************************
// Function Name  : GPIO_Toggle
// Description    : Toggles a GPIO
// Input          : myLED: The pin to turn on
// Return         : None
//*****************************************************************************
void GPIO_Toggle(uint32_t myLED) {
	GPIO_ToggleBits(myLED);
}

//****************************************************************************
// Function Name  : SPI_Master_Configuration
// Description    : Configures the SPI bus for master communication
// Input          : None
// Return         : None
//*****************************************************************************
void SPI_Master_Configuration(void) {
	SPI_InitType SPI_InitStructure;
	GPIO_InitType GPIO_InitStructure;

	/* Enable SPI and GPIO clocks */
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO | CLOCK_PERIPH_SPI, ENABLE);

	/* Configure SPI pins */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = SDK_EVAL_SPI_PERIPH_OUT_MODE;
	GPIO_InitStructure.GPIO_Pull = ENABLE;
	GPIO_InitStructure.GPIO_HighPwr = DISABLE;
	GPIO_Init(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = SDK_EVAL_SPI_PERIPH_IN_MODE;
	GPIO_Init(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = SDK_EVAL_SPI_PERIPH_SCLK_MODE;
	GPIO_Init(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_CS_Memory;
	GPIO_InitStructure.GPIO_Mode = GPIO_Output;
	GPIO_InitStructure.GPIO_HighPwr = ENABLE;
	GPIO_Init(&GPIO_InitStructure);
	GPIO_SetBits(SPI_CS_Memory);

	GPIO_InitStructure.GPIO_Pin = SPI_CS_ADC;
	GPIO_InitStructure.GPIO_Mode = GPIO_Output;
	GPIO_InitStructure.GPIO_HighPwr = ENABLE;
	GPIO_Init(&GPIO_InitStructure);
	GPIO_SetBits(SPI_CS_ADC);

	/* Configure SPI in master mode */
	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b ;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_BaudRate = SPI_BAUDRATE;
	SPI_Init(&SPI_InitStructure);

	/* Clear RX and TX FIFO */
	SPI_ClearTXFIFO();
	SPI_ClearRXFIFO();

	/* Set null character */
	SPI_SetDummyCharacter(0xFF);

	/* Set communication mode */
	SPI_SetMasterCommunicationMode(SPI_FULL_DUPLEX_MODE);

	/* Enable SPI functionality */
	SPI_Cmd(ENABLE);
}

//****************************************************************************
// Function Name  : SPI_Write_Memory
// Description    : Write to the SPI enabled memory. Must follow write enable command
// Input          : address: the location in memory to write to
//				  : byte: the data to be written
//				  : incrementAddress: whether or not to increment memory address
// Return         : None
//*****************************************************************************
void SPI_Write_Memory(uint32_t address, uint8_t byte, BOOL incrementAddress) {

	GPIO_ResetBits(SPI_CS_Memory);

	SPI_SendByte(0x02); // write command
	SPI_SendByte((uint8_t) (address >> 16));
	SPI_SendByte((uint8_t) (address >> 8));
	SPI_SendByte((uint8_t) (address >> 0));

	SPI_SendByte(byte);

	GPIO_SetBits(SPI_CS_Memory);

	while(SPI_ReadRegister_Memory() & 0x01); // stall until write in progress bit de-asserts

	if (incrementAddress) currentAddress += 1;
}

//****************************************************************************
// Function Name  : SPI_Read_Memory
// Description    : Read to the SPI enabled memory
// Input          : address: the location to read from
// Return         : data
//*****************************************************************************
uint8_t SPI_Read_Memory(uint32_t address) {

	GPIO_ResetBits(SPI_CS_Memory);

	SPI_SendByte(0x03);
	SPI_SendByte((uint8_t) (address >> 16));
	SPI_SendByte((uint8_t) (address >> 8));
	SPI_SendByte((uint8_t) (address >> 0));

	SPI_SendData(0x00);

	while(RESET == SPI_GetFlagStatus(SPI_FLAG_RNE));

	uint8_t data = SPI_ReceiveData();

	while (SET == SPI_GetFlagStatus(SPI_FLAG_BSY));

	GPIO_SetBits(SPI_CS_Memory);

	return data;
}

//****************************************************************************
// Function Name  : SPI_WriteEnable_Memory
// Description    : Enables the SPI memory to be written to
// Input          : None
// Return         : None
//*****************************************************************************
void SPI_WriteEnable_Memory() {
	GPIO_ResetBits(SPI_CS_Memory);

	SPI_SendByte(0x06); // write enable

	GPIO_SetBits(SPI_CS_Memory);
}

//****************************************************************************
// Function Name  : SPI_BulkErase_Memory
// Description    : Erases all of the memory to 1's. Must follow write enable command
// Input          : None
// Return         : None
//*****************************************************************************
void SPI_BulkErase_Memory(){

	GPIO_ResetBits(SPI_CS_Memory);

	SPI_SendByte(0xc7); // bulk erase command

	GPIO_SetBits(SPI_CS_Memory);

	while(SPI_ReadRegister_Memory() & 0x01); // stall until write in progress bit de-asserts

	currentAddress = 0;
}

//****************************************************************************
// Function Name  : SPI_SectorErase_Memory
// Description    : Clear a sector of memory. Must follow write enable command
//                  The memory is organized into 8*64 sectors, 256 pages each, 256 bytes each
// Input          : Any address inside a given sector
// Return         : None
//*****************************************************************************
void SPI_SectorErase_Memory(uint32_t address){

	GPIO_ResetBits(SPI_CS_Memory);

	SPI_SendByte(0xd8); // sector erase command

	SPI_SendByte((uint8_t) (address >> 16));
	SPI_SendByte((uint8_t) (address >> 8));
	SPI_SendByte((uint8_t) (address >> 0));

	GPIO_SetBits(SPI_CS_Memory);

	while(SPI_ReadRegister_Memory() & 0x01); // stall until write in progress bit de-asserts
}

//****************************************************************************
// Function Name  : SPI_ReadRegister_Memory
// Description    : Read status register of flash memory
// Input          : None
// Return         : Register contents
//*****************************************************************************
uint8_t SPI_ReadRegister_Memory(){

	GPIO_ResetBits(SPI_CS_Memory);

	SPI_SendByte(0x05); // Read status register command

	SPI_SendData(0x00);

	while(RESET == SPI_GetFlagStatus(SPI_FLAG_RNE));

	uint8_t data = SPI_ReceiveData();

	while (SET == SPI_GetFlagStatus(SPI_FLAG_BSY));

	GPIO_SetBits(SPI_CS_Memory);

	return data;
}

//****************************************************************************
// Function Name  : SPI_DeepPowerDown_Memory
// Description    : Put memory into deep power down mode
// Input          : None
// Return         : None
//*****************************************************************************
void SPI_DeepPowerDown_Memory(){

	GPIO_ResetBits(SPI_CS_Memory);

	SPI_SendByte(0xb9); // deep power down command

	GPIO_SetBits(SPI_CS_Memory);
}

//****************************************************************************
// Function Name  : SPI_DeepPowerRelease_Memory
// Description    : Release memory from deep power down mode
// Input          : None
// Return         : None
//*****************************************************************************
void SPI_DeepPowerRelease_Memory(){

	GPIO_ResetBits(SPI_CS_Memory);

	SPI_SendByte(0xab); // release from deep power down command

	GPIO_SetBits(SPI_CS_Memory);

	Clock_Wait(1); // Wait for release to complete
}

//****************************************************************************
// Function Name  : SPI_Write_ADC_Register
// Description    : Write to the SPI enabled ADC register space
// Input          : address: the register location to write to
//				  : byte: the data to be written
// Return         : None
//*****************************************************************************
void SPI_Write_ADC_Register(uint8_t address, uint8_t byte) {

	GPIO_ResetBits(SPI_CS_ADC);

	SPI_SendByte(0x08); // write command
	SPI_SendByte(address);
	SPI_SendByte(byte);

	GPIO_SetBits(SPI_CS_ADC);
}

//****************************************************************************
// Function Name  : SPI_Read_ADC_Register
// Description    : Read from the SPI enabled ADC register space
// Input          : address: the location to read from
// Return         : data
//*****************************************************************************
uint8_t SPI_Read_ADC_Register(uint8_t address) {

	GPIO_ResetBits(SPI_CS_ADC);

	SPI_SendByte(0x10); // read command
	SPI_SendByte(address);
	SPI_SendByte(0x00);

	SPI_SendData(0x00);
	// Toggle CSn to trigger read
	GPIO_SetBits(SPI_CS_ADC);
	//Clock_Wait(1);
	GPIO_ResetBits(SPI_CS_ADC);

	while(RESET == SPI_GetFlagStatus(SPI_FLAG_RNE));

	uint8_t data = SPI_ReceiveData();

	while (SET == SPI_GetFlagStatus(SPI_FLAG_BSY));

	GPIO_SetBits(SPI_CS_ADC);

	return data;
}

//****************************************************************************
// Function Name  : SPI_Read_ADC_Value
// Description    : Read from the SPI enabled ADC value
// Input          : None
// Return         : data
//*****************************************************************************
uint16_t SPI_Read_ADC_Value() {

	SPI_DataSizeConfig(SPI_DataSize_12b); // ADC output is 12 bits

	SPI_SendData(0x00);

	GPIO_ResetBits(SPI_CS_ADC);

	while(RESET == SPI_GetFlagStatus(SPI_FLAG_RNE));

	uint16_t data = SPI_ReceiveData();

	while (SET == SPI_GetFlagStatus(SPI_FLAG_BSY));

	GPIO_SetBits(SPI_CS_ADC);

	SPI_DataSizeConfig(SPI_DataSize_8b); // Return data size to 8 bits

	return data;
}

//****************************************************************************
// Function Name  : SPI_SendByte
// Description    : Sends a single byte on the SPI bus
// Input          : byte: the information to be sent
// Return         : None
//*****************************************************************************
void SPI_SendByte(uint8_t byte) {
	while(RESET == SPI_GetFlagStatus(SPI_FLAG_TFE));
	SPI_SendData(byte);
	while(RESET == SPI_GetFlagStatus(SPI_FLAG_RNE));
	SPI_ReceiveData();
	while (SET == SPI_GetFlagStatus(SPI_FLAG_BSY));
}

//****************************************************************************
// Function Name  : LEDs_Set_Blue
// Description    : Turn blue LED on
// Input          : Bool representing LED on/off
// Return         : None
//*****************************************************************************
void LEDs_Set_Blue(BOOL LED_Blue){
	if (LED_Blue == TRUE){
		GPIO_On(BLUE_LED);
	} else {
		GPIO_Off(BLUE_LED);
	}
}

//****************************************************************************
// Function Name  : LEDs_Set_Red
// Description    : Turn red LED on
// Input          : Bool representing LED on/off
// Return         : None
//*****************************************************************************
void LEDs_Set_Red(BOOL LED_Red){
	if (LED_Red == TRUE){
		GPIO_On(RED_LED);
	} else {
		GPIO_Off(RED_LED);
	}
}

//****************************************************************************
// Function Name  : LEDs_Flash
// Description    : Toggle blue or red LED's
// Input          : Bool representing which LED to toggle
// Return         : None
//*****************************************************************************
void LEDs_Flash(BOOL LED_Blue, BOOL LED_Red){

	if (LED_Blue == TRUE && LED_Red == TRUE){
		GPIO_On(BLUE_LED);
		GPIO_On(RED_LED);
		Clock_Wait(1000);
		GPIO_Off(BLUE_LED);
		GPIO_Off(RED_LED);
		Clock_Wait(1000);
	} else if (LED_Red == TRUE){
		GPIO_On(RED_LED);
		Clock_Wait(1000);
		GPIO_Off(RED_LED);
		Clock_Wait(1000);
	} else if (LED_Blue == TRUE){
		GPIO_On(BLUE_LED);
		Clock_Wait(1000);
		GPIO_Off(BLUE_LED);
		Clock_Wait(1000);
	} else {
		GPIO_Off(BLUE_LED);
		GPIO_Off(RED_LED);
	}
}

//****************************************************************************
// Function Name  : ADC_Reset
// Description    : Resets and configures external ADC
// Input          : None
// Return         : None
//*****************************************************************************
void ADC_Reset(){

	SPI_Write_ADC_Register(0x1, 0x1); // Set GENERAL_CFG.RST (1st bit) to 1; Reset ADC
    while(!(SPI_Read_ADC_Register(0x0) & 0x01)); // Read SYSTEM_STATUS.BOR (1st bit); Read until brown out reset indicator is set
    SPI_Write_ADC_Register(0x0, 0x1); // Reset SYSTEM_STATUS.BOR
    SPI_Write_ADC_Register(0x1, 0x4); // Set GENERAL_CFG.CH_RST (3rd bit) to 1; Force all channels to analog inputs
    SPI_Write_ADC_Register(0x12, 0xFF); // Set AUTO_SEQ_CH_SEL to FF; Configure all input channels for auto-sequence mode
}

//****************************************************************************
// Function Name  : Read_ADC
// Description    : Reads all 8 pressure sensors
// Input          : None
// Return         : None
//*****************************************************************************
void Read_ADC() {
	int offset;

	// Set SEQUENCE_CFG.SEQ_MODE to 1 - auto sequence mode
	// and set SEQUENCE_CFG.SEQ_START to 1 - start channel sequencing in ascending order
	SPI_Write_ADC_Register(0x10, 0b00010001);
	uint16_t output = SPI_Read_ADC_Value(); // Need to read it out once first for some reason. Perhaps to clear an internal buffer in ADC?

	// Read 8 times and average result
	for (int i = 0; i < 8;  i++) {
		// Read 8 ADC channels to get sensor readings
		for (int j = 0; j < 8; j++) {
			if (i == 0) CurrentPressures[j] = 0; // Reset value

			output = SPI_Read_ADC_Value();
			Clock_Wait(1); // Wait for conversion

			//CurrentPressures[j] += output;
			if (output < 462) output = 462;
			CurrentPressures[j] += (0.080626*output) - 37.191; // Experimentally determined equation to convert ADC value to mmHg
			//SEGGER_RTT_printf(0, "ADC channel %d, round %d - value: %d\n", j, i, output);

			// Take sensor's offset from equation into account
			offset = 456 - (sensorOffsets[j] + 400);
			CurrentPressures[j] += offset;

			if (i == 7) CurrentPressures[j] = CurrentPressures[j] >> 3; // Average value
		}
	}

	SPI_Write_ADC_Register(0x10, 0x00); // Stop channel sequencing
}

//****************************************************************************
// Function Name  : Open_Valve_Small
// Description    : Opens one of the 8 small valves
// Input          : valve: the one to be open
// Return         : None
//*****************************************************************************
void Open_Valve_Small(uint8_t valve) {
	switch (valve) {
		case 1: GPIO_On(Valve1); currentState |= 1; break;
		case 2: GPIO_On(Valve2); currentState |= 2; break;
		case 3: GPIO_On(Valve3); currentState |= 4; break;
		case 4: GPIO_On(Valve4); currentState |= 8; break;
		case 5: GPIO_On(Valve5); currentState |= 16; break;
		case 6: GPIO_On(Valve6); currentState |= 32; break;
		case 7: GPIO_On(Valve7); currentState |= 64; break;
		case 8: GPIO_On(Valve8); currentState |= 128; break;
	}
}

//****************************************************************************
// Function Name  : Close_Valve_Small
// Description    : Closes one of the 8 small valves
// Input          : valve: the one to be closed
// Return         : None
//*****************************************************************************
void Close_Valve_Small(uint8_t valve) {
	switch (valve) {
		case 1: GPIO_Off(Valve1); currentState &= (~1); break;
		case 2: GPIO_Off(Valve2); currentState &= (~2); break;
		case 3: GPIO_Off(Valve3); currentState &= (~4); break;
		case 4: GPIO_Off(Valve4); currentState &= (~8); break;
		case 5: GPIO_Off(Valve5); currentState &= (~16); break;
		case 6: GPIO_Off(Valve6); currentState &= (~32); break;
		case 7: GPIO_Off(Valve7); currentState &= (~64); break;
		case 8: GPIO_Off(Valve8); currentState &= (~128); break;
	}
}

//****************************************************************************
// Function Name  : Adjust_Valve_State
// Description    : Change the small valve states based on the toggled bits passed in
// Input          : state: byte, with each bit representing state of valve
// Return         : None
//*****************************************************************************
void Adjust_Valve_State(uint8_t state){
	if (state & 1) GPIO_On(Valve1); else GPIO_Off(Valve1);
	if (state & 2) GPIO_On(Valve2); else GPIO_Off(Valve2);
	if (state & 4) GPIO_On(Valve3); else GPIO_Off(Valve3);
	if (state & 8) GPIO_On(Valve4); else GPIO_Off(Valve4);
	if (state & 16) GPIO_On(Valve5); else GPIO_Off(Valve5);
	if (state & 32) GPIO_On(Valve6); else GPIO_Off(Valve6);
	if (state & 64) GPIO_On(Valve7); else GPIO_Off(Valve7);
	if (state & 128) GPIO_On(Valve8); else GPIO_Off(Valve8);
}

//****************************************************************************
// Function Name  : Open_Valve_Large
// Description    : Opens the large valve controlling air to smaller valves
// Input          : None
// Return         : None
//*****************************************************************************
void Open_Valve_Large() {
	GPIO_On(ValveControl);
}

//****************************************************************************
// Function Name  : Close_Valve_Large
// Description    : Closes the large valve controlling air to smaller valves
// Input          : None
// Return         : None
//*****************************************************************************
void Close_Valve_Large() {
	GPIO_Off(ValveControl);
}

//****************************************************************************
// Function Name  : Open_Valve_Small_Final
// Description    : Opens the small valve controlling air to outside
// Input          : None
// Return         : None
//*****************************************************************************
void Open_Valve_Small_Final(){
	GPIO_On(ValveFinal);
}

//****************************************************************************
// Function Name  : Close_Valve_Small_Final
// Description    : Closes the small valve controlling air to outside
// Input          : None
// Return         : None
//*****************************************************************************
void Close_Valve_Small_Final() {
	GPIO_Off(ValveFinal);
}

//****************************************************************************
// Function Name  : Close_Valve
// Description    : Closes all valves
// Input          : None
// Return         : None
//*****************************************************************************
void Close_Valve() {
	GPIO_Off(Valve1);
	GPIO_Off(Valve2);
	GPIO_Off(Valve3);
	GPIO_Off(Valve4);
	GPIO_Off(Valve5);
	GPIO_Off(Valve6);
	GPIO_Off(Valve7);
	GPIO_Off(Valve8);

	GPIO_Off(ValveControl);

	currentState = 0;
}

//****************************************************************************
// Function Name  : Pump_On
// Description    : Turns on air pump
// Input          : None
// Return         : None
//*****************************************************************************
void Pump_On() {
	GPIO_On(PVControl);
}

//****************************************************************************
// Function Name  : Pump_Off
// Description    : Turns off air pump
// Input          : None
// Return         : None
//*****************************************************************************
void Pump_Off() {
	GPIO_Off(PVControl);
}

//****************************************************************************
// Function Name  : Pressurize_Air_Bag
// Description    : Pressurize one of the air bags to a desired pressure using external air
// Input          : bag to be pressurized, desired mmHg to pressurize bag to, and a time multiplier
// Return         : None
//*****************************************************************************
void Pressurize_Air_Bag(uint8_t bag, uint16_t desiredPressure, float multiplier) {
	SEGGER_RTT_printf(0, "Pressurizing bag %d to %d mmHg, with multiplier %d\n", bag, desiredPressure, (uint16_t)(multiplier*100));

	Close_Valve(); // Start with all valves closed

	/*uint32_t startTime = HAL_VTimerGetCurrentTime_sysT32();
	uint32_t endTime = HAL_VTimerGetCurrentTime_sysT32();
	uint32_t timeDifference;*/

	// Find time to hold valves open and pump
	Read_ADC();
	uint16_t initialPressure = CurrentPressures[bag-1]; //Starts from 0
	if (initialPressure >= desiredPressure) return;
	float holdTime1 = 0.0001*pow((float)initialPressure, 3) - 0.0168*pow((float)initialPressure, 2) + 1.2474*initialPressure + 1.7436;  // Experimentally determined equation
	float holdTime2 = 0.0001*pow((float)desiredPressure, 3) - 0.0168*pow((float)desiredPressure, 2) + 1.2474*desiredPressure + 1.7436;
	float holdTime = (holdTime2 - holdTime1)*multiplier;
	uint32_t holdTime_ms = (uint32_t)(holdTime*1000);
	SEGGER_RTT_printf(0, "Pumping for %d ms\n", holdTime_ms);

	// Pressurize for correct time period
	Open_Valve_Small(bag);
	Open_Valve_Large();
	Pump_On();

	Clock_Wait(holdTime_ms);

		//endTime = HAL_VTimerGetCurrentTime_sysT32();
		//timeDifference = abs(HAL_VTimerDiff_ms_sysT32(endTime,startTime));
	//} while (timeDifference < holdTime_ms);

	Pump_Off();
	Close_Valve_Large();
	Close_Valve_Small(bag);

}

//****************************************************************************
// Function Name  : Depressurize_Air_Bag
// Description    : Depressurize one of the air bags to a desired pressure by letting air out
// Input          : bag to be depressurized, desired mmHg to depressurize bag to, and a time multiplier
// Return         : None
//*****************************************************************************
void Depressurize_Air_Bag(uint8_t bag, uint16_t desiredPressure, float multiplier) {
	SEGGER_RTT_printf(0, "Depressurizing bag %d to %d mmHg, with multiplier %d\n", bag, desiredPressure, (uint16_t)(multiplier*100));

	Close_Valve(); // Start with all valves closed

	// Find time to hold valves open
	Read_ADC();
	uint16_t initialPressure = CurrentPressures[bag-1]; // Starts from 0
	if (initialPressure <= desiredPressure) return;
	float holdTime1 = -1.881*log((float)initialPressure) + 9.6359;  // Experimentally determined equation
	float holdTime2 = -1.881*log((float)desiredPressure) + 9.6359;  // Experimentally determined equation
	float holdTime = (holdTime2 - holdTime1)*multiplier;
	uint32_t holdTime_ms = (uint32_t)(holdTime*1000);
	SEGGER_RTT_printf(0, "Releasing for %d ms\n", holdTime_ms);

	// Depressurize for correct time period
	Open_Valve_Small(bag);
	Open_Valve_Large();
	Open_Valve_Small_Final();

	Clock_Wait(holdTime_ms);

	Close_Valve_Small(bag);
	Close_Valve_Large();
	Close_Valve_Small_Final();
}

//****************************************************************************
// Function Name  : Redistribute_Pressure
// Description    : Redistribute pressure by allowing air to move from a high pressure bag to a low pressure bag or vice versa
// Input          : bags to redistribute air from, desired mmHg to pressurize/depressurize target bag to
// Return         : None
//*****************************************************************************
void Redistribute_Pressure(uint8_t targetBag, uint8_t otherBag, uint16_t desiredPressure) {
	SEGGER_RTT_printf(0, "Redistributing pressure from bag %d to target bag %d, with target pressure %d mmHg\n",
			otherBag, targetBag, desiredPressure);

	uint16_t smallestDifferential;
	uint16_t smallestDifferentialIndex;
	uint8_t numCharacteristics = 7;
	uint8_t usedCharacteristic[10];

	// Experimentally determined changes in pressure for high and low pressure bags
	//  when air is exchanged between them, for various pressure differentials
	// Each element is pressure change after valves have been open for 0.25 seconds (starting from 0s)
	uint8_t measuredPressureDifferentials[] = {100, 70, 50, 30, 20, 10, 5};
	uint8_t relationships[][10] = {{0, 11, 20, 28, 34, 40, 44, 46, 48, 50},
			{0, 9, 16, 22, 26, 30, 32, 34, 35, 35},
			{0, 7, 13, 17, 20, 23, 24, 25, 25, 25},
			{0, 5, 9, 12, 14, 15, 15, 15, 15, 15},
			{0, 4, 7, 9, 10, 10, 10, 10, 10, 10},
			{0, 3, 4, 5, 5, 5, 5, 5, 5, 5},
			{0, 2, 3, 3, 3, 3, 3, 3, 3, 3}};

	Read_ADC();
	uint16_t pressureDifferential = abs(CurrentPressures[targetBag-1] - CurrentPressures[otherBag-1]);
	uint16_t desiredPressureDifferential = abs(CurrentPressures[targetBag-1] - desiredPressure);

	// Find closest characteristic to use
	uint16_t pressureCharDifference[] = {0, 0, 0, 0, 0, 0, 0};
	for (int i=0; i<numCharacteristics; i++) pressureCharDifference[i] = abs(measuredPressureDifferentials[i] - pressureDifferential);
	smallestDifferential = pressureCharDifference[0];
	smallestDifferentialIndex = 0;
	for (int i=0; i<numCharacteristics; i++) {
		if (pressureCharDifference[i] < smallestDifferential) {
			smallestDifferential = pressureCharDifference[i];
			smallestDifferentialIndex = i;
		}
	}
	SEGGER_RTT_printf(0, "Using characteristic %d mmHg; ", measuredPressureDifferentials[smallestDifferentialIndex]);
	for (int i=0; i<10; i++){ // Copying array
		usedCharacteristic[i] = relationships[smallestDifferentialIndex][i];
	}

	// Find desired time to keep valves open
	uint16_t pressureDifference[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	for (int i=0; i<10; i++) pressureDifference[i] = abs(usedCharacteristic[i] - desiredPressureDifferential);
	smallestDifferential = pressureDifference[0];
	smallestDifferentialIndex = 0;
	for (int i=0; i<10; i++) {
		if (pressureDifference[i] < smallestDifferential) {
			smallestDifferential = pressureDifference[i];
			smallestDifferentialIndex = i;
		}
	}
	uint32_t holdTime = smallestDifferentialIndex*1000*0.25; // Time to keep valves open in ms

	SEGGER_RTT_printf(0, "Hold time: %d ms; Aiming for %d mmHg change in pressure\n",
			holdTime, usedCharacteristic[smallestDifferentialIndex]);

	// Peripheral Control
	Close_Valve(); // Start with all valves closed

	// Exchange air for correct time period
	if (holdTime != 0) {
		Open_Valve_Small(targetBag);
		Open_Valve_Small(otherBag);

		Clock_Wait(holdTime);

		//Close valves
		Close_Valve_Small(targetBag);
		Close_Valve_Small(otherBag);
	}
}

//****************************************************************************
// Function Name  : Initialize_AirBags
// Description    : Pump up all air bags to a starting pressure
// Input          : Starting pressure in mmHg, +/- 5 mmHg
// Return         : None
//*****************************************************************************
void Initialize_AirBags(uint16_t startingPressure) {
	SEGGER_RTT_printf(0, "Initializing all air bags to %d mmHg\n", startingPressure);

	uint8_t currentBag;
	uint8_t bagOrder [8] = {2, 3, 5, 7, 8, 1, 4, 6}; // Order to pump bags in // TODO: Change
	uint8_t bagReady[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Whether bag is pumped fully or not
	pressureSTATE state_pressure[8];
	uint8_t slack = 5;
	uint8_t numBags = 5; // TODO: Change

	Close_Valve(); // Start with all valves closed

	// Bring up all air bags to pressures with range of computed equation
	Read_ADC();
	for (int i=0; i<numBags; i++) {
		currentBag = bagOrder[i];
		if (CurrentPressures[currentBag-1] <= 0) {
			// Pump for 115 seconds, which will bring the bags within an appropriate range given they are initially empty
			SEGGER_RTT_printf(0, "Pumping bag %d for 115 seconds\n", currentBag);

			Open_Valve_Small(currentBag);
			Open_Valve_Large();
			Pump_On();

			Clock_Wait(115*1000);

			Pump_Off();
			Close_Valve_Large();
			Close_Valve_Small(currentBag);
		}
	}


	// Adjust to correct pressure
	Read_ADC();
	for (uint8_t j=0; j<4; j++) {
		SEGGER_RTT_printf(0, "Readjusting pressure: Iteration %d\n", j);

		for (uint8_t i=0; i<numBags; i++) {
			currentBag = bagOrder[i];
			if (CurrentPressures[currentBag-1] < (startingPressure - slack)) { // If a bag's pressure is too low
				Pressurize_Air_Bag(currentBag, startingPressure, ((float)multiplier_increase[currentBag-1])/100);
				state_pressure[currentBag-1] = INCREASE;
			} else if (CurrentPressures[currentBag-1] > (startingPressure + slack)) { // If a bag's pressure is too high
				Depressurize_Air_Bag(currentBag, startingPressure, ((float)multiplier_decrease[currentBag-1])/100);
				state_pressure[currentBag-1] = DECREASE;
			}
		}

		Read_ADC();

		// Adjust multipliers
		SEGGER_RTT_printf(0, "Current pressures: ");
		for (uint8_t i=0; i<numBags; i++) {
			currentBag = bagOrder[i];
			if (CurrentPressures[currentBag-1] < (startingPressure - slack)) {
				if (state_pressure[currentBag-1] == INCREASE && multiplier_increase[currentBag-1] < 250) multiplier_increase[currentBag-1] += 10; // Increased pressure and still too low -> Raise multiplier
				if (state_pressure[currentBag-1] == DECREASE && multiplier_decrease[currentBag-1] > 10) multiplier_decrease[currentBag-1] -= 10; // Decreased pressure and now its too low -> Lower multiplier
			} else if (CurrentPressures[currentBag-1] > (startingPressure + slack)) {
				if (state_pressure[currentBag-1] == INCREASE && multiplier_increase[currentBag-1] > 10) multiplier_increase[currentBag-1] -= 10; // Increased pressure and now its too high -> Lower multiplier
				if (state_pressure[currentBag-1] == DECREASE && multiplier_decrease[currentBag-1] < 250) multiplier_decrease[currentBag-1] += 10; // Decreased pressure and still too high -> Raise multiplier
			} else {
				bagReady[currentBag-1] = 1;
			}
			SEGGER_RTT_printf(0, "%d mmHg, ", CurrentPressures[currentBag-1]);
		} SEGGER_RTT_printf(0, "\n");

		BOOL readjustmentDone = FALSE;
		for (uint8_t i=0; i<numBags; i++){
			currentBag = bagOrder[i];
			if (bagReady[currentBag-1] == 0){
				readjustmentDone = FALSE;
				break;
			} else {
				readjustmentDone = TRUE;
			}
		}

		if (readjustmentDone == TRUE) {
			break;
		}
	}
	SEGGER_RTT_printf(0, "Pressure changes are done\n");

	WriteMultipliers_IntFlash(); // Save new multipliers
}

//****************************************************************************
// Function Name  : Release_AirBags
// Description    : Release air from all airbags
// Input          : None
// Return         : None
//*****************************************************************************
void Release_AirBags() {
	SEGGER_RTT_printf(0, "Releasing air from all bags\n");

	uint8_t currentBag;
	uint8_t bagOrder [8] = {2, 3, 5, 7, 8, 1, 4, 6}; // Order to release bags in // TODO: Change
	uint8_t numBags = 5; // TODO: Change

	for (int i=0; i<numBags; i++) {
		currentBag = bagOrder[i];
		SEGGER_RTT_printf(0, "Releasing bag %d\n", currentBag);
		// Depressurize for correct time period

		Open_Valve_Small(currentBag);
		Open_Valve_Large();
		Open_Valve_Small_Final();

		Clock_Wait(20*1000); // 20 seconds is enough to depressurize from any pressure

		Close_Valve_Small(currentBag);
		Close_Valve_Large();
		Close_Valve_Small_Final();
	}
}

//****************************************************************************
// Function Name  : WriteHeader_Memory
// Description    : Write date & time header to memory
// Input          : None
// Return         : None
//*****************************************************************************
void WriteHeader_Memory(){
	// Get current time
	stopTimeTicks = HAL_VTimerGetCurrentTime_sysT32();
	uint32_t diffMillis = abs(HAL_VTimerDiff_ms_sysT32(stopTimeTicks,startTimeTicks));
	uint8_t maxDays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
	if ((year % 4) == 0){
		maxDays[1] = 29;
	}

	// Need to consider overflow since real time recorder is only 8 bits number
	minute += diffMillis / (60*1000);
	diffMillis %= 60*1000;

	//Now diffTicks is in seconds level
	second += diffMillis / 1000;

	//Time incrementation start
	if (second >= 60){
		minute += second/60;
		second %= 60;
	}

	if (minute >= 60){
		hour += minute/60;
		minute %= 60;
	}

	if (hour >= 24){
		day += hour/24;
		hour %= 24;
	}

	if (month == 0){
		month = 1;
	}

	if (day > maxDays[month-1]){
		month += 1;
		day = 1;
	}

	if (month>12){
		year += 1;
		month = 1;
	}

	// Header starts with ABCDEF
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, 0xAB, TRUE);
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, 0xCD, TRUE);
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, 0xEF, TRUE);

	// Write date & time data
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, year, TRUE);
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, month, TRUE);
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, day, TRUE);
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, hour, TRUE);
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, minute, TRUE);

	// Reserve byte to make header length 9 bytes
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, 0x00, TRUE);

	// Will now measure time from this new header
	startTimeTicks = HAL_VTimerGetCurrentTime_sysT32();

	SEGGER_RTT_printf(0, "Header wrote: %d/%d/%d %d:%d; ending at memory location %d\n", day, month, year+2000, hour, minute, currentAddress);
}

//****************************************************************************
// Function Name  : WritePressures_Memory
// Description    : Write pressure data to memory
// Input          : None
// Return         : None
//*****************************************************************************
void WritePressures_Memory(){
	// Length of header is minutes (2 bytes) + pressure values (16 bytes)

	// Calculate minutes from header time
	uint32_t minutes_pressureWrite;
	stopTimeTicks = HAL_VTimerGetCurrentTime_sysT32();
	uint32_t diffMillis = abs(HAL_VTimerDiff_ms_sysT32(stopTimeTicks,startTimeTicks));
	minutes_pressureWrite = diffMillis / (60*1000);

	// Write difference in minutes from header time
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, (uint8_t)(minutes_pressureWrite>>8), TRUE);
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, (uint8_t)minutes_pressureWrite, TRUE);

	for (int i=0; i<8; i++){
		// Write sensor values to memory
		SPI_WriteEnable_Memory();
		SPI_Write_Memory(currentAddress, CurrentPressures[i]>>8, TRUE);
		SPI_WriteEnable_Memory();
		SPI_Write_Memory(currentAddress, CurrentPressures[i], TRUE);
	}

	SEGGER_RTT_printf(0, "Wrote pressure values to memory, location ending at: %d\n", currentAddress);
}

//****************************************************************************
// Function Name  : External_Find_Last_Entry
// Description    : Use binary search to find last unwritten address
//					Memory section must be written in a way that there is no jumps
// Input          : None
// Return         : First 0xFF address in memory range
//*****************************************************************************
uint32_t External_Find_Last_Entry(){
	uint32_t low = 0;
	uint32_t high = 0x3fffff;
	uint8_t value = 0;
	while (high>low){
		uint32_t middle = low + (high - low)/2;
		value = SPI_Read_Memory(middle);
		if (value == 0xFF) {
			high = middle;
		}
		else {
			low = middle+1;
		}
	}
	return low;
}

//****************************************************************************
// Function Name  : ReadDeviceNum_IntFlash
// Description    : Read value of device id from memory
// Input          : Array to put in device ID
// Return         : If read was successful
//*****************************************************************************
BOOL ReadDeviceNum_IntFlash(uint8_t* deviceNum){
	// Read from flash to get new device identifier
	deviceNum[0] = FLASH_ReadByte(device_num_address+3);
	deviceNum[1] = FLASH_ReadByte(device_num_address+2);
	deviceNum[2] = FLASH_ReadByte(device_num_address+1);

	if ( (0x30<=deviceNum[0]) && (deviceNum[0]<=0x39) && (0x30<=deviceNum[1]) && (deviceNum[1]<=0x39)
			&& (0x30<=deviceNum[2]) && (deviceNum[2]<=0x39) ){
		return TRUE;
	}
	else {
		return FALSE;
	}
}

//****************************************************************************
// Function Name  : WriteDeviceNum_IntFlash
// Description    : Write value of device id to memory
// Input          : Array to put in device ID
// Return         : None
//*****************************************************************************
void WriteDeviceNum_IntFlash(uint8_t* deviceNum) {
	uint8_t num2 = FLASH_ReadByte(device_num_address+1);
	uint8_t num1 = FLASH_ReadByte(device_num_address+2);
	uint8_t num0 = FLASH_ReadByte(device_num_address+3);

	// If memory has not been written to
	if ((num2 == 0xFF) && (num1 == 0xFF) && (num0 == 0xFF)){
		uint32_t identifier = 0;
		identifier += ('0'+deviceNum[2])<<8;
		identifier += ('0'+deviceNum[1])<<16;
		identifier += ('0'+deviceNum[0])<<24;
		FLASH_ProgramWord(device_num_address, identifier);
	}

	// Some info is changed, should erase and rewrite
	else if ((num0 != deviceNum[0]) || (num1 != deviceNum[1]) || (num2 != deviceNum[2])){
		//Erase
		FLASH_ErasePage(device_num_page);

		//Rewrite
		uint32_t identifier = 0;
		identifier += ('0'+deviceNum[2])<<8;
		identifier += ('0'+deviceNum[1])<<16;
		identifier += ('0'+deviceNum[0])<<24;
		FLASH_ProgramWord(device_num_address, identifier);
	}
}

//****************************************************************************
// Function Name  : ReadOffset_IntFlash
// Description    : Read offset of each pressure sensor from experimental equation
// Input          : None
// Return         : None
//*****************************************************************************
/*void ReadOffset_IntFlash() { // Currently not used
	// Read sensor offsets from memory
	sensorOffsets[0] = FLASH_ReadByte(offset_address+7);
	sensorOffsets[1] = FLASH_ReadByte(offset_address+6);
	sensorOffsets[2] = FLASH_ReadByte(offset_address+5);
	sensorOffsets[3] = FLASH_ReadByte(offset_address+4);
	sensorOffsets[4] = FLASH_ReadByte(offset_address+3);
	sensorOffsets[5] = FLASH_ReadByte(offset_address+2);
	sensorOffsets[6] = FLASH_ReadByte(offset_address+1);
	sensorOffsets[7] = FLASH_ReadByte(offset_address);

	for (uint8_t i=0; i<8; i++) {
		if (sensorOffsets[i] == 0xFF) {
			sensorOffsets[i] = 56;
		}
	}
}*/

//****************************************************************************
// Function Name  : WriteOffsetIntFlash
// Description    : Find offset of each pressure sensor from experimental equation, and write to memory
// Input          : None
// Return         : None
//*****************************************************************************
/*void WriteOffset_IntFlash() { // Currently not used
	// Read from ADC raw values
	SPI_Write_ADC_Register(0x10, 0b00010001);
	uint16_t output = SPI_Read_ADC_Value();
	for (int i = 0; i < 8;  i++) {
		for (int j = 0; j < 8; j++) {
			if (i == 0) sensorOffsets[j] = 0; // Reset value

			output = SPI_Read_ADC_Value();
			Clock_Wait(1); // Wait for conversion

			sensorOffsets[i] += output - 400; // Subtract 400 so no overflow

			if (i == 7) sensorOffsets[j] = sensorOffsets[j] >> 3; // Average value
		}
	}

	SPI_Write_ADC_Register(0x10, 0x00); // Stop channel sequencing

	// Write to flash
	uint8_t num7 = FLASH_ReadByte(offset_address);
	uint8_t num6 = FLASH_ReadByte(offset_address+1);
	uint8_t num5 = FLASH_ReadByte(offset_address+2);
	uint8_t num4 = FLASH_ReadByte(offset_address+3);
	uint8_t num3 = FLASH_ReadByte(offset_address+4);
	uint8_t num2 = FLASH_ReadByte(offset_address+5);
	uint8_t num1 = FLASH_ReadByte(offset_address+6);
	uint8_t num0 = FLASH_ReadByte(offset_address+7);

	if ((num7 == 0xFF) && (num6 == 0xFF) && (num5 == 0xFF)
			&& (num4 == 0xFF) && (num3 == 0xFF) && (num2 == 0xFF)
			&& (num1 == 0xFF) && (num0 == 0xFF)){
		uint32_t offset = 0;
		offset += (sensorOffsets[0]);
		offset += (sensorOffsets[1])<<8;
		offset += (sensorOffsets[2])<<16;
		offset += (sensorOffsets[3])<<24;
		FLASH_ProgramWord(offset_address, offset);
		offset = 0;
		offset += (sensorOffsets[4]);
		offset += (sensorOffsets[5])<<8;
		offset += (sensorOffsets[6])<<16;
		offset += (sensorOffsets[7])<<24;
		FLASH_ProgramWord(offset_address+4, offset);
	}
	//some info is changed, should erase and rewrite
	else if ((num7 != sensorOffsets[0]) || (num6 != sensorOffsets[1]) || (num5 != sensorOffsets[2])
			|| (num4 != sensorOffsets[3]) || (num3 != sensorOffsets[4]) || (num2 != sensorOffsets[5])
			|| (num1 != sensorOffsets[6]) || (num0 != sensorOffsets[7])){
		//Erase
		FLASH_ErasePage(offset_page);

		//Rewrite
		uint32_t offset = 0;
		offset += (sensorOffsets[0]);
		offset += (sensorOffsets[1])<<8;
		offset += (sensorOffsets[2])<<16;
		offset += (sensorOffsets[3])<<24;
		FLASH_ProgramWord(offset_address, offset);
		offset = 0;
		offset += (sensorOffsets[4]);
		offset += (sensorOffsets[5])<<8;
		offset += (sensorOffsets[6])<<16;
		offset += (sensorOffsets[7])<<24;
		FLASH_ProgramWord(offset_address+4, offset);
	}
}*/

//****************************************************************************
// Function Name  : ReadLP_IntFlash
// Description    : Read lower bound pressure from internal memory
// Input          : None
// Return         : lower bound pressure
//*****************************************************************************
uint16_t ReadLP_IntFlash(){
	// Read from flash to get new data
	uint16_t lp = (uint16_t)FLASH_ReadWord(lowerPressure_address);

	// If uninitialized
	if (lp == 0xFFFF || lp < 0 || lp > 300) {
		lp = 12; // 12 mmHg
	}

	SEGGER_RTT_printf(0, "Read lower bound pressure. lowerBoundPressure = %d; \n", lp);

	return lp;
}

//****************************************************************************
// Function Name  : ReadUP_IntFlash
// Description    : Read upper bound pressure from internal memory
// Input          : None
// Return         : upper bound pressure
//*****************************************************************************
uint16_t ReadUP_IntFlash(){
	// Read from flash to get new data
	uint16_t up = (uint16_t)FLASH_ReadWord(upperPressure_address);

	// If uninitialized
	if (up == 0xFFFF || up < 0 || up > 300) {
		up = 52; // 52 mmHg
	}

	SEGGER_RTT_printf(0, "Read upper bound pressure. upperBoundPressure = %d; \n", up);

	return up;
}

//****************************************************************************
// Function Name  : ReadTP_IntFlash
// Description    : Read target pressure from internal memory
// Input          : None
// Return         : target pressure
//*****************************************************************************
uint16_t ReadTP_IntFlash(){
	// Read from flash to get new data
	uint16_t tp = (uint16_t)FLASH_ReadWord(targetPressure_address);

	// If uninitialized
	if (tp == 0xFFFF || tp < 0 || tp > 300) { //TODO: consider cases such as upperBound < lowerBound
		tp = 32; // 32 mmHg
	}

	SEGGER_RTT_printf(0, "Read target pressure. targetPressure = %d; \n", tp);

	return tp;
}

//****************************************************************************
// Function Name  : ReadSP_IntFlash
// Description    : Read sampling period from internal memory
// Input          : None
// Return         : sampling period
//*****************************************************************************
uint8_t ReadSP_IntFlash(){
	// Read from flash to get new data
	uint8_t sp = (uint8_t)FLASH_ReadWord(samplePeriod_address);

	// If uninitialized
	if (sp == 0xFF || sp <= 0) {
		sp = 5; // 5 minutes
	}

	SEGGER_RTT_printf(0, "Read sampling period. Sampling period = %d; \n", sp);

	return sp;
}

//****************************************************************************
// Function Name  : ReadNS_IntFlash
// Description    : Read no. of samples from internal memory
// Input          : None
// Return         : no. of samples
//*****************************************************************************
uint8_t ReadNS_IntFlash(){
	// Read from flash to get new data
	uint8_t ns = (uint8_t)FLASH_ReadWord(numSamples_address);

	// If uninitialized
	if (ns == 0xFF || ns <= 0) {
		ns = 4; // 5 minutes
	}

	SEGGER_RTT_printf(0, "Read no. of samples. Number of samples = %d; \n", ns);

	return ns;
}

//****************************************************************************
// Function Name  : ReadMode_IntFlash
// Description    : Read no. of samples from internal memory
// Input          : None
// Return         : no. of samples
//*****************************************************************************
uint8_t ReadMode_IntFlash(){
	// Read from flash to get new data
	uint8_t mode = (uint8_t)FLASH_ReadWord(mode_address);

	// If uninitialized
	if ((mode != 0x00) && (mode != 0x01)) {
		mode = 1; // Advanced mode
	}

	SEGGER_RTT_printf(0, "Read cushion mode. Mode = %d; \n", mode);

	return mode;
}

//****************************************************************************
// Function Name  : WriteLP_IntFlash
// Description    : Write lower bound pressure to memory
// Input          : lower bound pressure
// Return         : None
//*****************************************************************************
void WriteLP_IntFlash(uint16_t lower) {
	uint8_t num1 = FLASH_ReadByte(lowerPressure_address);
	uint8_t num2 = FLASH_ReadByte(lowerPressure_address+1);

	uint16_t lp = ((num2<<8) & num1);

	// If memory has not been written to
	if ((num1 == 0xFF) && (num2 == 0xFF)) {
		uint32_t setting = 0;
		setting += (uint32_t)lower;
		FLASH_ProgramWord(lowerPressure_address, setting);
	}

	else if (lp != lower) {
		//Erase
		FLASH_ErasePage(lowerPressure_page);
		//Rewrite
		uint32_t setting = 0;
		setting += (uint32_t)lower;
		FLASH_ProgramWord(lowerPressure_address, setting);
	}
}

//****************************************************************************
// Function Name  : WriteUP_IntFlash
// Description    : Write upper bound pressure to memory
// Input          : upper bound pressure
// Return         : None
//*****************************************************************************
void WriteUP_IntFlash(uint16_t upper) {
	uint8_t num1 = FLASH_ReadByte(upperPressure_address);
	uint8_t num2 = FLASH_ReadByte(upperPressure_address+1);

	uint16_t up = ((num2<<8) & num1);

	// If memory has not been written to
	if ((num1 == 0xFF) && (num2 == 0xFF)) {
		uint32_t setting = 0;
		setting += (uint32_t)upper;
		FLASH_ProgramWord(upperPressure_address, setting);
	}

	else if (up != upper) {
		//Erase
		FLASH_ErasePage(upperPressure_page);
		//Rewrite
		uint32_t setting = 0;
		setting += (uint32_t)upper;
		FLASH_ProgramWord(upperPressure_address, setting);
	}
}

//****************************************************************************
// Function Name  : WriteTP_IntFlash
// Description    : Write target bound pressure to memory
// Input          : target bound pressure
// Return         : None
//*****************************************************************************
void WriteTP_IntFlash(uint16_t target) {
	uint8_t num1 = FLASH_ReadByte(targetPressure_address);
	uint8_t num2 = FLASH_ReadByte(targetPressure_address+1);

	uint16_t tp = ((num2<<8) & num1);

	// If memory has not been written to
	if ((num1 == 0xFF) && (num2 == 0xFF)) {
		uint32_t setting = 0;
		setting += (uint32_t)target;
		FLASH_ProgramWord(targetPressure_address, setting);
	}

	else if (tp != target) {
		//Erase
		FLASH_ErasePage(targetPressure_page);
		//Rewrite
		uint32_t setting = 0;
		setting += (uint32_t)target;
		FLASH_ProgramWord(targetPressure_address, setting);
	}
}

//****************************************************************************
// Function Name  : WriteSP_IntFlash
// Description    : Write sampling period to memory
// Input          : sampling period
// Return         : None
//*****************************************************************************
void WriteSP_IntFlash(uint8_t period) {
	uint8_t num1 = FLASH_ReadByte(samplePeriod_address);

	// If memory has not been written to
	if (num1 == 0xFF) {
		uint32_t setting = 0;
		setting += (uint32_t)period;
		FLASH_ProgramWord(samplePeriod_address, setting);
	}

	else if (num1 != period) {
		//Erase
		FLASH_ErasePage(samplePeriod_page);
		//Rewrite
		uint32_t setting = 0;
		setting += (uint32_t)period;
		FLASH_ProgramWord(samplePeriod_address, setting);
	}
}

//****************************************************************************
// Function Name  : WriteNS_IntFlash
// Description    : Write number of samples to memory
// Input          : number of samples
// Return         : None
//*****************************************************************************
void WriteNS_IntFlash(uint8_t samples) {
	uint8_t num1 = FLASH_ReadByte(numSamples_address);

	// If memory has not been written to
	if (num1 == 0xFF) {
		uint32_t setting = 0;
		setting += (uint32_t)samples;
		FLASH_ProgramWord(numSamples_address, setting);
	}

	else if (num1 != samples) {
		//Erase
		FLASH_ErasePage(numSamples_page);
		//Rewrite
		uint32_t setting = 0;
		setting += (uint32_t)samples;
		FLASH_ProgramWord(numSamples_address, setting);
	}
}

//****************************************************************************
// Function Name  : WriteMode_IntFlash
// Description    : Write cushion mode to memory
// Input          : mode
// Return         : None
//*****************************************************************************
void WriteMode_IntFlash(uint8_t mode) {
	uint8_t num1 = FLASH_ReadByte(mode_address);

	// If memory has not been written to
	if (num1 == 0xFF) {
		uint32_t setting = 0;
		setting += (uint32_t)mode;
		FLASH_ProgramWord(mode_address, setting);
	}

	else if (num1 != mode) {
		//Erase
		FLASH_ErasePage(mode_page);
		//Rewrite
		uint32_t setting = 0;
		setting += (uint32_t)mode;
		FLASH_ProgramWord(mode_address, setting);
	}
}

//****************************************************************************
// Function Name  : ReadMultipliers_IntFlash
// Description    : Read value of pressure time multipliers from memory
// Input          : None
// Return         : None
//*****************************************************************************
void ReadMultipliers_IntFlash(){
	// Read from flash to get new multipliers
	multiplier_increase[0] = FLASH_ReadByte(multipliers_address+7);
	multiplier_increase[1] = FLASH_ReadByte(multipliers_address+6);
	multiplier_increase[2] = FLASH_ReadByte(multipliers_address+5);
	multiplier_increase[3] = FLASH_ReadByte(multipliers_address+4);
	multiplier_increase[4] = FLASH_ReadByte(multipliers_address+3);
	multiplier_increase[5] = FLASH_ReadByte(multipliers_address+2);
	multiplier_increase[6] = FLASH_ReadByte(multipliers_address+1);
	multiplier_increase[7] = FLASH_ReadByte(multipliers_address);

	multiplier_decrease[0] = FLASH_ReadByte(multipliers_address+15);
	multiplier_decrease[1] = FLASH_ReadByte(multipliers_address+14);
	multiplier_decrease[2] = FLASH_ReadByte(multipliers_address+13);
	multiplier_decrease[3] = FLASH_ReadByte(multipliers_address+12);
	multiplier_decrease[4] = FLASH_ReadByte(multipliers_address+11);
	multiplier_decrease[5] = FLASH_ReadByte(multipliers_address+10);
	multiplier_decrease[6] = FLASH_ReadByte(multipliers_address+9);
	multiplier_decrease[7] = FLASH_ReadByte(multipliers_address+8);

	// If uninitialized
	for (uint8_t i=0; i<8; i++){
		if (multiplier_increase[i] == 0xFF) {
			multiplier_increase[i] = 100; // Default value (100%)
		} if (multiplier_decrease[i] == 0xFF) {
			multiplier_decrease[i] = 100; // Default value (100%)
		}
	}

}

//****************************************************************************
// Function Name  : WriteMultipliers_IntFlash
// Description    : Writes values of pressure time multipliers from memory
// Input          : None
// Return         : None
//*****************************************************************************
void WriteMultipliers_IntFlash() {
	uint8_t num_i7 = FLASH_ReadByte(multipliers_address);
	uint8_t num_i6 = FLASH_ReadByte(multipliers_address+1);
	uint8_t num_i5 = FLASH_ReadByte(multipliers_address+2);
	uint8_t num_i4 = FLASH_ReadByte(multipliers_address+3);
	uint8_t num_i3 = FLASH_ReadByte(multipliers_address+4);
	uint8_t num_i2 = FLASH_ReadByte(multipliers_address+5);
	uint8_t num_i1 = FLASH_ReadByte(multipliers_address+6);
	uint8_t num_i0 = FLASH_ReadByte(multipliers_address+7);

	uint8_t num_d7 = FLASH_ReadByte(multipliers_address+8);
	uint8_t num_d6 = FLASH_ReadByte(multipliers_address+9);
	uint8_t num_d5 = FLASH_ReadByte(multipliers_address+10);
	uint8_t num_d4 = FLASH_ReadByte(multipliers_address+11);
	uint8_t num_d3 = FLASH_ReadByte(multipliers_address+12);
	uint8_t num_d2 = FLASH_ReadByte(multipliers_address+13);
	uint8_t num_d1 = FLASH_ReadByte(multipliers_address+14);
	uint8_t num_d0 = FLASH_ReadByte(multipliers_address+15);

	// If memory has not been written to
	if ((num_d0 == 0xFF) && (num_d1 == 0xFF) && (num_d2 == 0xFF) &&
			(num_d3 == 0xFF) && (num_d4 == 0xFF) && (num_d5 == 0xFF) &&
			(num_d6 == 0xFF) && (num_d7 == 0xFF) &&
			(num_i0 == 0xFF) && (num_i1 == 0xFF) && (num_i2 == 0xFF) &&
			(num_i3 == 0xFF) && (num_i4 == 0xFF) && (num_i5 == 0xFF) &&
			(num_i6 == 0xFF) && (num_i7 == 0xFF)){
		uint32_t word = 0;
		word += multiplier_increase[7];
		word += (multiplier_increase[6])<<8;
		word += (multiplier_increase[5])<<16;
		word += (multiplier_increase[4])<<24;
		FLASH_ProgramWord(multipliers_address, word);
		word = 0;
		word += multiplier_increase[3];
		word += (multiplier_increase[2])<<8;
		word += (multiplier_increase[1])<<16;
		word += (multiplier_increase[0])<<24;
		FLASH_ProgramWord(multipliers_address+4, word);
		word = 0;
		word += multiplier_decrease[7];
		word += (multiplier_decrease[6])<<8;
		word += (multiplier_decrease[5])<<16;
		word += (multiplier_decrease[4])<<24;
		FLASH_ProgramWord(multipliers_address+8, word);
		word = 0;
		word += multiplier_decrease[3];
		word += (multiplier_decrease[2])<<8;
		word += (multiplier_decrease[1])<<16;
		word += (multiplier_decrease[0])<<24;
		FLASH_ProgramWord(multipliers_address+12, word);
	}

	// Some info is changed, should erase and rewrite
	else if ((num_d0 != multiplier_decrease[0]) && (num_d1 != multiplier_decrease[1]) && (num_d2 != multiplier_decrease[2]) &&
			(num_d3 != multiplier_decrease[3]) && (num_d4 != multiplier_decrease[4]) && (num_d5 != multiplier_decrease[5]) &&
			(num_d6 != multiplier_decrease[6]) && (num_d7 != multiplier_decrease[7]) &&
			(num_i0 != multiplier_increase[0]) && (num_i1 != multiplier_increase[1]) && (num_i2 != multiplier_increase[2]) &&
			(num_i3 != multiplier_increase[3]) && (num_i4 != multiplier_increase[4]) && (num_i5 != multiplier_increase[5]) &&
			(num_i6 != multiplier_increase[6]) && (num_i7 != multiplier_increase[7])){
		//Erase
		FLASH_ErasePage(multipliers_page);

		//Rewrite
		uint32_t word = 0;
		word += multiplier_increase[7];
		word += (multiplier_increase[6])<<8;
		word += (multiplier_increase[5])<<16;
		word += (multiplier_increase[4])<<24;
		FLASH_ProgramWord(multipliers_address, word);
		word = 0;
		word += multiplier_increase[3];
		word += (multiplier_increase[2])<<8;
		word += (multiplier_increase[1])<<16;
		word += (multiplier_increase[0])<<24;
		FLASH_ProgramWord(multipliers_address+4, word);
		word = 0;
		word += multiplier_decrease[7];
		word += (multiplier_decrease[6])<<8;
		word += (multiplier_decrease[5])<<16;
		word += (multiplier_decrease[4])<<24;
		FLASH_ProgramWord(multipliers_address+8, word);
		word = 0;
		word += multiplier_decrease[3];
		word += (multiplier_decrease[2])<<8;
		word += (multiplier_decrease[1])<<16;
		word += (multiplier_decrease[0])<<24;
		FLASH_ProgramWord(multipliers_address+12, word);
	}

	SEGGER_RTT_printf(0, "Wrote multipliers to flash\n");
}

/*******************************************************************************
 * Function Name  : DeInitialize()
 * Description    : Deinitializes some peripherals to save power
 * Input          : None
 * Return         : None
 *******************************************************************************/
void DeInitialize(){
	// TODO: Low power for spi enable bits? Check Johnny's code
	// Turn off peripherals for disconnected mode, to save power
	// Internal ADC remains on to monitor battery level
	if (disconnectedMode == FALSE){
		//HAL_VTimer_Stop(0); // Stop notification timer
		HAL_VTimer_Stop(1); // Stop connection monitoring timer

		SPI_DeepPowerDown_Memory();
		GPIO_ResetBits(RED_LED|BLUE_LED|PVControl|ValveControl|Valve1|Valve2|Valve3|Valve4|Valve5|Valve6|Valve7|Valve8|SensorControl);
		SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_SPI, DISABLE);
		SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_ADC, DISABLE);

		disconnectedMode = TRUE;
		SEGGER_RTT_printf(0, "Disconnected mode on\n");
	}
}

/*******************************************************************************
 * Function Name  : ReInitialize()
 * Description    : Reinitializes some peripherals that had been previously deinitialized
 * Input          : None
 * Return         : None
 *******************************************************************************/
void ReInitialize(){
	HAL_VTimer_Stop(1);
	if (disconnectedMode == TRUE) {
		//HAL_VTimerStart_ms(0, 1000); // Start notification timer

		disconnectedMode = FALSE;

		// Turn peripherals back on
		SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_SPI, ENABLE);
		SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_ADC, ENABLE);
		GPIO_SetBits(RED_LED|BLUE_LED|SensorControl);
		SPI_DeepPowerRelease_Memory();

		SEGGER_RTT_printf(0, "Disconnected mode off\n");
	}

}

// ----- TESTING FUNCTIONS ---------------------------------------------------

//****************************************************************************
// Function Name  : SPI_Memory_Test
// Description    : A test of the SPI flash memory for development purposes
// Input          : None
// Return         : None
//*****************************************************************************

void SPI_Memory_Test(){
	SEGGER_RTT_printf(0, "SPI Memory Test in progress...\n");

	// Reset memory
	SPI_WriteEnable_Memory();
	SPI_BulkErase_Memory();

	// Writing to sector 0
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(0x00, 0x55, FALSE);
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(0x01, 0xAA, FALSE);
	// Writing to sector 1 (256*256=0x10000)
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(0x10000, 0x55, FALSE);

	uint8_t Read_Result1 = SPI_Read_Memory(0x00);
	uint8_t Read_Result2 = SPI_Read_Memory(0x01);
	uint8_t Read_Result3 = SPI_Read_Memory(0x10000);

	if ((Read_Result1 != 0x55) || (Read_Result2 != 0xAA) || (Read_Result3 != 0x55)){
		SEGGER_RTT_printf(0, "SPI Memory Test: Initial write failed; Result1 = %d, Result2 = %d, Result3 = %d\n", Read_Result1, Read_Result2, Read_Result3);
		return;
	}

	// Clearing sector 0 (clears to 1's)
	SPI_WriteEnable_Memory();
	SPI_SectorErase_Memory(0x01);

	Read_Result1 = SPI_Read_Memory(0x00);
	Read_Result2 = SPI_Read_Memory(0x01);
	Read_Result3 = SPI_Read_Memory(0x10000);

	if ((Read_Result1 != 0xFF) || (Read_Result2 != 0xFF) || (Read_Result3 != 0x55)){ // Sector 1 will not have been cleared
		SEGGER_RTT_printf(0, "SPI Memory Test: Sector erase failed; Result1 = %d, Result2 = %d, Result3 = %d\n", Read_Result1, Read_Result2, Read_Result3);
		return;
	}

	// Reset memory
	SPI_WriteEnable_Memory();
	SPI_BulkErase_Memory();

	Read_Result1 = SPI_Read_Memory(0x00);
	Read_Result2 = SPI_Read_Memory(0x01);
	Read_Result3 = SPI_Read_Memory(0x10000);

	if ((Read_Result1 != 0xFF) || (Read_Result2 != 0xFF) || (Read_Result3 != 0xFF)){
		SEGGER_RTT_printf(0, "SPI Memory Test: Bulk erase failed\n; Result1 = %d, Result2 = %d, Result3 = %d\n", Read_Result1, Read_Result2, Read_Result3);
		return;
	}

	SEGGER_RTT_printf(0, "SPI Memory Test: Passed\n");
}

//****************************************************************************
// Function Name  : SPI_ADC_Test
// Description    : A test of the SPI external ADC for development purposes
// Input          : None
// Return         : None
//*****************************************************************************

void SPI_ADC_Test(){
	SEGGER_RTT_printf(0, "SPI ADC Test in progress...\n");

	// Set SEQUENCE_CFG.SEQ_MODE to 1 - auto sequence mode
	// and set SEQUENCE_CFG.SEQ_START to 1 - start channel sequencing in ascending order
	SPI_Write_ADC_Register(0x10, 0b00010001);

	SPI_Write_ADC_Register(0x2, 0x80); // Output fixed pattern from ADC

	uint16_t Read_Result = SPI_Read_ADC_Value();

	if (Read_Result == 0xA5A){
		SEGGER_RTT_printf(0, "SPI ADC Test: Passed\n");
	} else {
		SEGGER_RTT_printf(0, "SPI ADC Test: Failed; Result = 0x%x\n", Read_Result);
	}

	SPI_Write_ADC_Register(0x2, 0x00); // Return to normal
	SPI_Write_ADC_Register(0x10, 0x00); // Stop channel sequencing
}

//****************************************************************************
// Function Name  : BLE_DataDownloader_Test
// Description    : A test of memory contents download over BLE, for development purposes
// Input          : None
// Return         : None
//*****************************************************************************

void BLE_DataDownloader_Test(){
	SEGGER_RTT_printf(0, "Data Downloader Test in progress...\n");

	uint16_t pressurePlaceholder;

	// Clear memory
	SPI_WriteEnable_Memory();
	SPI_BulkErase_Memory();

	// Header starts with ABCDEF
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, 0xAB, TRUE);
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, 0xCD, TRUE);
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, 0xEF, TRUE);

	// Write date & time data
	// Aug. 15, 2021 3:20 pm
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, 21, TRUE);
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, 8, TRUE);
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, 15, TRUE);
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, 15, TRUE);
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, 20, TRUE);

	// Reserve byte to make header length 9 bytes
	SPI_WriteEnable_Memory();
	SPI_Write_Memory(currentAddress, 0x00, TRUE);

	for (uint16_t i=0; i<249; i++) {
		// Write difference in minutes from header time
		SPI_WriteEnable_Memory();
		SPI_Write_Memory(currentAddress, (uint8_t)(i>>8), TRUE);
		SPI_WriteEnable_Memory();
		SPI_Write_Memory(currentAddress, (uint8_t)i, TRUE);
		for (uint16_t j=0; j<8; j++){
			// Write sensor values to memory
			pressurePlaceholder = (uint16_t)(i+j+10);  // Random value
			// SEGGER_RTT_printf(0, "Wrote: %d mmHg\n", pressurePlaceholder);
			SPI_WriteEnable_Memory();
			SPI_Write_Memory(currentAddress, (uint8_t)(pressurePlaceholder>>8), TRUE);
			SPI_WriteEnable_Memory();
			SPI_Write_Memory(currentAddress, (uint8_t)pressurePlaceholder, TRUE);
		}
	}

	WriteAddress_IntFlash(currentAddress); // Write new currentAddress to memory

	SEGGER_RTT_printf(0, "Wrote to memory ... Enable download notification now\n");
}

/*****END OF FILE****/
