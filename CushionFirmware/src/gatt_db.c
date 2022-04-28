/*******************************************************************************
* File Name          : gatt_db.c
* Author             : Brendan Coutts / Ibrahim Elmallah
* Date               : August-2021
* Description        : Functions to build GATT DB and handle GATT events.
*******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "ble_const.h"
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Config.h"
#include "BlueNRG1_adc.h"
#include "clock.h"
#include "gp_timer.h"
#include "gatt_db.h"
#include "osal.h"

#include "defines.h"

/* Private define ------------------------------------------------------------*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

#define NOTIFICATION_ENABLE 			1
#define ENC_KEY_SIZE					16
#define FIXED_SIZE						0
#define VARIABLE						1
#define NO_NOTIFICATION 				0
#define NOTIFICATION					1
#define DEFAULT_OFFSET					0
#define LENGTH(array)					(sizeof(array)/sizeof(array[0]))


/* Service Handles */
uint16_t 	battSensServiceHandle, battValCharHandle,
			LEDServiceHandle, LEDValCharHandle,
			pressureServiceHandle, pressureValCharHandle,
			controlServiceHandle, valveSmallCharHandle,
			valveLargeCharHandle, pumpCharHandle,
			bagControlCharHandle, bagRedistributeCharHandle,
			initializeBagsCharHandle, releaseBagsCharHandle,
			longTermServiceHandle, longTermModeHandle,
			dataDownloadServiceHandle, dataDownloadValCharHandle,
			settingsServiceHandle, lowerBoundPressureCharHandle,
			upperBoundPressureCharHandle, targetPressureCharHandle,
			samplingPeriodCharHandle, numSamplesCharHandle,
			modeServiceHandle, modeCharHandle,
			deviceInfoServiceHandle, deviceInfoCharHandle,
			dateTimeServiceHandle, dateTimeCharHandle;

/* UUIDS */
Service_UUID_t service_uuid;
Char_UUID_t char_uuid;
Char_Desc_Uuid_t char_desc_uuid;

#define BattServiceUUID 0x180F
#define BattValCharUUID 0x2A00

#define LEDServiceUUID 0x1900
#define LEDCharUUID 0x2B00

#define PressureServiceUUID 0x1901
#define SensorCharUUID 0x2B10

#define ControlServiceUUID 0x1902
#define ValveSmallCharUUID 0x2B20
#define ValveLargeCharUUID 0x2B21
#define PumpCharUUID 0x2B22
#define BagControlCharUUID 0x2B23
#define BagRedistributeCharUUID 0x2B24
#define InitializeBagsCharUUID 0x2B25
#define ReleaseBagsCharUUID 0x2B26

#define LTMServiceUUID 0x1903
#define LTMModeCharUUID 0x2B30

#define DataDownloaderServiceUUID 0x1904
#define DataDownloaderCharUUID 0x2B40

#define SettingsServiceUUID 0x1905
#define LowerBoundPressureCharUUID 0x2B50
#define UpperBoundPressureCharUUID 0x2B51
#define TargetPressureCharUUID 0x2B52
#define SamplingPeriodCharUUID 0x2B53
#define NumSamplesCharUUID 0x2B54

#define ModeServiceUUID 0x1906
#define ModeCharUUID 0x2B60

#define DeviceInfoServiceUUID 0x1907
#define DeviceInfoCharUUID 0x2B70

#define DateTimeServiceUUID 0x1908
#define DateTimeCharUUID 0x2B80

/* Extern variables */
extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);

/*******************************************************************************
* Function Name  : Add_Sensor_Service
* Description    : Add the Sensor services.
* Input          : None
* Return         : Status.
*******************************************************************************/
tBleStatus Add_Sensor_Service(void)
{
	tBleStatus ret;

	/* Battery Service Characteristic */
	uint16_t uuid16 = BattServiceUUID;
	Osal_MemCpy(&service_uuid.Service_UUID_16, &uuid16, 2);
	ret = aci_gatt_add_service(UUID_TYPE_16,&service_uuid, PRIMARY_SERVICE, 5 , &battSensServiceHandle);
	//SEGGER_RTT_printf(0,"here1:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Battery Value Characteristic. Read only. Size: 2 byte */
	uuid16 = BattValCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(battSensServiceHandle, UUID_TYPE_16, &char_uuid, 2, CHAR_PROP_READ, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
	  ENC_KEY_SIZE, FIXED_SIZE, &battValCharHandle);
	//SEGGER_RTT_printf(0,"here1.1:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* LED Service Characteristic */
	uuid16 = LEDServiceUUID;
	Osal_MemCpy(&service_uuid.Service_UUID_16, &uuid16, 2);
	ret = aci_gatt_add_service(UUID_TYPE_16, &service_uuid, PRIMARY_SERVICE, 5, &LEDServiceHandle);
	//SEGGER_RTT_printf(0,"here2:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* LED Value Characteristic. Read/Write. Size: 2 bytes*/
	uuid16 = LEDCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(LEDServiceHandle, UUID_TYPE_16, &char_uuid, 2,  CHAR_PROP_READ|CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE|GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,ENC_KEY_SIZE, FIXED_SIZE, &LEDValCharHandle);
	//SEGGER_RTT_printf(0,"here2.1:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Pressure Service Characteristic.  */
	uuid16 = PressureServiceUUID;
	Osal_MemCpy(&service_uuid.Service_UUID_16, &uuid16, 2);
	ret = aci_gatt_add_service(UUID_TYPE_16, &service_uuid, PRIMARY_SERVICE, 5, &pressureServiceHandle);
    //SEGGER_RTT_printf(0,"here3:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Pressure Value Characteristic. Read/Notify. Size: 16 byte */
	uuid16 = SensorCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(pressureServiceHandle, UUID_TYPE_16, &char_uuid, 16, CHAR_PROP_READ|CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP|GATT_NOTIFY_ATTRIBUTE_WRITE, ENC_KEY_SIZE, FIXED_SIZE, &pressureValCharHandle);
   // SEGGER_RTT_printf(0,"here3.1:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Control Service Characteristic. */
	uuid16 = ControlServiceUUID;
	Osal_MemCpy(&service_uuid.Service_UUID_16, &uuid16, 2);
	ret = aci_gatt_add_service(UUID_TYPE_16, &service_uuid, PRIMARY_SERVICE, 15, &controlServiceHandle);
	//SEGGER_RTT_printf(0,"here4:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Small Valves Value Characteristic. Read/Write. Size: 2 bytes */
	uuid16 = ValveSmallCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(controlServiceHandle, UUID_TYPE_16, &char_uuid, 2, CHAR_PROP_READ|CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE|GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,ENC_KEY_SIZE, FIXED_SIZE, &valveSmallCharHandle);
	//SEGGER_RTT_printf(0,"here4.1:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Large Value Characteristic. Read/Write. Size: 1 byte */
	uuid16 = ValveLargeCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(controlServiceHandle, UUID_TYPE_16, &char_uuid, 1, CHAR_PROP_READ|CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE|GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,ENC_KEY_SIZE, FIXED_SIZE, &valveLargeCharHandle);
	//SEGGER_RTT_printf(0,"here4.2:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Pump Value Characteristic. Read/Write. Size: 1 byte */
	uuid16 = PumpCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(controlServiceHandle, UUID_TYPE_16, &char_uuid, 1, CHAR_PROP_READ|CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE|GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,ENC_KEY_SIZE, FIXED_SIZE, &pumpCharHandle);
	//SEGGER_RTT_printf(0,"here4.3:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Air Bag Control Characteristic. Write. Size: 3 bytes */
	uuid16 = BagControlCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(controlServiceHandle, UUID_TYPE_16, &char_uuid, 3, CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE,ENC_KEY_SIZE, FIXED_SIZE, &bagControlCharHandle);
	//SEGGER_RTT_printf(0,"here4.4:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Air Bag Redistribute air Characteristic. Write. Size: 5 bytes */
	uuid16 = BagRedistributeCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(controlServiceHandle, UUID_TYPE_16, &char_uuid, 5, CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE,ENC_KEY_SIZE, FIXED_SIZE, &bagRedistributeCharHandle);
	//SEGGER_RTT_printf(0,"here4.5:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Initialize air bags Characteristic. Write. Size: 2 bytes */
	uuid16 = InitializeBagsCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(controlServiceHandle, UUID_TYPE_16, &char_uuid, 2, CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE,ENC_KEY_SIZE, FIXED_SIZE, &initializeBagsCharHandle);
	//SEGGER_RTT_printf(0,"here4.6:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Release air bags Characteristic. Write. Size: 1 bytes */
	uuid16 = ReleaseBagsCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(controlServiceHandle, UUID_TYPE_16, &char_uuid, 1, CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE,ENC_KEY_SIZE, FIXED_SIZE, &releaseBagsCharHandle);
	//SEGGER_RTT_printf(0,"here4.7:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Long Term Service Characteristic */
	uuid16 = LTMServiceUUID;
	Osal_MemCpy(&service_uuid.Service_UUID_16, &uuid16, 2);
	ret = aci_gatt_add_service(UUID_TYPE_16, &service_uuid, PRIMARY_SERVICE, 7, &longTermServiceHandle);
	//SEGGER_RTT_printf(0,"here5:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Long Term Mode Characteristic. Read/Write. Size: 1 byte */
	uuid16 = LTMModeCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(longTermServiceHandle, UUID_TYPE_16, &char_uuid, 1 , CHAR_PROP_READ|CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE|GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,ENC_KEY_SIZE, FIXED_SIZE, &longTermModeHandle);
	//SEGGER_RTT_printf(0,"here5.1:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Data Downloader Service Characteristic */
	uuid16 = DataDownloaderServiceUUID;
	Osal_MemCpy(&service_uuid.Service_UUID_16, &uuid16, 2);
	ret = aci_gatt_add_service(UUID_TYPE_16, &service_uuid, PRIMARY_SERVICE, 10, &dataDownloadServiceHandle);
	//SEGGER_RTT_printf(0,"here6:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Data Downloader Value Characteristic. Read/Write/Notify. Size: 216 byte */
	uuid16 = DataDownloaderCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(dataDownloadServiceHandle, UUID_TYPE_16, &char_uuid, 216 , CHAR_PROP_NOTIFY|CHAR_PROP_READ|CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP|GATT_NOTIFY_ATTRIBUTE_WRITE,ENC_KEY_SIZE,
			VARIABLE, &dataDownloadValCharHandle);
	//SEGGER_RTT_printf(0,"here6.1:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Settings Service Characteristic */
	uuid16 = SettingsServiceUUID;
	Osal_MemCpy(&service_uuid.Service_UUID_16, &uuid16, 2);
	ret = aci_gatt_add_service(UUID_TYPE_16, &service_uuid, PRIMARY_SERVICE, 13, &settingsServiceHandle);
	//SEGGER_RTT_printf(0,"here7:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Lower Bound Pressure Characteristic. Read/Write. Size: 2 bytes */
	uuid16 = LowerBoundPressureCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(settingsServiceHandle, UUID_TYPE_16, &char_uuid, 2, CHAR_PROP_READ|CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE|GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,ENC_KEY_SIZE, FIXED_SIZE, &lowerBoundPressureCharHandle);
	//SEGGER_RTT_printf(0,"here7.1:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Upper Bound Pressure Characteristic. Read/Write. Size: 2 bytes */
	uuid16 = UpperBoundPressureCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(settingsServiceHandle, UUID_TYPE_16, &char_uuid, 2, CHAR_PROP_READ|CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE|GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,ENC_KEY_SIZE, FIXED_SIZE, &upperBoundPressureCharHandle);
	//SEGGER_RTT_printf(0,"here7.2:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Target Bound Pressure Characteristic. Read/Write. Size: 2 bytes */
	uuid16 = TargetPressureCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(settingsServiceHandle, UUID_TYPE_16, &char_uuid, 2, CHAR_PROP_READ|CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE|GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,ENC_KEY_SIZE, FIXED_SIZE, &targetPressureCharHandle);
	//SEGGER_RTT_printf(0,"here7.3:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Sampling Period Characteristic. Read/Write. Size: 1 byte */
	uuid16 = SamplingPeriodCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(settingsServiceHandle, UUID_TYPE_16, &char_uuid, 1, CHAR_PROP_READ|CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE|GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,ENC_KEY_SIZE, FIXED_SIZE, &samplingPeriodCharHandle);
	//SEGGER_RTT_printf(0,"here7.4:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Number of Samples Characteristic. Read/Write. Size: 1 byte */
	uuid16 = NumSamplesCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(settingsServiceHandle, UUID_TYPE_16, &char_uuid, 1, CHAR_PROP_READ|CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE|GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,ENC_KEY_SIZE, FIXED_SIZE, &numSamplesCharHandle);
	//SEGGER_RTT_printf(0,"here7.5:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Mode Service Characteristic */
	uuid16 = ModeServiceUUID;
	Osal_MemCpy(&service_uuid.Service_UUID_16, &uuid16, 2);
	ret = aci_gatt_add_service(UUID_TYPE_16, &service_uuid, PRIMARY_SERVICE, 3, &modeServiceHandle);
	//SEGGER_RTT_printf(0,"here8:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Mode Characteristic. Read/Write. Size: 1 byte */
	uuid16 = ModeCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(modeServiceHandle, UUID_TYPE_16, &char_uuid, 1, CHAR_PROP_READ|CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE|GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,ENC_KEY_SIZE, FIXED_SIZE, &modeCharHandle);
	//SEGGER_RTT_printf(0,"here8.1:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Device Info Service Characteristic */
	uuid16 = DeviceInfoServiceUUID;
	Osal_MemCpy(&service_uuid.Service_UUID_16, &uuid16, 2);
	ret = aci_gatt_add_service(UUID_TYPE_16, &service_uuid, PRIMARY_SERVICE, 3, &deviceInfoServiceHandle);
	//SEGGER_RTT_printf(0,"here9:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Device Info Characteristic. Read/Write. Size: 3 bytes */
	uuid16 = DeviceInfoCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(deviceInfoServiceHandle, UUID_TYPE_16, &char_uuid, 3, CHAR_PROP_READ|CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE|GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,ENC_KEY_SIZE, FIXED_SIZE, &deviceInfoCharHandle);
	//SEGGER_RTT_printf(0,"here9.1:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Date & Time Service Characteristic */
	uuid16 = DateTimeServiceUUID;
	Osal_MemCpy(&service_uuid.Service_UUID_16, &uuid16, 2);
	ret = aci_gatt_add_service(UUID_TYPE_16, &service_uuid, PRIMARY_SERVICE, 3, &dateTimeServiceHandle);
	//SEGGER_RTT_printf(0,"here10:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	/* Date & Time Characteristic. Read/Write. Size: 5 bytes */
	uuid16 = DateTimeCharUUID;
	Osal_MemCpy(&char_uuid.Char_UUID_16, &uuid16, 2);
	ret =  aci_gatt_add_char(dateTimeServiceHandle, UUID_TYPE_16, &char_uuid, 5, CHAR_PROP_READ|CHAR_PROP_WRITE, ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE|GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,ENC_KEY_SIZE, FIXED_SIZE, &dateTimeCharHandle);
	//SEGGER_RTT_printf(0,"here10.1:%04x\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
		return BLE_STATUS_ERROR;

	SEGGER_RTT_printf(0,"Finished adding all services\n");
	return BLE_STATUS_SUCCESS;
}


/*******************************************************************************
* Function Name  : Read_Batt
* Description    : Read battery characteristic value
* Input          : None
* Return         : Status.
*******************************************************************************/
tBleStatus Read_Batt()
{
	tBleStatus ret;
	uint16_t value = ADC_Value(ADC_Input_AdcPin2);

	ret = aci_gatt_update_char_value_ext(connection_handle, battSensServiceHandle, battValCharHandle, NO_NOTIFICATION, 2, DEFAULT_OFFSET, 2, (uint8_t*)&value);
	return ret;
}


/*******************************************************************************
* Function Name  : Read_Pressure
* Description    : Read pressure characteristic value
* Input          : None
* Return         : Status.
*******************************************************************************/
tBleStatus Read_Pressure()
{
	tBleStatus ret;

	Read_ADC();

	for (int i=0; i<2; i++) SEGGER_RTT_printf(0, "Sensor %d: %d\n", i, CurrentPressures[i]); //TODO: Remove

	ret = aci_gatt_update_char_value_ext(connection_handle, pressureServiceHandle, pressureValCharHandle, NO_NOTIFICATION, 16, DEFAULT_OFFSET, 16, CurrentPressures);
	return ret;
}

/*******************************************************************************
* Function Name  : Notify_Pressure
* Description    : Notify pressure characteristic value
* Input          : None
* Return         : Status.
*******************************************************************************/
tBleStatus Notify_Pressure()
{
	tBleStatus ret;

	Read_ADC();

	//for (int i=0; i<8; i++) SEGGER_RTT_printf(0,"Pressure %d: %d mmHg\n", i, CurrentPressures[i]);

	ret = aci_gatt_update_char_value_ext(connection_handle, pressureServiceHandle, pressureValCharHandle, NOTIFICATION, 16, DEFAULT_OFFSET, 16, CurrentPressures);
	return ret;
}

/*******************************************************************************
* Function Name  : Notify_Download_Val
* Description    : Update memory records characteristic value
* Input          : memory data
* Return         : Status.
*******************************************************************************/
tBleStatus Notify_Download_Val(uint8_t* data, uint8_t size)
{

	tBleStatus ret = aci_gatt_update_char_value_ext(connection_handle, dataDownloadServiceHandle,
			dataDownloadValCharHandle, NOTIFICATION, size, DEFAULT_OFFSET, size, data);
	return ret;
}

/*******************************************************************************
* Function Name  : Read_Control_SmallValves
* Description    : Read the state of the small valves
* Input          : None.
* Return         : Status.
*******************************************************************************/
tBleStatus Read_Control_SmallValves()
{
	tBleStatus ret = aci_gatt_update_char_value_ext(connection_handle, controlServiceHandle, valveSmallCharHandle, NO_NOTIFICATION, 1, DEFAULT_OFFSET, 1, (uint8_t*)&currentState);

	return ret;
}

/*******************************************************************************
* Function Name  : Read_Control_BigValve
* Description    : Read the state of the big valve
* Input          : None.
* Return         : Status.
*******************************************************************************/
tBleStatus Read_Control_BigValve()
{
	uint8_t state;

	if (GPIO_ReadBit(ValveControl) == Bit_SET) {
		state = 0x01;
	} else {
		state = 0x00;
	}

	tBleStatus ret = aci_gatt_update_char_value_ext(connection_handle, controlServiceHandle, valveLargeCharHandle, NO_NOTIFICATION, 1, DEFAULT_OFFSET, 1, (uint8_t*)&state);

	return ret;
}

/*******************************************************************************
* Function Name  : Read_Control_Pump
* Description    : Read the state of the pump
* Input          : None.
* Return         : Status.
*******************************************************************************/
tBleStatus Read_Control_Pump()
{
	uint8_t state;

	if (GPIO_ReadBit(PVControl) == Bit_SET) {
		state = 0x01;
	} else {
		state = 0x00;
	}

	tBleStatus ret = aci_gatt_update_char_value_ext(connection_handle, controlServiceHandle, pumpCharHandle, NO_NOTIFICATION, 1, DEFAULT_OFFSET, 1, (uint8_t*)&state);

	return ret;
}

/*******************************************************************************
* Function Name  : Read_Device_Time
* Description    : Read date and time values of last header
* Input          : None.
* Return         : Status.
*******************************************************************************/
tBleStatus Read_Device_Time()
{
	uint8_t data[5];
	data[0] = year;
	data[1] = month;
	data[2] = day;
	data[3] = hour;
	data[4] = minute;

	tBleStatus ret = aci_gatt_update_char_value_ext(connection_handle, deviceInfoServiceHandle, deviceInfoCharHandle, NO_NOTIFICATION,6, DEFAULT_OFFSET, 5, data);
	return ret;
}

/*******************************************************************************
* Function Name  : External_Memory_Clear
* Description    : Erase memory
* Input          : None.
* Return         : None.
*******************************************************************************/
void External_Memory_Clear(){
	// Clear memory
	SPI_WriteEnable_Memory();
	SPI_BulkErase_Memory();

	uint8_t confirm = 1;
	aci_gatt_update_char_value_ext(connection_handle,dataDownloadServiceHandle,dataDownloadValCharHandle, NO_NOTIFICATION,1, DEFAULT_OFFSET, 1, (uint8_t*)&confirm);
}

//*******************************************************************************
// Function Name  : Read_Request_CB.
// Description    : Callback of read requests.
// Input          : Handle of the characteristic to update.
// Return         : None.
//*******************************************************************************
void Read_Request_CB(uint16_t handle)
{
	tBleStatus ret;
	if (connection_handle!=0){
		ret = aci_gatt_allow_read(connection_handle);
	  if (ret != BLE_STATUS_SUCCESS)
		return;
	}

	if (handle == battValCharHandle+1) {
		ret = Read_Batt();
		SEGGER_RTT_printf(0, "Read battery\n");
	}
	else if (handle == pressureValCharHandle+1){
		ret = Read_Pressure();
		SEGGER_RTT_printf(0, "Read pressure\n");
	}
	else if (handle == valveSmallCharHandle+1) {
		ret = Read_Control_SmallValves();
		SEGGER_RTT_printf(0, "Read small valves\n");
	}
	else if (handle == valveLargeCharHandle+1) {
		ret = Read_Control_BigValve();
		SEGGER_RTT_printf(0, "Read big valves\n");
	}
	else if (handle == pumpCharHandle+1) {
		ret = Read_Control_Pump();
		SEGGER_RTT_printf(0, "Read pump\n");
	}
	else if (handle == longTermModeHandle+1) {
		ret = disconnectedMode;
		SEGGER_RTT_printf(0, "Read LTM\n");
	}
	else if (handle == dataDownloadValCharHandle+1) {
		SEGGER_RTT_printf(0, "Clearing memory...\n");
		External_Memory_Clear();
		SEGGER_RTT_printf(0, "Read - Memory clear\n");
	}
	else if (handle == dateTimeCharHandle+1) {
		Read_Device_Time();
		SEGGER_RTT_printf(0, "Read last updated date & time\n");
	}

}


///*******************************************************************************
// Function Name  : Write_Request_CB.
// Description    : Callback of write requests.
// Input          : Handle of the characteristic to update. Data to be write and their length.
// Return         : None.
//*******************************************************************************
void Write_Request_CB(uint16_t handle, uint16_t data_length, uint8_t *att_data) {

	// A notification is issued
	if (handle == pressureValCharHandle+2) {
		//if(data_length == 2) { //make sure it is enable/disable notification
			if (att_data[0]==ENABLE){
				isPressureReaderNotificationEnabled = TRUE;
			}
			else if (att_data[0]==DISABLE){
				isPressureReaderNotificationEnabled = FALSE;
			}
		//}
	}
	else if (handle == dataDownloadValCharHandle+2){
		//make sure it is enable/disable notification
		//if(data_length == 2){
			if (att_data[0]==ENABLE){
				//Disable Other Notifications to increase efficiency
				isPressureReaderNotificationEnabled = FALSE;
				isDownloadNotificationEnabled = TRUE;
				address = 0;
			}
			else if (att_data[0]==DISABLE)
				isDownloadNotificationEnabled = FALSE;

			SEGGER_RTT_printf(0, "DataDownload notification of: %x\n", att_data[0]);
		//}
	}

	// a write is issued
	else if (handle == LEDValCharHandle+1) {
		if (att_data[0] == 0) { // Red LED
			if (att_data[1] == 0) // Off
				LEDs_Set_Red(FALSE);
			else if (att_data[1] == 1) // On
				LEDs_Set_Red(TRUE);
		}
		else if (att_data[0] == 1) { // Blue LED
			if (att_data[1] == 0) // Off
				LEDs_Set_Blue(FALSE);
			else if (att_data[1] == 1) // On
				LEDs_Set_Blue(TRUE);
		}
		SEGGER_RTT_printf(0, "LED state changed to LED #%d, state %d\n", att_data[0], att_data[1]);
	}

	else if (handle == valveSmallCharHandle+1) {
		uint8_t valve_state = 0;
		uint8_t valve_number = 0;

		if (att_data[1] == 0){
			valve_state = 0x00;
		} else if (att_data[1] == 1){
			valve_state = 0xFF;
		}

		switch (att_data[0]) { // To make data work with my function
		case 0: valve_number = 1; break;
		case 1: valve_number = 2; break;
		case 2: valve_number = 4; break;
		case 3: valve_number = 8; break;
		case 4: valve_number = 16; break;
		case 5: valve_number = 32; break;
		case 6: valve_number = 64; break;
		case 7: valve_number = 128; break;
		default: valve_number = 0;
		}

		Adjust_Valve_State(valve_number & valve_state);
		SEGGER_RTT_printf(0, "Valve state changed to: Valve %x, State %x\n", att_data[0], att_data[1]);
	}

	else if (handle == valveLargeCharHandle+1) {
		if (att_data[0] == 1)
			Open_Valve_Large();
		else if (att_data[0] == 0)
			Close_Valve_Large();

		// For testing purposes
		/*Adjust_Valve_State(1);
		Open_Valve_Large();
		Clock_Wait(300);
		Close_Valve_Large();
		Adjust_Valve_State(0);*/

		SEGGER_RTT_printf(0, "Toggled large valve: %x\n", att_data[0]);
	}

	else if (handle == pumpCharHandle+1) {
		if (att_data[0] == 1)
			Pump_On();
		else if (att_data[0] == 0)
			Pump_Off();

		// For testing purposes
		/*Adjust_Valve_State(1);
		Pump_On();
		Clock_Wait(5000);
		Pump_Off();
		Adjust_Valve_State(0);*/

		SEGGER_RTT_printf(0, "Toggled pump: %x\n", att_data[0]);
	}

	else if (handle == bagControlCharHandle+1) {
		uint16_t pressure = (att_data[1] << 8) | att_data[2];
		SEGGER_RTT_printf(0, "Air bag control. bag: %x; value: %d mmHg\n",
						att_data[0], pressure);
		if (pressure > CurrentPressures[att_data[0]-1])
			Pressurize_Air_Bag(att_data[0], pressure, 1);
		else if (pressure < CurrentPressures[att_data[0]-1])
			Depressurize_Air_Bag(att_data[0], pressure, 1);
	}

	else if (handle == bagRedistributeCharHandle+1) {
		uint16_t pressure = (att_data[2] << 8) | att_data[3];
		SEGGER_RTT_printf(0, "Redistributing pressure from bag %d to %d; Target: %d mmHg\n",
					att_data[0], att_data[1], pressure);
		Redistribute_Pressure(att_data[0], att_data[1], pressure);
	}

	else if (handle == initializeBagsCharHandle+1) {
		uint16_t pressure = (att_data[0] << 8) | att_data[1];
		SEGGER_RTT_printf(0, "Initializing air bags. Target: %d mmHg\n", pressure);
		Initialize_AirBags(pressure);
	}

	else if (handle == releaseBagsCharHandle+1) {
		SEGGER_RTT_printf(0, "Releasing air from all air bags\n");
		Release_AirBags();
	}

	else if (handle == longTermModeHandle+1) {
		if (att_data[0] == 1) {
			if (disconnectedMode == FALSE) {
				DeInitialize();
			}
		} else if (att_data[0] == 0) {
			if (disconnectedMode == TRUE) {
				ReInitialize();
			}
		}

		SEGGER_RTT_printf(0, "Toggled LTM\n");
	}

	else if (handle == lowerBoundPressureCharHandle + 1) {
		lowerBoundPressure = (att_data[0] << 8) | att_data[1];
		WriteLP_IntFlash(lowerBoundPressure);
		SEGGER_RTT_printf(0, "Lower Bound Pressure set: %d mmHg\n", lowerBoundPressure);
	}

	else if (handle == upperBoundPressureCharHandle + 1) {
		upperBoundPressure = (att_data[0] << 8) | att_data[1];
		WriteUP_IntFlash(upperBoundPressure);
		SEGGER_RTT_printf(0, "Upper Bound Pressure set: %d mmHg\n", upperBoundPressure);
	}

	else if (handle == targetPressureCharHandle + 1) {
		targetPressure = (att_data[0] << 8) | att_data[1];
		WriteTP_IntFlash(targetPressure);
		SEGGER_RTT_printf(0, "Target Pressure set: %d mmHg\n", targetPressure);
	}

	else if (handle == samplingPeriodCharHandle + 1) {
		samplingPeriod = att_data[0];
		WriteSP_IntFlash(samplingPeriod);
		SEGGER_RTT_printf(0, "Sampling Period set: %d minutes\n", samplingPeriod);
	}

	else if (handle == numSamplesCharHandle + 1) {
		numberOfSamples = att_data[0];

		WriteNS_IntFlash(numberOfSamples);
		SEGGER_RTT_printf(0, "Number of samples set: %d\n", numberOfSamples);
	}

	else if (handle == modeCharHandle + 1) {
		cushionMode = att_data[0];
		WriteMode_IntFlash(cushionMode);
		SEGGER_RTT_printf(0, "Mode set: %d\n", cushionMode);

		if (cushionMode == 0) { // Autonomous (patient) mode
			// Readjustment timer will have started since it is connected
			DeInitialize();
		} else if (cushionMode == 1) { // Non-autonomous (clinician / developer) mode
			ReInitialize();
			LEDs_Set_Blue(TRUE);
			LEDs_Set_Red(FALSE);
		}
	}

	else if (handle == deviceInfoCharHandle+1) {
		WriteDeviceNum_IntFlash(att_data);
		SEGGER_RTT_printf(0, "Set device ID: %d %d %d\n", att_data[0], att_data[1], att_data[2]);
	}

	else if (handle == dateTimeCharHandle+1) {
		year 	= att_data[0]; // Year from 2000
		month 	= att_data[1];
		day 	= att_data[2];
		hour 	= att_data[3];
		minute 	= att_data[4];
		second = 0; // Start it exactly on the minute
		startTimeTicks = HAL_VTimerGetCurrentTime_sysT32();

		WriteHeader_Memory();


		SEGGER_RTT_printf(0, "Date set: %d/%d/%d %d:%d\n", day, month, year+2000, hour, minute);

		//BLE_Misc_Test();
	}
}
