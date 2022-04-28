/*******************************************************************************
 * File Name          : sensor.c
 * Author             : Brendan Coutts / Ibrahim Elmallah
 * Date               : August-2021
 * Description        : Sensor init and sensor state machines
 *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "ble_const.h"
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "BlueNRG1_adc.h"
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Config.h"
#include "OTA_btl.h"   
#include "clock.h"
#include "sleep.h"
#include "gatt_db.h"
#include "gp_timer.h"

#include "defines.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
volatile uint8_t set_connectable = 1;
uint8_t connInfo[20];
int connected = FALSE;
uint32_t end;

//uint32_t address = MEMORY_START_ADDRESS;//initial memory address where data are saved
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);
extern volatile int wakeupExpired;

/*******************************************************************************
 * Function Name  : Sensor_DeviceInit.
 * Description    : Init the device sensors.
 * Input          : None.
 * Return         : Status.
 *******************************************************************************/
uint8_t Sensor_DeviceInit()
{
	uint8_t bdaddr[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02};
	uint8_t ret;
	uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
	uint8_t device_name[] = {'C', 'u', 's', 'h', 'i', 'o', 'n', '0', '0', '0'};

	/* Set the device public address */
	ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN,
								  bdaddr);
	if(ret != BLE_STATUS_SUCCESS) return ret;

	/* Set the TX power -2 dBm */
	aci_hal_set_tx_power_level(1, 7); //5->7

	/* GATT Init */
	ret = aci_gatt_init();
	if(ret != BLE_STATUS_SUCCESS) return ret;

	/* GAP Init */
	ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, sizeof(device_name), &service_handle, &dev_name_char_handle, &appearance_char_handle);
	if(ret != BLE_STATUS_SUCCESS) return ret;

	ret = aci_gatt_update_char_value_ext(0, service_handle, dev_name_char_handle, 0,sizeof(device_name), 0, sizeof(device_name), device_name);

	if(ret != BLE_STATUS_SUCCESS) return ret;
  
	ret = aci_gap_set_authentication_requirement(BONDING,
											   MITM_PROTECTION_REQUIRED,
											   SC_IS_NOT_SUPPORTED,
											   KEYPRESS_IS_NOT_SUPPORTED,
											   7,
											   16,
											   USE_FIXED_PIN_FOR_PAIRING,
											   123456,
											   0x00);
	if(ret != BLE_STATUS_SUCCESS) return ret;


	/* Add sensor service and Characteristics */

	SEGGER_RTT_printf(0,"Set bluetooth settings\n");

	ret = Add_Sensor_Service();
	if(ret != BLE_STATUS_SUCCESS) {
		SEGGER_RTT_printf(0,"Failure in Add Sensor Service\n");
		return ret;
	}

	//BLE Stack Initialized with SUCCESS
	return BLE_STATUS_SUCCESS;
}



/*******************************************************************************
 * Function Name  : Set_DeviceConnectable.
 * Description    : Puts the device in connectable mode.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Set_DeviceConnectable(void)
{  
	uint8_t ret;
	uint8_t local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME, 'C', 'u', 's', 'h', 'i', 'o', 'n', '0', '0', '0'};

	//Set General Discoverable Mode
	hci_le_set_scan_response_data(0,NULL);

	uint8_t deviceNum[3];
	BOOL isModified = ReadDeviceNum_IntFlash(deviceNum);
	if (isModified){
		//so it has been modified and name is valid
		local_name[10] = deviceNum[2];
		local_name[9] = deviceNum[1];
		local_name[8] = deviceNum[0];
	}

	ret = aci_gap_set_discoverable(ADV_IND,
			0x0020,0x0030,RANDOM_ADDR, NO_WHITE_LIST_USE,sizeof(local_name), local_name, 0, NULL,0x0010, 0x0C80); //Changed
	if(ret != BLE_STATUS_SUCCESS) {
		SEGGER_RTT_printf(0,"In Set_DeviceConnectable. Error: %04x\n",ret);
		while (1);
	}
	SEGGER_RTT_printf(0, "Device has been set as connectable.\n");
}

/********************************************************************************
 * Function Name  : APP_Tick.
 * Description    : Sensor Demo state machine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void APP_Tick(void)
{
	/* Make the device discoverable */
	if(set_connectable) {
		Set_DeviceConnectable();
		set_connectable = FALSE;
	}

	/*  Update sensor value */
	if (connected){
		tBleStatus ret = BLE_STATUS_SUCCESS;

		// Send data to master if notification is enabled
		if (isDownloadNotificationEnabled) {
			uint8_t data[216];
			uint8_t index = 0;
			for (uint8_t a = 0; a<216; a++){
				data[a] = SPI_Read_Memory(address);
				address += 1;
				if (address > currentAddress){ // Reached the end. No need to continue read.
					isDownloadNotificationEnabled = FALSE;
					index = a+1;
					break;
				}
			}

			if (index == 0){
				//SEGGER_RTT_printf(0, "1 - Data: %d - %d\n", data[0], data[215]);
				for (uint8_t i=0; i<216; i++) SEGGER_RTT_printf(0, "Data: %x\n", data[i]); // causes disconnection
				SEGGER_RTT_printf(0, "----------------------------\n");
				ret = Notify_Download_Val((uint8_t*)&data,216);
				if (ret != BLE_STATUS_SUCCESS) address -= 216; // Resend data
			}
			else {
				ret = Notify_Download_Val((uint8_t*)&data,index);
				//SEGGER_RTT_printf(0, "2 - Data: %d - %d\n", data[0], data[index-1]);
				for (uint8_t i=0; i<index; i++) SEGGER_RTT_printf(0, "Data: %x\n", data[i]);
				SEGGER_RTT_printf(0, "----------------------------\n");
				if (ret != BLE_STATUS_SUCCESS){
					address -= index; // Resend data
					isDownloadNotificationEnabled = TRUE;
				}
			}
		}

		if (ret != BLE_STATUS_SUCCESS)
		{
			Clock_Wait(10);
		}
	}
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates that a new connection has been created.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{
  	SEGGER_RTT_printf(0, "Connection has been made\n");
	connected = TRUE;
	connection_handle = Connection_Handle;
	aci_gap_set_non_discoverable();

	//if (cushionMode == 1) {
	// Reinitialize, to accept data. If android sends mode = 0, will go back to patient mode via gatt_db function
		ReInitialize();
		LEDs_Set_Blue(TRUE);
		LEDs_Set_Red(FALSE);
		//Start timer 2: Start pressure adjustment counter after a connection has been made
		//HAL_VTimerStart_ms(2, samplingPeriod*60*1000); //TODO: Turned off for testing purposes - turn on
	//}
}/* end hci_le_connection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event occurs when a connection is terminated.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
	aci_gap_terminate(connection_handle,0x13);
	connected = FALSE;

	connection_handle = 0;

	/* Make the device connectable again. */
	if (!longTermMode){
		set_connectable = TRUE;
	}

	SEGGER_RTT_printf(0, "The device has been disconnected\n");

	// Start timer for disconnectedMode if in Admin mode
	// No need in cushion mode as it will already be in a low power setting
	if (cushionMode == 1) {
		LEDs_Set_Blue(TRUE); //TODO: Does not work - try deleting prints
		LEDs_Set_Red(TRUE);
		HAL_VTimerStart_ms(1, 60000); // Start disconnection timer
		HAL_VTimer_Stop(2); // Stop readjustment timer
	}

}/* end hci_disconnection_complete_event() */

/*******************************************************************************
 * Function Name  : aci_gatt_read_permit_req_event.
 * Description    : This event is given when a read request is received
 *                  by the server from the client.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_read_permit_req_event(uint16_t Connection_Handle,
                                    uint16_t Attribute_Handle,
                                    uint16_t Offset)
{
	//SEGGER_RTT_printf(0,"read received. Attribute Handle: %x, Offset: %x\n", Attribute_Handle, Offset);
	Read_Request_CB(Attribute_Handle);
}

/*******************************************************************************
 * Function Name  : HAL_VTimerTimeoutCallback.
 * Description    : This function will be called on the expiry of 
 *                  a one-shot virtual timer.
 * Input          : See file bluenrg1_stack.h
 * Output         : See file bluenrg1_stack.h
 * Return         : See file bluenrg1_stack.h
 *******************************************************************************/
void HAL_VTimerTimeoutCallback(uint8_t timerNum)
{
	if (timerNum == 0) { // Timer to keep track of low power mode
		//restart timer
		int ret = HAL_VTimerStart_ms(0,1000);
		if (ret != BLE_STATUS_SUCCESS)
					return;

		// Pressure value notification
		if (isPressureReaderNotificationEnabled && connected && !longTermMode){
			Notify_Pressure();
		}

		// Timer for testing purposes: Reads out ADC value every second
		//Read_Pressure();
	}

	if (timerNum == 1){ // Timer to keep track of disconnected mode
		DeInitialize();
	}

	if (timerNum == 2) {
		SEGGER_RTT_printf(0, "Beginning pressure adjustment\n");

		pressureSTATE state_pressure[8];
		uint8_t currentBag_i;
		uint8_t currentBag_j;
		uint8_t numBags = 5; // TODO: Change
		uint8_t bagOrder [8] = {2, 3, 5, 7, 8, 1, 4, 6}; // Order to release bags in // TODO: Change
		//uint8_t deltaPressure = abs(upperBoundPressure - lowerBoundPressure)*0.2;

		// If in autonomous mode, wake up peripherals
		if (cushionMode == 0) {
			ReInitialize();
		}

		// Read sensor values
		Read_ADC();

		// Write pressure values to memory
		WritePressures_Memory();

		for (int i=0; i<numBags; i++){
			currentBag_i = bagOrder[i];
			// Check if pressure is in acceptable range
			if (CurrentPressures[currentBag_i- 1] < lowerBoundPressure){
				logCounter[currentBag_i - 1]--;
			} else if (CurrentPressures[currentBag_i - 1] > upperBoundPressure) {
				logCounter[currentBag_i - 1]++;
			} else {
				logCounter[currentBag_i - 1] = 0;
			}

			SEGGER_RTT_printf(0, "Pressure count for bag %d is %d\n", currentBag_i - 1, logCounter[currentBag_i - 1]);
		}

		// Redistribute pressure
		SEGGER_RTT_printf(0, "Redistributing pressure between bags now\n");
		for (uint8_t i=0; i<numBags; i++){
			currentBag_i = bagOrder[i];

			if ((-1*logCounter[currentBag_i - 1]) >= (int)numberOfSamples) { // If a bag's pressure is too low
				uint8_t highBagIndex = currentBag_i - 1;
				uint16_t highBagValue = CurrentPressures[currentBag_i - 1];

				// Look for a bag with too high a pressure, and redistribute
				for (uint8_t j=i; j<numBags; j++) {
					currentBag_j = bagOrder[j];

					if (logCounter[currentBag_j - 1] >= (int)numberOfSamples) {
						if (CurrentPressures[currentBag_j - 1] > highBagValue) { // Get highest pressure bag available
							highBagIndex = currentBag_j - 1;
							highBagValue = CurrentPressures[currentBag_j - 1];
						}
					}
				}

				if (highBagIndex != currentBag_i - 1) Redistribute_Pressure(currentBag_i, highBagIndex+1, targetPressure); // Only redistribute pressure if an appropriate bag is found

				// If pressure has been fixed, set count back to 0
				Read_ADC();
				if ((CurrentPressures[currentBag_i - 1] > lowerBoundPressure) && (CurrentPressures[currentBag_i - 1] < upperBoundPressure)) {
					logCounter[currentBag_i - 1] = 0;
					SEGGER_RTT_printf(0, "Pressure in cushion %d is now ok\n", currentBag_i - 1);
				} // Pressure in other cushion may have been also unintentionally fixed
				if ((CurrentPressures[highBagIndex] > lowerBoundPressure) && (CurrentPressures[highBagIndex] < upperBoundPressure)) {
					logCounter[highBagIndex] = 0;
					SEGGER_RTT_printf(0, "Pressure in cushion %d has now been fixed in the process\n", highBagIndex);
				}
			}

			else if (logCounter[currentBag_i - 1] >= (int)numberOfSamples) { // If a bag's pressure is too high
				uint8_t lowBagIndex = currentBag_i - 1;
				uint16_t lowBagValue = CurrentPressures[currentBag_i - 1];

				// Look for a bag with too low a pressure, and redistribute
				for (uint8_t j=i; j<numBags; j++) {
					currentBag_j = bagOrder[j];

					if ((-1*logCounter[currentBag_j - 1]) >= (int)numberOfSamples) {
						if (CurrentPressures[currentBag_j - 1] < lowBagValue) {  // Get lowest pressure bag available
							lowBagIndex = currentBag_j - 1;
							lowBagValue = CurrentPressures[currentBag_j - 1];
						}
					}
				}

				if (lowBagIndex != currentBag_i - 1) Redistribute_Pressure(currentBag_i, lowBagIndex+1, targetPressure); // Only redistribute pressure if an appropriate bag is found

				// If pressure has been fixed, set count back to 0
				Read_ADC();
				if ((CurrentPressures[currentBag_i - 1] > lowerBoundPressure) && (CurrentPressures[currentBag_i - 1] < upperBoundPressure)) {
					logCounter[currentBag_i - 1] = 0;
					SEGGER_RTT_printf(0, "Pressure in cushion %d is now ok\n", currentBag_i - 1);
				} // Pressure in other cushion may have been also unintentionally fixed
				if ((CurrentPressures[lowBagIndex] > lowerBoundPressure) && (CurrentPressures[lowBagIndex] < upperBoundPressure)) {
					logCounter[lowBagIndex] = 0;
					SEGGER_RTT_printf(0, "Pressure in cushion %d has now been fixed in the process\n", lowBagIndex);
				}
			}
		}

		SEGGER_RTT_printf(0, "Readjusting pressure of individual bags now\n");
		for (uint8_t j=0; j<4; j++) {
			SEGGER_RTT_printf(0, "Readjusting pressure: Iteration %d\n", j);

			for (uint8_t i=0; i<numBags; i++) {
				currentBag_i = bagOrder[i];

				if ((CurrentPressures[currentBag_i - 1] < lowerBoundPressure) && (((-1*logCounter[currentBag_i - 1]) >= (int)numberOfSamples) || (logCounter[currentBag_i - 1] >= (int)numberOfSamples))) { //If a bag's pressure is too low
					Pressurize_Air_Bag(currentBag_i, targetPressure, ((float)multiplier_increase[currentBag_i - 1])/100);
					state_pressure[currentBag_i - 1] = INCREASE;
				} else if ((CurrentPressures[currentBag_i - 1] > upperBoundPressure) && (((-1*logCounter[currentBag_i - 1]) >= (int)numberOfSamples) || (logCounter[currentBag_i - 1] >= (int)numberOfSamples))) { //If a bag's pressure is too high
					Depressurize_Air_Bag(currentBag_i, targetPressure, ((float)multiplier_decrease[currentBag_i - 1])/100);
					state_pressure[currentBag_i - 1] = DECREASE;
				}
			}

			Read_ADC();

			// Adjust multipliers
			SEGGER_RTT_printf(0, "Current pressures: ");
			for (uint8_t i=0; i<numBags; i++) {
				currentBag_i = bagOrder[i];

				if ((logCounter[currentBag_i - 1] >= (int)numberOfSamples) || ((-1*logCounter[currentBag_i - 1]) >= (int)numberOfSamples)) {
					if (CurrentPressures[currentBag_i - 1] < lowerBoundPressure) {
						if (state_pressure[currentBag_i - 1] == INCREASE && multiplier_increase[currentBag_i - 1] < 250) multiplier_increase[currentBag_i - 1] += 10; // Increased pressure and still too low -> Raise multiplier
						if (state_pressure[currentBag_i - 1] == DECREASE && multiplier_decrease[currentBag_i - 1] > 10) multiplier_decrease[currentBag_i - 1] -= 10; // Decreased pressure and now its too low -> Lower multiplier
					} else if (CurrentPressures[currentBag_i - 1] > upperBoundPressure) {
						if (state_pressure[currentBag_i - 1] == INCREASE && multiplier_increase[currentBag_i - 1] > 10) multiplier_increase[currentBag_i - 1] -= 10; // Increased pressure and now its too high -> Lower multiplier
						if (state_pressure[currentBag_i - 1] == DECREASE && multiplier_decrease[currentBag_i - 1] < 250) multiplier_decrease[currentBag_i - 1] += 10; // Decreased pressure and still too high -> Raise multiplier
					} else {
						logCounter[currentBag_i - 1] = 0;
					}
				} SEGGER_RTT_printf(0, "%d mmHg, ", CurrentPressures[currentBag_i - 1]);
			} SEGGER_RTT_printf(0, "\n");

			BOOL readjustmentDone = FALSE;
			for (uint8_t i=0; i<numBags; i++){
				currentBag_i = bagOrder[i];

				if ((logCounter[currentBag_i - 1] >= (int)numberOfSamples) || ((-1*logCounter[currentBag_i - 1]) >= (int)numberOfSamples)){
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

		// Restart counter
		int ret = HAL_VTimerStart_ms(2, samplingPeriod*60*1000);
		if (ret != BLE_STATUS_SUCCESS)
			return;

		// If in autonomous mode, put components back to sleep
		if (cushionMode == 0) {
			DeInitialize();
		}
	}

	if (timerNum == 3) { // Battery monitoring timer
		// If in autonomous mode, wake up peripherals
		if (cushionMode == 0) {
			ReInitialize();
		}

		int ret = HAL_VTimerStart_ms(3, samplingPeriod*60*1000);
		if (ret != BLE_STATUS_SUCCESS)
			return;

		SEGGER_RTT_printf(0, "Periodically reading battery\n");

		BatteryReading = ADC_Value(ADC_Input_AdcPin2);
		if (BatteryReading < 6600) { // Battery below 6.6 V
			SEGGER_RTT_printf(0, "Battery level too low - shutting off\n");
			for (int i=0; i<10; i++) // Flash RED Led 10 times
				LEDs_Flash(FALSE, TRUE);
			HAL_VTimer_Stop(0); // Stop notification timer
			HAL_VTimer_Stop(1); // Stop connection monitoring timer
			HAL_VTimer_Stop(2); // Stop readjustment timer
			HAL_VTimer_Stop(3); // Stop this timer

			SPI_DeepPowerDown_Memory();
			GPIO_ResetBits(RED_LED|BLUE_LED|PVControl|ValveControl|Valve1|Valve2|Valve3|Valve4|Valve5|Valve6|Valve7|Valve8|SensorControl);
			SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_SPI, DISABLE);
			SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_ADC, DISABLE);

			longTermMode = TRUE;
		}

		if ((cushionMode == 0) && (longTermMode != TRUE)){ // If in patient mode, and not already powered down
			DeInitialize();
		}
	}
}

/*******************************************************************************
 * Function Name  : aci_gatt_attribute_modified_event.
 * Description    : This event occurs when an attribute is modified.
 * 				  : aci_gatt_notification_event and aci_gatt_write_permit_req_event
 * 				  	will not be triggered if send by android, which is stupid
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
	//SEGGER_RTT_printf(0,"write req received.Connection_Handle is:%04x, Attr_Handle is:%04x, datalength is:%04x\n",Connection_Handle,Attr_Handle,Attr_Data_Length);
	for(int i=0;i<Attr_Data_Length;i++){
		//SEGGER_RTT_printf(0,"attr_data[%d]: %04x\n",i,Attr_Data[i]);
	}
	Write_Request_CB(Attr_Handle,Attr_Data_Length,Attr_Data);
}


void aci_hal_end_of_radio_activity_event(uint8_t Last_State,
                                         uint8_t Next_State,
                                         uint32_t Next_State_SysTime)
{

}

/**
  * @brief This event is generated in response to an Exchange MTU request. See
@ref aci_gatt_exchange_config.
  * @param Connection_Handle Connection handle related to the response
  * @param Server_RX_MTU ATT_MTU value agreed between server and client
  * @retval None
*/
void aci_att_exchange_mtu_resp_event(uint16_t Connection_Handle,
                                     uint16_t Server_RX_MTU)
{
	SEGGER_RTT_printf(0,"mtu now.%d\n",Server_RX_MTU);
}


