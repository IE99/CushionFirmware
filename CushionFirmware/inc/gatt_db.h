#ifndef _GATT_DB_H_
#define _GATT_DB_H_


/** 
 * @brief Number of application services
 */
#define MEMORY_START_ADDRESS  			0x10053020
#define MEMORY_END_ADDRESS  			0x1007FFFF

tBleStatus Add_Sensor_Service(void);
void Read_Request_CB(uint16_t handle);
void Write_Request_CB(uint16_t handle, uint16_t data_length, uint8_t *att_data);

tBleStatus Read_Batt();
tBleStatus Read_LED();
tBleStatus Read_SampleRate();
tBleStatus Read_DeviceTime();
tBleStatus Read_Pressure();
tBleStatus Notify_Pressure();
tBleStatus Notify_Download_Val(uint8_t* data,uint8_t size);
tBleStatus Read_Control_SmallValves();
tBleStatus Read_Control_BigValves();
tBleStatus Read_Control_Pump();
tBleStatus Read_Device_Time();

#endif /* _GATT_DB_H_ */
