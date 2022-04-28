

#ifndef _SENSOR_H_
#define _SENSOR_H_
uint8_t Sensor_DeviceInit(void);
void APP_Tick(void);
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);
extern uint8_t Application_Max_Attribute_Records[]; 

#endif /* _SENSOR_H_ */
