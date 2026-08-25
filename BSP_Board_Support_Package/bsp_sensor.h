#ifndef BSP_SENSOR_H
#define BSP_SENSOR_H
#include <stdint.h>

void BSP_Sensors_Init(void);
void BSP_Sensor_SyncAdcFrame(void);
float BSP_Sensor_GetDistance(void);
float BSP_Sensor_GetSmoke(void);
uint8_t BSP_Sensor_ReadDHT11(uint8_t *temp, uint8_t *humi);
float BSP_Sensor_GetLight(void);
void BSP_Sensors_Sleep(void);
void BSP_Sensors_Wakeup(void);

#endif
