/**
 ********************************************************************
 * @file    sensor_simulation.c
 * @brief   Simulate oxygen and temperature sensor readings and send them
 *          to the remote controller via data transmission APIs.
 *********************************************************************
 */

#include "sensor_simulation.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "utils/util_misc.h"
#include "dji_low_speed_data_channel.h"
#include "dji_high_speed_data_channel.h"
#include "dji_aircraft_info.h"
#include "widget_interaction_test/test_widget_interaction.h"

/* Private constants ---------------------------------------------------------*/
#define SENSOR_SIM_TASK_FREQ_MS        (2000)
#define SENSOR_SIM_TASK_STACK_SIZE     (2048)

/* Private variables ---------------------------------------------------------*/
static T_DjiTaskHandle s_sensorSimThread = 0;

/* Private functions ---------------------------------------------------------*/
static void *SensorSim_Task(void *arg);

T_DjiReturnCode DjiTest_SensorSimStartService(void)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    if (osalHandler == NULL) {
        USER_LOG_ERROR("osal handler is null");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    if (osalHandler->TaskCreate("sensor_sim_task", SensorSim_Task,
                                SENSOR_SIM_TASK_STACK_SIZE, NULL, &s_sensorSimThread) !=
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("sensor sim task create error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_SensorSimStopService(void)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    if (osalHandler == NULL) {
        USER_LOG_ERROR("osal handler is null");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    if (osalHandler->TaskDestroy(s_sensorSimThread) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("sensor sim task destroy error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void *SensorSim_Task(void *arg)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode djiStat;
    E_DjiChannelAddress channelAddress;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;
    USER_UTIL_UNUSED(arg);

    if (DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("get aircraft base info fail in sensor sim");
    }

    while (1) {
        osalHandler->TaskSleepMs(SENSOR_SIM_TASK_FREQ_MS);

        /* Simulate sensor values */
        uint32_t t = (uint32_t)osalHandler->GetTimeMs();
        float temperature = 20.0f + (float)(DjiPlatform_GetOsalHandler()->GetRandomNum() % 100) / 10.0f; /* 20.0-30.0 */
        float oxygen = 90.0f + (float)(DjiPlatform_GetOsalHandler()->GetRandomNum() % 100) / 10.0f; /* 90.0-100.0 */

        char payload[128];
        int len = snprintf(payload, sizeof(payload), "{\"type\":\"sensor\",\"temp\":%.1f,\"oxi\":%.1f,\"ts\":%u}",
                       temperature, oxygen, t);

        /* Send to mobile/RC first */
        channelAddress = DJI_CHANNEL_ADDRESS_MASTER_RC_APP;
        djiStat = DjiLowSpeedDataChannel_SendData(channelAddress, (const uint8_t *)payload, (uint16_t)len);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("sensor sim: send data to mobile error.");
        } else {
            USER_LOG_DEBUG("sensor sim: sent %s", payload);
            DjiTest_WidgetLogAppend("SensorSim send: %s", payload);
        }

        /* Also send to extension/payload ports depending on mount, similar to test_data_transmission */
        if (aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1 ||
            aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_PAYLOAD_PORT_NO2 ||
            aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_PAYLOAD_PORT_NO3) {
            channelAddress = DJI_CHANNEL_ADDRESS_EXTENSION_PORT;
            djiStat = DjiLowSpeedDataChannel_SendData(channelAddress, (const uint8_t *)payload, (uint16_t)len);
            if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("sensor sim: send data to extension port error.");
            }
        } else if (aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_EXTENSION_PORT) {
            channelAddress = DJI_CHANNEL_ADDRESS_PAYLOAD_PORT_NO1;
            djiStat = DjiLowSpeedDataChannel_SendData(channelAddress, (const uint8_t *)payload, (uint16_t)len);
            if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("sensor sim: send data to payload port error.");
            }
        }
    }

    return NULL;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
