/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:29:49
 */
#include "super_cap.h"
#include "memory.h"
#include "stdlib.h"
#include "FreeRTOS.h"

static SuperCapInstance *super_cap_instance = NULL; // 可以由app保存此指针

static void SuperCapRxCallback(CANInstance *_instance)
{
    SuperCap_Msg_s *Msg;

    if (_instance == NULL || super_cap_instance == NULL)
        return;
    if (_instance->rx_len < 6)
        return;

    Msg = &super_cap_instance->cap_msg;
    Msg->vol = (uint16_t)(_instance->rx_buff[0] << 8 | _instance->rx_buff[1]);
    Msg->current = (uint16_t)(_instance->rx_buff[2] << 8 | _instance->rx_buff[3]);
    Msg->power = (uint16_t)(_instance->rx_buff[4] << 8 | _instance->rx_buff[5]);
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    if (supercap_config == NULL)
        return NULL;

    super_cap_instance = (SuperCapInstance *)pvPortMalloc(sizeof(SuperCapInstance));
    if (super_cap_instance == NULL)
        return NULL;

    memset(super_cap_instance, 0, sizeof(SuperCapInstance));
    
    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance->can_ins = CANRegister(&supercap_config->can_config);
    return super_cap_instance;
}

void SuperCapSend(SuperCapInstance *instance, uint8_t *data)
{
    if (instance == NULL || instance->can_ins == NULL || data == NULL)
        return;

    memcpy(instance->can_ins->tx_buff, data, 8);
    CANTransmit(instance->can_ins, 1);
}

SuperCap_Msg_s SuperCapGet(SuperCapInstance *instance)
{
    SuperCap_Msg_s empty = {0};

    if (instance == NULL)
        return empty;

    return instance->cap_msg;
}