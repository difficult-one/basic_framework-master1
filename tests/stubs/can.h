#pragma once
#include "main.h"
typedef struct { int unused; } CAN_HandleTypeDef;
typedef struct {
    uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime;
} CAN_TxHeaderTypeDef;
extern CAN_HandleTypeDef hcan1, hcan2;
