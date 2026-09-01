#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "can.h"

#define CAN_MX_REGISTER_CNT 16
#define MX_CAN_FILTER_CNT (2 * 14)
#define DEVICE_CAN_CNT 2

extern volatile uint32_t can_debug_rx_total;
extern volatile uint32_t can_debug_match_count;
extern volatile uint32_t can_debug_no_match_count;
extern volatile uint32_t can_debug_last_bus;
extern volatile uint32_t can_debug_last_fifo;
extern volatile uint32_t can_debug_last_std_id;
extern volatile uint32_t can_debug_last_dlc;

extern volatile uint32_t can_debug_watch_id;
extern volatile uint32_t can_debug_watch_count;
extern volatile uint32_t can_debug_watch_bus;
extern volatile uint32_t can_debug_watch_fifo;
extern volatile uint32_t can_debug_watch_dlc;
extern volatile uint32_t can_debug_watch_registered_count;
extern volatile uint32_t can_debug_watch_registered_bus_mask;

#pragma pack(1)
typedef struct _
{
    CAN_HandleTypeDef *can_handle;
    CAN_TxHeaderTypeDef txconf;
    uint32_t tx_id;
    uint32_t tx_mailbox;
    uint8_t tx_buff[8];
    uint8_t rx_buff[8];
    uint32_t rx_id;
    uint8_t rx_len;
    void (*can_module_callback)(struct _ *);
    void *id;
} CANInstance;
#pragma pack()

typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
} CAN_Init_Config_s;

CANInstance *CANRegister(CAN_Init_Config_s *config);
void CANSetDLC(CANInstance *_instance, uint8_t length);
uint8_t CANTransmit(CANInstance *_instance, float timeout);
void CANEnableDebugPassthrough(CAN_HandleTypeDef *hcan);
void CANPrintDebugUART(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);

#endif
