#include "bsp_can.h"
#include "main.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "stdio.h"

static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx;

volatile uint32_t can_debug_rx_total;
volatile uint32_t can_debug_match_count;
volatile uint32_t can_debug_no_match_count;
volatile uint32_t can_debug_last_bus;
volatile uint32_t can_debug_last_fifo;
volatile uint32_t can_debug_last_std_id;
volatile uint32_t can_debug_last_dlc;
volatile uint32_t can_debug_watch_id = 0x605;
volatile uint32_t can_debug_watch_count;
volatile uint32_t can_debug_watch_bus;
volatile uint32_t can_debug_watch_fifo;
volatile uint32_t can_debug_watch_dlc;
volatile uint32_t can_debug_watch_registered_count;
volatile uint32_t can_debug_watch_registered_bus_mask;
volatile uint32_t can_debug_passthrough_bus_mask;

static void CANAddFilter(CANInstance *_instance)
{
    CAN_FilterTypeDef can_filter_conf = {0};
    static uint8_t can1_filter_idx = 0;
    static uint8_t can2_filter_idx = 14;

    can_filter_conf.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_conf.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_conf.FilterFIFOAssignment =
        (_instance->rx_id & 1U) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    can_filter_conf.SlaveStartFilterBank = 14;
    can_filter_conf.FilterIdHigh = _instance->rx_id << 5;
    can_filter_conf.FilterIdLow = 0;
    can_filter_conf.FilterMaskIdHigh = 0x7FF << 5;
    can_filter_conf.FilterMaskIdLow = 0;
    can_filter_conf.FilterBank =
        _instance->can_handle == &hcan1 ? can1_filter_idx++ : can2_filter_idx++;
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;

    if (HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf) != HAL_OK)
    {
        while (1)
            LOGERROR("[bsp_can] CAN filter config failed, rx id [%d]", _instance->rx_id);
    }
}

static void CANServiceInit(void)
{
    HAL_CAN_Start(&hcan1);
    /* Recovery: if booted with bus already in bus-off (no ACK nodes),
       stop+restart clears the peripheral state */
    if (__HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_BOF))
    {
        HAL_CAN_Stop(&hcan1);
        HAL_CAN_Start(&hcan1);
    }
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

    HAL_CAN_Start(&hcan2);
    if (__HAL_CAN_GET_FLAG(&hcan2, CAN_FLAG_BOF))
    {
        HAL_CAN_Stop(&hcan2);
        HAL_CAN_Start(&hcan2);
    }
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (!idx)
    {
        CANServiceInit();
        LOGINFO("[bsp_can] CAN Service Init");
    }

    if (idx >= CAN_MX_REGISTER_CNT)
    {
        while (1)
            LOGERROR("[bsp_can] CAN instance exceeded MAX num");
    }

    for (size_t i = 0; i < idx; i++)
    {
        if (can_instance[i]->rx_id == config->rx_id &&
            can_instance[i]->can_handle == config->can_handle)
        {
            while (1)
                LOGERROR("[bsp_can] CAN id crash, tx [%d] or rx [%d] already registered",
                         config->tx_id, config->rx_id);
        }
    }

    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance));
    if (instance == NULL)
    {
        while (1)
            LOGERROR("[bsp_can] CAN instance malloc failed");
    }

    memset(instance, 0, sizeof(CANInstance));

    instance->txconf.StdId = config->tx_id;
    instance->txconf.IDE = CAN_ID_STD;
    instance->txconf.RTR = CAN_RTR_DATA;
    instance->txconf.DLC = 0x08;
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    if (instance->rx_id == can_debug_watch_id)
    {
        can_debug_watch_registered_count++;
        can_debug_watch_registered_bus_mask |=
            (instance->can_handle == &hcan1) ? 0x01U : 0x02U;
    }

    CANAddFilter(instance);
    can_instance[idx++] = instance;

    return instance;
}

uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    static uint32_t busy_count;
    static volatile float wait_time __attribute__((unused));
    float dwt_start = DWT_GetTimeline_ms();

    while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0)
    {
        if (DWT_GetTimeline_ms() - dwt_start > timeout)
        {
            LOGWARNING("[bsp_can] CAN mailbox full. Cnt [%d]", busy_count);
            busy_count++;
            return 0;
        }
    }

    wait_time = DWT_GetTimeline_ms() - dwt_start;

    if (HAL_CAN_AddTxMessage(_instance->can_handle,
                             &_instance->txconf,
                             _instance->tx_buff,
                             &_instance->tx_mailbox))
    {
        LOGWARNING("[bsp_can] CAN bus busy. Cnt [%d]", busy_count);
        busy_count++;
        return 0;
    }

    return 1;
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    if (length > 8 || length == 0)
    {
        while (1)
            LOGERROR("[bsp_can] CAN DLC error");
    }

    _instance->txconf.DLC = length;
}

static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf;
    uint8_t can_rx_buff[8];

    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox))
    {
        HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff);

        can_debug_rx_total++;
        can_debug_last_bus = (_hcan == &hcan1) ? 1U : 2U;
        can_debug_last_fifo = (fifox == CAN_RX_FIFO0) ? 0U : 1U;
        can_debug_last_std_id = rxconf.StdId;
        can_debug_last_dlc = rxconf.DLC;

        if (rxconf.StdId == can_debug_watch_id)
        {
            can_debug_watch_count++;
            can_debug_watch_bus = can_debug_last_bus;
            can_debug_watch_fifo = can_debug_last_fifo;
            can_debug_watch_dlc = rxconf.DLC;
        }

        for (size_t i = 0; i < idx; ++i)
        {
            if (_hcan == can_instance[i]->can_handle &&
                rxconf.StdId == can_instance[i]->rx_id)
            {
                if (can_instance[i]->can_module_callback != NULL)
                {
                    can_debug_match_count++;
                    can_instance[i]->rx_len = rxconf.DLC;
                    memcpy(can_instance[i]->rx_buff, can_rx_buff, rxconf.DLC);
                    can_instance[i]->can_module_callback(can_instance[i]);
                }
                return;
            }
        }

        can_debug_no_match_count++;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1);
}

void CANEnableDebugPassthrough(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef can_filter_conf = {0};
    uint32_t bus_mask;

    if (hcan == NULL)
        return;

    bus_mask = (hcan == &hcan1) ? 0x01U : 0x02U;
    if (can_debug_passthrough_bus_mask & bus_mask)
        return;

    can_filter_conf.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_conf.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_conf.FilterFIFOAssignment = CAN_RX_FIFO1;
    can_filter_conf.SlaveStartFilterBank = 14;
    can_filter_conf.FilterIdHigh = 0;
    can_filter_conf.FilterIdLow = 0;
    can_filter_conf.FilterMaskIdHigh = 0;
    can_filter_conf.FilterMaskIdLow = 0;
    can_filter_conf.FilterBank = (hcan == &hcan1) ? 13U : 27U;
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;

    if (HAL_CAN_ConfigFilter(hcan, &can_filter_conf) == HAL_OK)
        can_debug_passthrough_bus_mask |= bus_mask;
}

void CANPrintDebugUART(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart)
{
    char msg[220];
    uint32_t bus;
    uint32_t state;
    uint32_t error;
    uint32_t esr;
    uint32_t msr;
    uint32_t fifo0;
    uint32_t fifo1;
    int len;

    if (hcan == NULL || huart == NULL)
        return;

    bus = (hcan == &hcan1) ? 1U : 2U;
    state = (uint32_t)HAL_CAN_GetState(hcan);
    error = HAL_CAN_GetError(hcan);
    esr = hcan->Instance->ESR;
    msr = hcan->Instance->MSR;
    fifo0 = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
    fifo1 = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1);

    len = snprintf(msg, sizeof(msg),
                   "CAN%lu dbg: state=0x%02lX err=0x%08lX ESR=0x%08lX MSR=0x%08lX "
                   "fifo=%lu/%lu rx=%lu match=%lu no_match=%lu last=CAN%lu/F%lu/id=0x%03lX/dlc=%lu "
                   "watch=0x%03lX cnt=%lu reg=%lu mask=0x%02lX pass=0x%02lX\r\n",
                   (unsigned long)bus,
                   (unsigned long)state,
                   (unsigned long)error,
                   (unsigned long)esr,
                   (unsigned long)msr,
                   (unsigned long)fifo0,
                   (unsigned long)fifo1,
                   (unsigned long)can_debug_rx_total,
                   (unsigned long)can_debug_match_count,
                   (unsigned long)can_debug_no_match_count,
                   (unsigned long)can_debug_last_bus,
                   (unsigned long)can_debug_last_fifo,
                   (unsigned long)can_debug_last_std_id,
                   (unsigned long)can_debug_last_dlc,
                   (unsigned long)can_debug_watch_id,
                   (unsigned long)can_debug_watch_count,
                   (unsigned long)can_debug_watch_registered_count,
                   (unsigned long)can_debug_watch_registered_bus_mask,
                   (unsigned long)can_debug_passthrough_bus_mask);

    if (len < 0)
        return;
    if (len > (int)sizeof(msg))
        len = (int)sizeof(msg);

    HAL_UART_Transmit(huart, (uint8_t *)msg, (uint16_t)len, 30);
}
