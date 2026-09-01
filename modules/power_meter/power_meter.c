#include "power_meter.h"
#include "memory.h"
#include "stdlib.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"

volatile PowerMeter_Data_s power_meter_debug_data;
volatile uint32_t power_meter_debug_callback_count;
volatile uint32_t power_meter_debug_short_frame_count;
volatile uint32_t power_meter_debug_rx_bus;
volatile uint32_t power_meter_debug_rx_len;
volatile uint8_t power_meter_debug_raw[8];
volatile uint32_t power_meter_debug_can1_rx_count;
volatile uint32_t power_meter_debug_can2_rx_count;
volatile uint32_t power_meter_debug_rx_count;// 记录接收到的报文数量
volatile uint32_t power_meter_debug_rx_id;// 记录接收到的报文id

static int16_t PowerMeterDecodeI16(const uint8_t *buf)
{
    return (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

static void PowerMeterLostCallback(void *owner)
{
    PowerMeterInstance *meter = (PowerMeterInstance *)owner;
    if (meter == NULL)
        return;

    // Retain the last complete sample for diagnostics. Snapshot validity expires
    // independently; clearing here could race with a newly received CAN frame.
}

static void PowerMeterRxCallback(CANInstance *_instance)
{
    PowerMeterInstance *meter;
    PowerMeter_Data_s *data;
    uint8_t *rx;
    uint8_t copy_len;
    uint8_t i;

    if (_instance == NULL)
        return;

    meter = (PowerMeterInstance *)_instance->id;
    power_meter_debug_callback_count++;
    power_meter_debug_rx_id = _instance->rx_id;
    power_meter_debug_rx_bus = (_instance->can_handle == &hcan1) ? 1U : 2U;
    power_meter_debug_rx_len = _instance->rx_len;
    copy_len = (_instance->rx_len > 8U) ? 8U : _instance->rx_len;
    for (i = 0; i < 8U; i++)
        power_meter_debug_raw[i] = (i < copy_len) ? _instance->rx_buff[i] : 0U;

    if (_instance->can_handle == &hcan1)
        power_meter_debug_can1_rx_count++;
    else
        power_meter_debug_can2_rx_count++;

    if (meter != NULL)
    {
        meter->rx_len = copy_len;
        meter->rx_bus = (uint8_t)power_meter_debug_rx_bus;
        memcpy(meter->rx_raw, _instance->rx_buff, copy_len);
        if (copy_len < 8U)
            memset(&meter->rx_raw[copy_len], 0, 8U - copy_len);
    }

    if (meter == NULL || _instance->rx_len < 8)
    {
        power_meter_debug_short_frame_count++;
        return;
    }

    data = &meter->data;
    rx = _instance->rx_buff;

    data->shunt_mv = PowerMeterDecodeI16(&rx[0]);
    data->bus_mv = PowerMeterDecodeI16(&rx[2]);
    data->current_ma = PowerMeterDecodeI16(&rx[4]);
    data->power_centiw = PowerMeterDecodeI16(&rx[6]);

    data->shunt_voltage_v = (float)data->shunt_mv * 0.001f;
    data->bus_voltage_v = (float)data->bus_mv * 0.001f;
    data->current_a = (float)data->current_ma * 0.001f;
    data->power_w = (float)data->power_centiw * 0.01f;

    meter->rx_count++;
    meter->last_rx_ms = HAL_GetTick();
    meter->has_sample = 1;
    power_meter_debug_data = *data;
    power_meter_debug_rx_count = meter->rx_count;

    if (meter->daemon != NULL)
        DaemonReload(meter->daemon);
}

PowerMeterInstance *PowerMeterInit(PowerMeter_Init_Config_s *config)
{
    PowerMeterInstance *meter = (PowerMeterInstance *)pvPortMalloc(sizeof(PowerMeterInstance));
    if (meter == NULL)
        return NULL;

    memset(meter, 0, sizeof(PowerMeterInstance));
    meter->timeout_ms = config->timeout_ms ? config->timeout_ms : 100U;

    config->can_config.id = meter;
    config->can_config.can_module_callback = PowerMeterRxCallback;
    meter->can_ins = CANRegister(&config->can_config);

    Daemon_Init_Config_s daemon_config = {
        .callback = PowerMeterLostCallback,
        .owner_id = (void *)meter,
        .reload_count = config->daemon_count,
        .init_count = config->daemon_count,
    };
    meter->daemon = DaemonRegister(&daemon_config);

    return meter;
}

PowerMeter_Data_s PowerMeterGet(PowerMeterInstance *instance)
{
    PowerMeter_Data_s empty = {0};
    PowerMeter_Snapshot_s snapshot = PowerMeterGetSnapshot(instance);
    return snapshot.online ? snapshot.data : empty;
}

uint8_t PowerMeterIsOnline(PowerMeterInstance *instance)
{
    return PowerMeterGetSnapshot(instance).online;
}

PowerMeter_Snapshot_s PowerMeterGetSnapshot(PowerMeterInstance *instance)
{
    PowerMeter_Snapshot_s snapshot = {0};
    if (instance == NULL)
        return snapshot;

    // CAN RX IRQs have FreeRTOS-compatible priority 5 in this project.
    taskENTER_CRITICAL();
    snapshot.data = instance->data;
    snapshot.sequence = instance->rx_count;
    snapshot.sample_ms = instance->last_rx_ms;
    snapshot.online = instance->has_sample &&
                      (uint32_t)(HAL_GetTick() - instance->last_rx_ms) < instance->timeout_ms;
    taskEXIT_CRITICAL();
    return snapshot;
}

void PowerMeterPrintCanFrameUART(PowerMeterInstance *instance, UART_HandleTypeDef *huart)
{
    char msg[160];
    uint8_t len;
    uint8_t i;
    int written;
    int offset;

    if (instance == NULL || huart == NULL || instance->rx_len == 0U)
        return;

    len = (instance->rx_len > 8U) ? 8U : instance->rx_len;
    written = snprintf(msg, sizeof(msg),
                       "PM CAN%u id=0x%03lX len=%u data=",
                       instance->rx_bus,
                       (unsigned long)instance->can_ins->rx_id,
                       len);
    if (written < 0)
        return;

    offset = written;
    if (offset >= (int)sizeof(msg))
        offset = (int)sizeof(msg) - 1;

    for (i = 0; i < len && offset < (int)sizeof(msg) - 4; i++)
    {
        written = snprintf(&msg[offset], sizeof(msg) - (size_t)offset,
                           "%02X%s", instance->rx_raw[i], (i + 1U < len) ? " " : "");
        if (written < 0)
            return;
        offset += written;
        if (offset >= (int)sizeof(msg))
        {
            offset = (int)sizeof(msg) - 1;
            break;
        }
    }

    if (len >= 8U && offset < (int)sizeof(msg) - 2)
    {
        written = snprintf(&msg[offset], sizeof(msg) - (size_t)offset,
                           " shunt=%dmV bus=%dmV current=%dmA power=%dcW",
                           instance->data.shunt_mv,
                           instance->data.bus_mv,
                           instance->data.current_ma,
                           instance->data.power_centiw);
        if (written < 0)
            return;
        offset += written;
        if (offset >= (int)sizeof(msg))
            offset = (int)sizeof(msg) - 1;
    }

    if (offset < (int)sizeof(msg) - 2)
    {
        msg[offset++] = '\r';
        msg[offset++] = '\n';
    }
    else
    {
        msg[sizeof(msg) - 3U] = '\r';
        msg[sizeof(msg) - 2U] = '\n';
        offset = (int)sizeof(msg) - 1;
    }

    HAL_UART_Transmit(huart, (uint8_t *)msg, (uint16_t)offset, 20);
}
