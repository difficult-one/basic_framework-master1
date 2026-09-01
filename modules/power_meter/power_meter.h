#ifndef POWER_METER_H
#define POWER_METER_H

#include "bsp_can.h"
#include "daemon.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PowerMeter_Data
{
    int16_t shunt_mv;
    int16_t bus_mv;
    int16_t current_ma;
    int16_t power_centiw;
    float shunt_voltage_v;
    float bus_voltage_v;
    float current_a;
    float power_w;
} PowerMeter_Data_s;

typedef struct
{
    PowerMeter_Data_s data;
    uint32_t sequence;
    uint32_t sample_ms;
    uint8_t online;
} PowerMeter_Snapshot_s;

typedef struct PowerMeter_Instance
{
    CANInstance *can_ins;
    DaemonInstance *daemon;
    PowerMeter_Data_s data;
    uint32_t rx_count;
    uint32_t last_rx_ms;
    uint32_t timeout_ms;
    uint8_t has_sample;
    uint8_t rx_raw[8];
    uint8_t rx_len;
    uint8_t rx_bus;
} PowerMeterInstance;

typedef struct PowerMeter_Init_Config
{
    CAN_Init_Config_s can_config;
    uint16_t daemon_count;
    uint32_t timeout_ms; // 0 selects 100ms; independent of the slower daemon
} PowerMeter_Init_Config_s;

extern volatile PowerMeter_Data_s power_meter_debug_data;
extern volatile uint32_t power_meter_debug_callback_count;
extern volatile uint32_t power_meter_debug_short_frame_count;
extern volatile uint32_t power_meter_debug_rx_bus;
extern volatile uint32_t power_meter_debug_rx_len;
extern volatile uint8_t power_meter_debug_raw[8];
extern volatile uint32_t power_meter_debug_can1_rx_count;
extern volatile uint32_t power_meter_debug_can2_rx_count;
extern volatile uint32_t power_meter_debug_rx_count;
extern volatile uint32_t power_meter_debug_rx_id;

PowerMeterInstance *PowerMeterInit(PowerMeter_Init_Config_s *config);
PowerMeter_Data_s PowerMeterGet(PowerMeterInstance *instance);
/* Task context only; data and original receive time are copied atomically. */
PowerMeter_Snapshot_s PowerMeterGetSnapshot(PowerMeterInstance *instance);
uint8_t PowerMeterIsOnline(PowerMeterInstance *instance);
void PowerMeterPrintCanFrameUART(PowerMeterInstance *instance, UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
