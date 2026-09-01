#include "power_meter.h"
#include "rm_referee.h"
#include "crc_ref.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#undef assert
#define assert(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL: %s (line %d)\n", #condition, __LINE__); exit(1); } } while (0)

CAN_HandleTypeDef hcan1, hcan2;
static uint32_t tick_ms;
static USARTInstance uart;
uint32_t HAL_GetTick(void) { return tick_ms; }
int HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *p, uint16_t n, uint32_t t)
{ (void)h; (void)p; (void)n; (void)t; return 0; }
CANInstance *CANRegister(CAN_Init_Config_s *c)
{
    CANInstance *i = calloc(1, sizeof(*i));
    assert(i);
    i->can_handle = c->can_handle;
    i->rx_id = c->rx_id;
    i->id = c->id;
    i->can_module_callback = c->can_module_callback;
    return i;
}
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *c)
{
    DaemonInstance *i = calloc(1, sizeof(*i));
    assert(i);
    i->callback = c->callback;
    i->owner_id = c->owner_id;
    i->reload_count = c->reload_count;
    return i;
}
void DaemonReload(DaemonInstance *i) { i->temp_count = i->reload_count; }
USARTInstance *USARTRegister(USART_Init_Config_s *c)
{ uart.module_callback = c->module_callback; return &uart; }
void USARTServiceInit(USARTInstance *i) { (void)i; }
void USARTSend(USARTInstance *i, uint8_t *p, uint16_t n, USART_TRANSFER_MODE mode)
{ (void)i; (void)p; (void)n; (void)mode; }

static void receive_meter(PowerMeterInstance *m, uint8_t length)
{
    // 1mV shunt, 24V bus, 2A, 48W in big-endian signed wire units.
    const uint8_t frame[] = {0, 1, 0x5d, 0xc0, 7, 0xd0, 0x12, 0xc0};
    memcpy(m->can_ins->rx_buff, frame, sizeof(frame));
    m->can_ins->rx_len = length;
    m->can_ins->can_module_callback(m->can_ins);
}

static void referee_frame(uint16_t command, const void *payload, uint16_t length)
{
    memset(uart.recv_buff, 0, sizeof(uart.recv_buff));
    uart.recv_buff[0] = 0xa5;
    uart.recv_buff[1] = (uint8_t)length;
    uart.recv_buff[2] = (uint8_t)(length >> 8);
    uart.recv_buff[5] = (uint8_t)command;
    uart.recv_buff[6] = (uint8_t)(command >> 8);
    memcpy(uart.recv_buff + 7, payload, length);
    Append_CRC8_Check_Sum(uart.recv_buff, 5);
    Append_CRC16_Check_Sum(uart.recv_buff, length + 9);
    uart.recv_len = length + 9;
}

int main(void)
{
    PowerMeter_Init_Config_s config = {0};
    config.can_config.can_handle = &hcan1;
    config.can_config.rx_id = 0x605;
    config.daemon_count = 100;
    config.timeout_ms = 100;
    PowerMeterInstance *m = PowerMeterInit(&config);
    assert(m && !PowerMeterGetSnapshot(m).online);
    assert(!PowerMeterGetSnapshot(NULL).online);
    tick_ms = 10;
    receive_meter(m, 8);
    PowerMeter_Snapshot_s sample = PowerMeterGetSnapshot(m);
    assert(sample.online && sample.sequence == 1 && sample.sample_ms == 10);
    assert(fabsf(sample.data.power_w - 48) < 0.001f);
    assert(fabsf(sample.data.bus_voltage_v - 24) < 0.001f);
    tick_ms = 109;
    receive_meter(m, 7);
    sample = PowerMeterGetSnapshot(m);
    assert(sample.online && sample.sequence == 1 && sample.sample_ms == 10);
    tick_ms = 110;
    assert(!PowerMeterGetSnapshot(m).online); // short frame did not extend validity
    assert(PowerMeterGet(m).power_w == 0);
    tick_ms = UINT32_MAX - 10;
    receive_meter(m, 8);
    tick_ms = 5;
    assert(PowerMeterGetSnapshot(m).online);
    m->rx_count = UINT32_MAX;
    receive_meter(m, 8);
    assert(PowerMeterGetSnapshot(m).online); // frame counter rollover is not startup
    memset(m->can_ins->rx_buff + 6, 0, 2);
    m->can_ins->can_module_callback(m->can_ins);
    assert(PowerMeterGetSnapshot(m).online && PowerMeterGet(m).power_w == 0);
    m->daemon->callback(m); // stale daemon callback cannot corrupt a new sample
    assert(PowerMeterGetSnapshot(m).online && PowerMeterGet(m).bus_voltage_v > 23.9f);

    RefereeInit(NULL);
    assert(!RefereeGetPowerSnapshot().power_received);
    ext_power_heat_data_t heat = {0};
    tick_ms = 100;
    referee_frame(ID_power_heat_data, &heat, sizeof(heat));
    uart.module_callback();
    RefereePowerSnapshot ref = RefereeGetPowerSnapshot();
    assert(ref.power_received && ref.power_sequence == 1 && ref.power_sample_ms == 100);
    assert(ref.power_w == 0 && ref.buffer_energy_j == 0);
    tick_ms = 200;
    heat.chassis_power = 50;
    referee_frame(ID_power_heat_data, &heat, sizeof(heat));
    uart.recv_buff[8] ^= 1; // bad CRC cannot refresh power time
    uart.module_callback();
    assert(RefereeGetPowerSnapshot().power_sample_ms == 100);
    referee_frame(ID_power_heat_data, &heat, sizeof(heat) - 1);
    uart.module_callback();
    assert(RefereeGetPowerSnapshot().power_sequence == 1);
    ext_game_robot_state_t robot = {0};
    robot.chassis_power_limit = 40;
    referee_frame(ID_game_robot_state, &robot, sizeof(robot));
    uart.module_callback();
    ref = RefereeGetPowerSnapshot();
    assert(ref.limit_received && ref.power_limit_w == 40 && ref.limit_sample_ms == 200);
    assert(ref.power_sample_ms == 100); // other referee traffic cannot refresh old power
    referee_frame(ID_power_heat_data, &heat, sizeof(heat));
    uart.module_callback();
    assert(RefereeGetPowerSnapshot().power_w == 50);
    assert(RefereeGetPowerSnapshot().power_sequence == 2);
    tick_ms = 250;
    referee_frame(ID_power_heat_data, &heat, sizeof(heat));
    uart.recv_len = 8; // complete-looking stale tail is outside this DMA reception
    uart.module_callback();
    assert(RefereeGetPowerSnapshot().power_sample_ms == 200);
    assert(RefereeGetPowerSnapshot().power_sequence == 2);
    // A zero missing CRC byte used to be supplied by the cleared DMA tail.
    const uint8_t truncated[] = {
        0xa5, 0x10, 0, 0, 0x89, 2, 2, 0x59, 2, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xa4
    };
    memset(uart.recv_buff, 0, sizeof(uart.recv_buff));
    memcpy(uart.recv_buff, truncated, sizeof(truncated));
    uart.recv_len = sizeof(truncated);
    assert(Verify_CRC16_Check_Sum(uart.recv_buff, 25)); // proves the stale-tail hazard
    uart.module_callback();
    assert(RefereeGetPowerSnapshot().power_sequence == 2);
    // A CRC8-valid oversized header must not cause a CRC16 scan or recursive OOB read.
    uart.recv_buff[1] = 0xff;
    uart.recv_buff[2] = 0xff;
    Append_CRC8_Check_Sum(uart.recv_buff, 5);
    uart.module_callback();
    assert(RefereeGetPowerSnapshot().power_sequence == 2);
    // Two complete frames in one actual reception must both be decoded.
    referee_frame(ID_power_heat_data, &heat, sizeof(heat));
    uint8_t first_frame[25];
    memcpy(first_frame, uart.recv_buff, sizeof(first_frame));
    referee_frame(ID_game_robot_state, &robot, sizeof(robot));
    memmove(uart.recv_buff + sizeof(first_frame), uart.recv_buff, uart.recv_len);
    memcpy(uart.recv_buff, first_frame, sizeof(first_frame));
    uart.recv_len += sizeof(first_frame);
    uart.module_callback();
    ref = RefereeGetPowerSnapshot();
    assert(ref.power_sequence == 3 && ref.power_sample_ms == 250 && ref.limit_sample_ms == 250);

    puts("PASS: power meter and referee driver regression tests");
    free(m->can_ins);
    free(m->daemon);
    free(m);
    return 0;
}
