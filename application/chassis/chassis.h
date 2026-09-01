#ifndef CHASSIS_H
#define CHASSIS_H

#include "stdint.h"

/* Chassis power debug data. */
typedef struct
{
    float predict_power_w;            // 修正后的模型预测功率（限幅前）
    float raw_predict_power_w;        // 原始模型预测功率（修正前）
    float actual_power_w;             // 功率控制实际使用的底盘功率
    float model_feedback_power_w;     // 修正后的电机模型反馈功率
    float raw_model_feedback_power_w; // 原始电机模型反馈功率
    float power_limit_w;              // 配置的功率上限
    float model_correction;           // 在线模型修正系数
    float measured_power_w;           // 裁判系统或功率计实测功率
    float buffer_energy_j;            // 裁判系统缓冲能量
    float buffer_attenuation;         // 缓冲能量衰减系数
    float effective_power_limit_w;    // 最终可用的功率上限
    uint8_t power_meter_online;       // 功率计反馈是否在线
    uint8_t feedback_source;          // 0=模型降级, 1=CAN1功率计, 2=CAN2功率计, 3=裁判
    uint8_t feedback_valid;
    uint8_t buffer_feedback_valid;
    uint32_t feedback_age_ms;         // 电机控制实际使用的实测功率采样年龄
    float meter_bus_voltage_v;        // 功率计母线电压
    float meter_current_a;            // 功率计电流
    float meter_power_w;              // 功率计功率
    float slip_score[4];              // 各轮打滑评分
    float dynamic_weight[4];          // 各轮动态功率权重
} ChassisPowerInfo;

void ChassisInit();
void ChassisTask();

#endif // CHASSIS_H
