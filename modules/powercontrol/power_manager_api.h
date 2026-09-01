/**
 * @file power_manager_api.h
 * @brief C兼容接口层 — 将C++ power_manager（MotorPower/ChassisPowerManager）桥接到C框架
 *
 * 用法：
 *   1. ChassisInit() 中调用 ChassisPower_Init() 初始化
 *   2. ChassisTask()  中调用 ChassisPower_UpdateErrors() 更新四轮误差
 *   3. MotorTask 1kHz 中自动调用 ChassisPower_Update() 限幅（由motor_task.c完成）
 *
 * 整体流程：
 *   DJIMotorControl() → PID计算 → output_current
 *   ChassisPower_Update()   → 功率模型 + 分配 + limiter() → 修改output_current
 *   DJIMotorTransmit()      → output_current → tx_buff → CAN发送
 */

#ifndef POWER_MANAGER_API_H
#define POWER_MANAGER_API_H

#include "dji_motor.h"
#include "power_feedback.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化底盘功率管理器
 *        创建4个MotorPower对象(M3508)和1个ChassisPowerManager
 *        motor_lf/rf/lb/rb 必须是已通过DJIMotorInit注册的电机实例
 *
 * @param motor_lf 左前轮电机
 * @param motor_rf 右前轮电机
 * @param motor_lb 左后轮电机
 * @param motor_rb 右后轮电机
 */
void ChassisPower_Init(DJIMotorInstance *motor_lf, DJIMotorInstance *motor_rf,
                       DJIMotorInstance *motor_lb, DJIMotorInstance *motor_rb);

/**
 * @brief 初始化四舵轮底盘的舵向电机功率控制器
 *
 * @param steer_lf 左前舵向电机
 * @param steer_rf 右前舵向电机
 * @param steer_lb 左后舵向电机
 * @param steer_rb 右后舵向电机
 */
void ChassisPower_InitSteering(DJIMotorInstance *steer_lf, DJIMotorInstance *steer_rf,
                               DJIMotorInstance *steer_lb, DJIMotorInstance *steer_rb);

/**
 * @brief 设置舵向电机可占用的底盘总功率比例，范围0~0.5
 *
 * @param ratio 舵向功率比例
 */
void ChassisPower_SetSteeringPowerRatio(float ratio);
void ChassisPower_SetModelIdlePower(float idle_power_w);

/**
 * @brief 更新各电机的控制误差（速度设定值与实际值之差）
 *        误差越大 → 分配的功率预算越多
 *        在ChassisTask()中调用，每次调完DJIMotorSetRef后
 *
 * @param index 电机索引 0:左前 1:右前 2:左后 3:右后
 * @param error 速度误差（参考速度 - 反馈速度）
 */
void ChassisPower_UpdateError(int index, float error);

/**
 * @brief 执行功率限幅，修改各电机的output_current
 *        由motor_task.c在DJIMotorControl()之后调用（1kHz）
 *        内部流程：
 *          1. 用实时电流+转速更新各电机功率模型
 *          2. 从裁判系统获取total_power_limit（若未就绪则默认80W）
 *          3. 按误差比例分配功率预算
 *          4. MotorPower::limiter() 对每个电机限幅
 */
void ChassisPower_Update(void);

/**
 * @brief 设置总功率上限（W）
 *        在ChassisTask()中根据裁判系统数据调用
 *
 * @param power_limit 功率上限，单位瓦特
 */
void ChassisPower_SetLimit(float power_limit);

/**
 * @brief 发布实测功率与裁判缓冲能量的独立采样快照（任务上下文）。
 *        时间戳必须为原始接收时间，不能使用轮询时刻。NULL 清除输入。
 */
void ChassisPower_SetFeedback(const ChassisPowerFeedback *feedback);
ChassisPowerFeedbackStatus ChassisPower_GetFeedbackStatus(void);

/**
 * @brief 获取当前预估总功率（未限幅前）
 *
 * @return 总功率(W)
 */
float ChassisPower_GetPredictPower(void);
float ChassisPower_GetFeedbackPower(void);
float ChassisPower_GetCorrectedPredictPower(void);
float ChassisPower_GetCorrectedFeedbackPower(void);

/**
 * @brief 获取当前功率上限（W）
 *
 * @return 功率上限(W)
 */
float ChassisPower_GetLimit(void);

/**
 * @brief 获取模型在线修正系数。1.0表示不修正，大于1表示模型偏低、正在保守限功率。
 *
 * @return 当前模型修正系数
 */
float ChassisPower_GetModelCorrection(void);

float ChassisPower_GetBufferAttenuation(void);
float ChassisPower_GetEffectiveLimit(void);
float ChassisPower_GetMeasuredPower(void);
float ChassisPower_GetBufferEnergy(void);
float ChassisPower_GetSlipScore(int index);
float ChassisPower_GetDynamicWeight(int index);

/**
 * @brief 初始化一个单电机功率控制器
 *        不参与四轮分配，独立控制，有自己的功率预算
 *
 * @param slot 槽位编号 0~3，每个槽位一个独立电机
 */
void SingleMotorPower_Init(int slot);

/**
 * @brief 对单个电机执行功率限幅
 *        在 DJIMotorControl() 之后、DJIMotorTransmit() 之前调用（1kHz）
 *
 * @param motor  电机实例指针
 * @param slot   槽位编号（与 Init 时一致）
 * @param power_budget_w 此电机的功率预算 (W)
 */
void SingleMotorPower_Limit(DJIMotorInstance *motor, int slot, float power_budget_w);

#ifdef __cplusplus
}
#endif

#endif // POWER_MANAGER_API_H
