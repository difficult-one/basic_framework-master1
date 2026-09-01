/**
 * @file jam_detector.h
 * @author basic_framework
 * @brief 拨盘卡弹检测有限状态机, 通过监测拨盘电机电流判断是否卡弹并自动反转处理
 * @version 0.1
 * @date 2026-07-20
 *
 * @copyright Copyright (c) 2022-2026
 *
 */

#ifndef JAM_DETECTOR_H
#define JAM_DETECTOR_H

#include "dji_motor.h"
#include "stdint.h"

/* ============================================================================
  卡弹检测状态枚举
  ============================================================================ */

/**
 * @brief 卡弹检测 FSM 状态
 *
 * 状态流转:
 * NORMAL ──(电流超阈值)──> SUSPECT ──(超时)──> CONFIRMED ──(反转)──> HANDLING ──(超时)──> NORMAL
 *   ↑                         │                      ↑                        │
 *   └─────────(电流恢复正常)───┘                      └────────────────────────┘
 */
typedef enum
{
    JAM_NORMAL    = 0, /**< 常规状态: 发射机构正常运行, 持续监测电流 */
    JAM_SUSPECT   = 1, /**< 嫌疑状态: 电流超过阈值, 等待确认 */
    JAM_CONFIRMED = 2, /**< 确认状态: 嫌疑持续时间达标, 执行反转处理(瞬间进入HANDLING) */
    JAM_HANDLING  = 3, /**< 处理状态: 拨盘反转中, 等待超时后恢复 */
} JamState_e;

/* ============================================================================
  初始化配置结构体
  ============================================================================ */

/**
 * @brief 卡弹检测器初始化配置
 *
 * 所有参数均有默认值, 不传则使用默认(直接 memset 为 0 后再赋值所需项)
 */
typedef struct
{
    float current_threshold;   /**< 电流阈值, 单位为 CAN 原始值 (M3508: 9.5A ≈ 9.5 * M3508_CURRENT_COEF = 7782) */
    float suspect_timeout_ms;  /**< 嫌疑状态超时时间(ms), 若电流持续超过阈值达到此时间则确认卡弹. 推荐 300 */
    float handling_timeout_ms; /**< 处理状态超时时间(ms), 反转持续此时间后恢复常规状态. 推荐 200 */
    float reverse_speed;       /**< 反转速度(°/s), 电机轴转速. 推荐 1500, 越大扭矩越足 */
} JamDetector_Init_Config_s;

/* ============================================================================
  卡弹检测器实例
  ============================================================================ */

/**
 * @brief 卡弹检测器实例
 *
 * 每个拨盘电机对应一个实例, 在 shoot 应用初始化时创建.
 * 外部仅需通过 JamDetectorTask() 轮询, 通过返回值判断当前状态.
 */
typedef struct
{
    JamState_e state;              /**< 当前状态 */

    float suspect_entry_time;      /**< 进入嫌疑状态的时间戳 (ms, DWT 时间线) */
    float handling_entry_time;     /**< 进入处理状态的时间戳 (ms, DWT 时间线) */

    /* 配置参数 (副本, 从 Init_Config 拷贝) */
    float current_threshold;
    float suspect_timeout_ms;
    float handling_timeout_ms;
    float reverse_speed;

    /* 关联的硬件 */
    DJIMotorInstance *motor;       /**< 拨盘电机实例指针 */
} JamDetectorInstance;

/* ============================================================================
  函数声明
  ============================================================================ */

/**
 * @brief 初始化卡弹检测器
 *
 * @param config 初始化配置, 填入各阈值和角度. 浮点值传 0 则使用默认值
 * @param motor  拨盘电机实例指针 (来自 DJIMotorInit)
 * @return JamDetectorInstance* 返回实例指针, 供 JamDetectorTask() 使用
 */
JamDetectorInstance *JamDetectorInit(JamDetector_Init_Config_s *config, DJIMotorInstance *motor);

/**
 * @brief 卡弹检测 FSM 核心, 每周期调用一次
 *
 * 在 ShootTask 中调用, 调用频率建议 ≥ 200Hz.
 * 调用者根据返回值决定是否跳过常规发射控制:
 *   - 返回 JAM_HANDLING: 跳过 loader 常规控制, 拨盘正在反转处理
 *   - 返回 JAM_CONFIRMED: 本周期已触发反转, 常规控制自然跳过
 *   - 返回 JAM_NORMAL / JAM_SUSPECT: 发射机构正常运行
 *
 * @param jam 卡弹检测器实例
 * @return JamState_e 当前状态
 */
JamState_e JamDetectorTask(JamDetectorInstance *jam);

/**
 * @brief 手动复位卡弹检测器到常规状态
 *
 * 在发射任务停止、遥控器急停等场景调用.
 *
 * @param jam 卡弹检测器实例
 */
void JamDetectorReset(JamDetectorInstance *jam);

#endif // !JAM_DETECTOR_H
