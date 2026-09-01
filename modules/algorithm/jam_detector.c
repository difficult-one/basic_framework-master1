/**
 * @file jam_detector.c
 * @author basic_framework
 * @brief 拨盘卡弹检测有限状态机实现
 * @version 0.1
 * @date 2026-07-20
 *
 * 状态流转:
 * NORMAL ──(│real_current│ > threshold)──> SUSPECT
 *   ↑                                         │
 *   │                                   (timeout > t_suspect)
 *   │                                         ↓
 *   │                                    CONFIRMED
 *   │                                         │ (立即: 切角度环 → 回拨 → HANDLING)
 *   │                                         ↓
 *   └──────────────(timeout > t_handling)── HANDLING
 *
 *   SUSPECT ──(│real_current│ ≤ threshold)──> NORMAL  (电流恢复正常, 取消嫌疑)
 *
 * @copyright Copyright (c) 2022-2026
 */

/* Includes ------------------------------------------------------------------*/

#include "jam_detector.h"

#include <math.h>     // fabsf
#include <stdlib.h>   // malloc, memset

#include "bsp_dwt.h"  // DWT_GetTimeline_ms
#include "motor_def.h"// ANGLE_LOOP, SPEED_LOOP, Closeloop_Type_e

/* 默认参数 ---------------------------------------------------------------- */
#define DEFAULT_CURRENT_THRESHOLD   7782.0f  /**< 默认电流阈值, CAN原始值 (M3508: 9.5A * 819.2 ≈ 7782) */
#define DEFAULT_SUSPECT_TIMEOUT_MS  300.0f   /**< 默认嫌疑超时 300ms */
#define DEFAULT_HANDLING_TIMEOUT_MS 200.0f   /**< 默认处理超时 200ms */
#define DEFAULT_REVERSE_SPEED       1500.0f   /**< 默认反转速度 1500°/s (电机轴), 确保 PID 输出饱和 */

/* 函数实现 ---------------------------------------------------------------- */

/**
 * @brief 初始化卡弹检测器
 *
 * @param config 配置参数, 传入零值自动填充默认值
 * @param motor  拨盘电机实例
 * @return 检测器实例指针
 */
JamDetectorInstance *JamDetectorInit(JamDetector_Init_Config_s *config, DJIMotorInstance *motor)
{
    JamDetectorInstance *jam = (JamDetectorInstance *)malloc(sizeof(JamDetectorInstance));
    memset(jam, 0, sizeof(JamDetectorInstance));

    /* 参数填充: 零值 → 默认值, 非零 → 使用用户值 */
    jam->current_threshold   = (config->current_threshold   > 0.0f) ? config->current_threshold   : DEFAULT_CURRENT_THRESHOLD;
    jam->suspect_timeout_ms  = (config->suspect_timeout_ms  > 0.0f) ? config->suspect_timeout_ms  : DEFAULT_SUSPECT_TIMEOUT_MS;
    jam->handling_timeout_ms = (config->handling_timeout_ms > 0.0f) ? config->handling_timeout_ms : DEFAULT_HANDLING_TIMEOUT_MS;
    jam->reverse_speed       = (config->reverse_speed       > 0.0f) ? config->reverse_speed       : DEFAULT_REVERSE_SPEED;

    jam->motor = motor;
    jam->state = JAM_NORMAL;

    return jam;
}

/**
 * @brief 核心 FSM: 每周期调用, 返回当前状态
 *
 * 调用者需根据返回值决定是否跳过常规 loader 控制:
 *   JAM_CONFIRMED / JAM_HANDLING → 跳过
 *   JAM_NORMAL / JAM_SUSPECT     → 正常运行
 */
JamState_e JamDetectorTask(JamDetectorInstance *jam)
{
    float now = DWT_GetTimeline_ms();
    float abs_current = fabsf((float)jam->motor->measure.real_current);

    switch (jam->state)
    {
    /* --------------------------------------------------------------------
       NORMAL: 监测电流, 超阈值进入 SUSPECT
       -------------------------------------------------------------------- */
    case JAM_NORMAL:
        if (abs_current > jam->current_threshold)
        {
            jam->state = JAM_SUSPECT;
            jam->suspect_entry_time = now;
        }
        break;

    /* --------------------------------------------------------------------
       SUSPECT: 电流恢复 → 返回 NORMAL; 超时 → 确认卡弹进入 CONFIRMED
       -------------------------------------------------------------------- */
    case JAM_SUSPECT:
        if (abs_current <= jam->current_threshold)
        {
            /* 电流恢复正常, 解除嫌疑 */
            jam->state = JAM_NORMAL;
        }
        else if ((now - jam->suspect_entry_time) > jam->suspect_timeout_ms)
        {
            /* 持续超阈值达到时限, 确认卡弹 */
            jam->state = JAM_CONFIRMED;
        }
        break;

    /* --------------------------------------------------------------------
       CONFIRMED: 执行反转动作, 立即进入 HANDLING
       -------------------------------------------------------------------- */
    case JAM_CONFIRMED:
        /* 切换至速度环反向旋转，设大值让速度环输出饱和到 MaxOut */
        DJIMotorOuterLoop(jam->motor, SPEED_LOOP);
        DJIMotorSetRef(jam->motor, -jam->reverse_speed);

        jam->handling_entry_time = now;
        jam->state = JAM_HANDLING;
        break;

    /* --------------------------------------------------------------------
       HANDLING: 等待处理超时, 超时后返回 NORMAL
       -------------------------------------------------------------------- */
    case JAM_HANDLING:
        if ((now - jam->handling_entry_time) > jam->handling_timeout_ms)
        {
            jam->state = JAM_NORMAL;
        }
        break;

    default:
        jam->state = JAM_NORMAL;
        break;
    }

    return jam->state;
}

/**
 * @brief 强制复位到常规状态
 */
void JamDetectorReset(JamDetectorInstance *jam)
{
    jam->state = JAM_NORMAL;
}

/************************ COPYRIGHT(C) 2022-2026 **************************/
