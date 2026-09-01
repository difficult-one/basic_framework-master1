/**
 * @file smoother.cpp
 * @author USTC-RoboWalker (modified for basic_framework)
 * @brief 斜坡函数实现, 用于速度规划、输入平滑等
 * @version 0.2
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2024-06-03 1.1 规划引入优先级方式
 * @date 2026-07-19 适配 basic_framework 项目
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "smoother.h"
#include <math.h>    // fabsf

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化
 *
 * @param __Increase_Value 增长最大幅度（每周期）
 * @param __Decrease_Value 降低最大幅度（每周期）
 * @param __Slope_First    规划优先类型：真实值优先 or 目标值优先
 */
void Class_Slope::Init(float __Increase_Value, float __Decrease_Value, Enum_Slope_First __Slope_First)
{
    Increase_Value = __Increase_Value;
    Decrease_Value = __Decrease_Value;
    Slope_First = __Slope_First;
}

/**
 * @brief 斜坡函数核心，每周期调用一次，逐步将 Out 推向 Target
 *
 * 调用频率由使用者决定，例如在底盘 200Hz 任务中调用。
 */
void Class_Slope::TIM_Calculate_PeriodElapsedCallback()
{
    // 真实值优先：若真实值夹在规划值和目标值之间，直接跳到真实值（避免"反方向"规划）
    if (Slope_First == Slope_First_REAL)
    {
        if ((Target >= Now_Real && Now_Real >= Now_Planning) ||
            (Target <= Now_Real && Now_Real <= Now_Planning))
        {
            Out = Now_Real;
        }
    }

    if (Now_Planning > 0.0f)
    {
        if (Target > Now_Planning)
        {
            // 正值加速
            if (fabsf(Now_Planning - Target) > Increase_Value)
                Out += Increase_Value;
            else
                Out = Target;
        }
        else if (Target < Now_Planning)
        {
            // 正值减速
            if (fabsf(Now_Planning - Target) > Decrease_Value)
                Out -= Decrease_Value;
            else
                Out = Target;
        }
    }
    else if (Now_Planning < 0.0f)
    {
        if (Target < Now_Planning)
        {
            // 负值加速
            if (fabsf(Now_Planning - Target) > Increase_Value)
                Out -= Increase_Value;
            else
                Out = Target;
        }
        else if (Target > Now_Planning)
        {
            // 负值减速
            if (fabsf(Now_Planning - Target) > Decrease_Value)
                Out += Decrease_Value;
            else
                Out = Target;
        }
    }
    else
    {
        // Now_Planning == 0
        if (Target > Now_Planning)
        {
            // 从零正加速
            if (fabsf(Now_Planning - Target) > Increase_Value)
                Out += Increase_Value;
            else
                Out = Target;
        }
        else if (Target < Now_Planning)
        {
            // 从零负加速
            if (fabsf(Now_Planning - Target) > Increase_Value)
                Out -= Increase_Value;
            else
                Out = Target;
        }
    }

    // 更新当前规划值，供下一周期使用
    Now_Planning = Out;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
