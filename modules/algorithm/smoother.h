/**
 * @file smoother.h
 * @author USTC-RoboWalker (modified for basic_framework)
 * @brief 斜坡函数, 用于速度规划、输入平滑等
 * @version 0.2
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2024-06-03 1.1 规划引入优先级方式
 * @date 2026-07-19 适配 basic_framework 项目
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

#ifndef ALG_SLOPE_H
#define ALG_SLOPE_H

/* Includes ------------------------------------------------------------------*/

#include <math.h>    // 提供 fabsf

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 规划优先类型
 * - Slope_First_REAL:   真实值优先 — 若当前真实值夹在规划值和目标值之间，规划值直接跳到真实值
 * - Slope_First_TARGET: 目标值优先 — 规划值严格按斜坡增减，不受真实值影响
 */
enum Enum_Slope_First
{
    Slope_First_REAL = 0,
    Slope_First_TARGET,
};

/**
 * @brief 斜坡函数类
 *
 * 每次调用 TIM_Calculate_PeriodElapsedCallback() 时，根据 Increase_Value / Decrease_Value
 * 将输出 Out 逐步推向 Target。调用频率由使用者决定（如底盘 200Hz 任务中调用）。
 */
class Class_Slope
{
public:
    void Init(float __Increase_Value, float __Decrease_Value, Enum_Slope_First __Slope_First = Slope_First_REAL);

    inline float Get_Out();
    inline void Set_Now_Real(float __Now_Real);
    inline void Set_Increase_Value(float __Increase_Value);
    inline void Set_Decrease_Value(float __Decrease_Value);
    inline void Set_Target(float __Target);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 输出值
    float Out = 0.0f;

    // 规划优先类型
    Enum_Slope_First Slope_First = Slope_First_REAL;
    // 当前规划值
    float Now_Planning = 0.0f;
    // 当前真实值
    float Now_Real = 0.0f;

    // 绝对值增量，一次计算周期改变值
    float Increase_Value = 0.0f;
    // 绝对值减量，一次计算周期改变值
    float Decrease_Value = 0.0f;
    // 目标值
    float Target = 0.0f;
};

/* Inline functions ----------------------------------------------------------*/

/**
 * @brief 获取输出值
 */
inline float Class_Slope::Get_Out()
{
    return Out;
}

/**
 * @brief 设定当前真实值（如电机实际速度反馈）
 *        仅在 Slope_First_REAL 模式下有意义
 */
inline void Class_Slope::Set_Now_Real(float __Now_Real)
{
    Now_Real = __Now_Real;
}

/**
 * @brief 设定加速度（正方向每周期最大变化量）
 */
inline void Class_Slope::Set_Increase_Value(float __Increase_Value)
{
    Increase_Value = __Increase_Value;
}

/**
 * @brief 设定减速度（负方向每周期最大变化量）
 */
inline void Class_Slope::Set_Decrease_Value(float __Decrease_Value)
{
    Decrease_Value = __Decrease_Value;
}

/**
 * @brief 设定目标值（最终要到达的值）
 */
inline void Class_Slope::Set_Target(float __Target)
{
    Target = __Target;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
