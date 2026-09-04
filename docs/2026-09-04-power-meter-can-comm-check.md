# 功率计与 C 板 CAN 通信成功性判定方案

- 日期:2026-09-04
- 适用固件:basic_framework(STM32F407 主控 / "C 板")
- 对象:INA226 CAN 功率计(底盘总供电支路) ↔ C 板 CAN1
- 性质:纯排查/验收方案,**不改代码**,全部使用板载现有调试输出
  (SEGGER RTT、USART 打印、调试器变量观察)

---

## 1. 目的

在**不修改固件、不依赖 USB-CAN 分析仪**的前提下,判定 C 板与 CAN 功率计之间的
通信是否"成功",并把"成功"细化到**帧到达 → ID 匹配 → 帧完整 → 在线稳定 →
数据合理**五个层级。任一层不通过都能给出对应的排查方向(见第 6 节)。

## 2. 基线(方案所依据的当前代码状态)

| 项目 | 当前值 | 出处 |
|---|---|---|
| 功率计接入 | CAN1(`POWER_METER_USE_CAN1=1`,CAN2 关闭) | `application/robot_def.h` |
| 收发 ID | `0x605`(仅接收,功率计主动周期上报,8 字节) | `robot_def.h`, `modules/power_meter/power_meter.c` |
| 帧字段(大端 i16) | `shunt_mv, bus_mv, current_ma, power_centiw` | `power_meter.c` 解码 |
| 换算 | `shunt_mv→V×0.001, current_ma→A×0.001, power_centiw→W×0.01` | 同上 |
| 在线超时 | `POWER_METER_TIMEOUT_MS=100ms`,daemon `100` | `robot_def.h` / `chassis.c` |
| 观测通道 1 | RTT:`LOG("pm online=%d bus=... current=... power=...")` 周期 `POWER_METER_DEBUG_PRINT_PERIOD=100` 次任务循环一条 | `application/chassis/chassis.c` ~L839 |
| 观测通道 2 | USART1 原始帧打印 `PowerMeterPrintCanFrameUART`(`PM CAN1 id=0x605 len=8 data=...`);未在线时改打总线 debug `CANPrintDebugUART` | `chassis.c` / `modules/power_meter/power_meter.c` |
| 观测通道 3 | 调试器实时观察 volatile 变量(`power_meter_debug_*`、`can_debug_*`) | `power_meter.h` / `bsp/can/bsp_can.h` |

> ⚠️ **需现场确认的两个假设**
> 1. 功率计为**主动周期上报**,C 板只收不发(当前代码无发送路径);
>    若实际硬件是"查询-应答"型,请先说明,再补充 C 板发送侧验证。
> 2. 报文周期应 < `100ms`(在线超时)。实测周期按 6.5 节推算确认。

## 3. 通信链路的五层判据

| 层 | 含义 | 判定观测量 | "成功"标准 |
|---|---|---|---|
| L1 总线活动 | CAN 控制器确实收到帧 | `can_debug_rx_total`、`power_meter_debug_can1_rx_count` | 数值持续增长 |
| L2 ID 匹配 | 收到的帧能命中功率计注册 | `can_debug_watch_count` / `match_count`;`power_meter_debug_callback_count`、`power_meter_debug_rx_id==0x605` | 回调计数增长,`rx_id==0x605`,`no_match` 不增 |
| L3 帧完整 | DLC 与长度符合协议 | `power_meter_debug_rx_len==8`、`power_meter_debug_short_frame_count` | 恒为 8;短帧计数不增长 |
| L4 在线稳定 | 持续接收、不超时 | `PowerMeterIsOnline()`/`chassis_power_info.power_meter_online`、RTT `pm online=1` | 观察期 ≥1 min 全程 `online=1`,无掉线 |
| L5 数据合理 | 数值符合物理实际 | `bus_voltage_v / current_a / power_w` 及原始字节 | 见第 5 节 |

**总判定:五层全部通过 = CAN 通信成功。** 只要 L1 不通过,后面各层必然不通过,
此时直接跳第 6.1 节处理。

## 4. 准备工作

1. **接线核对(断电进行)**:
   - 功率计 `CANH/CANL` ↔ C 板 CAN1 `CANH/CANL`,且两端**共地**;
   - 总线两端(或至少一端)有 **120 Ω 终端电阻**,无其他设备干扰;
   - 功率计由总线供电支路正常供电(否则只有电压没有电流/功率样本)。
2. 固件确认:烧录当前代码(保证 `POWER_METER_ENABLE=1`、`USE_CAN1=1`、`RX_ID=0x605`),
   上电前确认 C 板与功率计均已通电。
3. 打开观测工具:
   - J-Link + RTT Viewer(通道 1);
   - USART1 串口终端(波特率与 `huart1` 配置一致)(通道 2);
   - 调试器(建议 ST-LINK/J-Link 的 Live Watch),把以下变量加入观察窗:
     `can_debug_rx_total`、`can_debug_watch_count`、`can_debug_no_match_count`、
     `power_meter_debug_callback_count`、`power_meter_debug_rx_id`、
     `power_meter_debug_rx_len`、`power_meter_debug_short_frame_count`、
     `power_meter_debug_rx_count`、`power_meter_debug_data`、
     `chassis_power_info`。
4. 底盘置于安全位(建议轮子悬空),仅用于通断验证时不使能大负载。

## 5. 分步执行与数据合理性核对

> 每步先记录观测量数值,再与"预期"比对;不满足预期则记下症状,完成后查第 6 节。

### 5.1 L1 总线活动

- 操作:上电后静置 2~3 s。
- 观察:`can_debug_rx_total` / `power_meter_debug_can1_rx_count` 是否持续增长;
  RTT 是否持续出现 `pm ...` 打印;USART1 是否出现 `PM CAN1` 或总线 debug 打印。
- 预期:两个计数单调递增(增长速率反映总线负载,只要求"在涨")。
- 若一个都不涨:见 6.1(物理/配置层)。

### 5.2 L2 ID 匹配

- 观察:`power_meter_debug_callback_count` 是否随之增长;
  `power_meter_debug_rx_id` 是否为 `0x605`;`can_debug_no_match_count` 是否保持为 0
  (或相对 `match_count` 可忽略)。
- 预期:回调计数增长且 `rx_id==0x605`。
- 若总线有帧但回调不涨:见 6.2(ID/滤波器不匹配)。

### 5.3 L3 帧完整

- 观察:USART1 打印 `PM CAN1 id=0x605 len=8 ...`;
  `power_meter_debug_rx_len` 恒为 8;`power_meter_debug_short_frame_count` 不增长。
- 预期:`len=8` 且短帧计数为 0 或长时间不变。
- 若出现短帧增长:见 6.3(DLC 不符/干扰截断)。

### 5.4 L4 在线稳定

- 观察:RTT `pm online=1`;`chassis_power_info.power_meter_online=1`;
  `feedback_source==1`(1=CAN1 功率计,见 `docs/superpowers/specs/2026-08-31-power-meter-feedback-design.md`)。
- 操作:持续观察 ≥1 min(可轻推油门制造负载变化)。
- 预期:`online` 全程为 1,`pm ...` 行持续打印,**无间歇消失**。
- 若 `online` 在 0/1 间跳变:见 6.4(周期接近超时或偶发丢帧)。

### 5.5 L5 数据合理性(本方案的核心验收项)

在 L1–L4 均通过的前提下,核对物理量:

| 检查项 | 操作 | 预期(示例值按实际供电修正) |
|---|---|---|
| 总线电压 | 静置读取 `bus`(RTT)/`bus_mv` | ≈ 底盘供电电压(如 24 V 电源下约 23~25 V),非 0、不跳变 |
| 空载电流/功率 | 电机不使能 | `current` 接近 0、`power` 接近 0(仅静态损耗,可用 `POWER_METER_IDLE_POWER_W` 对照) |
| 电流方向与随负载变化 | 缓慢使能电机/推油门(轮悬空) | `current_a`、`power_w` 随油门**同向上升**,松油门回落;符号与"消耗功率为正"约定一致(若有负号需记录) |
| 功率≈电压×电流 | 任一稳态 | `|power_w − bus_v × current_a|` 在换算误差内(注意代码 `power_centiw` 为外部上报,可能存在系统误差,只做数量级核对) |
| 原始帧与解析一致 | 对照 USART1 `PM CAN1 ... data=` 16 进制与 RTT 数值 | 手动按大端换算 `shunt_mv/bus_mv/current_ma/power_centiw` 与打印一致 |
| 数值无明显跳变/毛刺 | 全程观察 | 无周期性的乱跳(偶发 1 帧可接受,持续毛刺见 6.5) |

> 若无条件使能电机,可用**已知阻性负载**(如功率电阻)接入受测支路:
> 记录电流/功率,与 `U²/R` 或 `U×I` 理论值对比,误差在 INA226 精度 + 采样电阻误差范围内即通过。

### 5.6 报文周期实测(确认假设 2)

- 方法:在调试器里连续两次读取 `power_meter_debug_rx_count` 与对应时间戳
  (或观察 `chassis_power_info.feedback_age_ms` 的典型值),推算单帧周期;
  也可数 RTT 打印间隔内 `rx_count` 的增量换算。
- 预期:**实测周期 + 抖动余量 < 100 ms**(代码超时值)。
- 若接近甚至超过 100 ms:在线判定会不稳定,见 6.4。

## 6. 症状 → 原因速查

| # | 症状 | 最可能原因 | 动作 |
|---|---|---|---|
| 6.1 | L1 无任何计数/打印 | 接线断/接反/未共地;功率计未供电或未上电;CAN 控制器未初始化(检查 `robot_def.h` 开关与 `bsp_can` 初始化);波特率与功率计不一致 | 万用表量 CANH/L 对地电压与供电;核对 CAN 波特率配置与功率计手册;检查终端电阻;示波器看波形(可选) |
| 6.2 | 总线有帧但回调不涨 | ID 不符(`0x605` 假定错误);滤波器未放行(标准/扩展帧格式或掩码设置) | 用 `can_debug_last_std_id` 看实际收到的 ID;核对帧格式(标准帧 `0x605` vs 扩展帧)与 `CAN_Init_Config_s` 滤波器配置 |
| 6.3 | `short_frame_count` 增长 | 功率计 DLC≠8 或时序截断;总线干扰 | 检查帧长度设置与线缆屏蔽/终端 |
| 6.4 | `online` 0/1 跳变 | 报文周期接近 `timeout_ms(100)`;偶发丢帧(总线负载/仲裁/干扰) | 实测周期后上调 `POWER_METER_TIMEOUT_MS`(属改配置,需用户同意);排除干扰 |
| 6.5 | 帧完整但数值不合理(电压≈0/固定/乱跳) | 功率计本身未在测总线(接线位置);单位或标定不符;采样电阻/量程设置;C 板与功率计地电位差 | 复核功率计安装支路与手册量程;对照万用表实测总线电压 |

## 7. 判定记录表(建议现场填写)

| 层 | 观测量(示例) | 通过? | 备注 |
|---|---|---|---|
| L1 | `can_debug_rx_total` 增长 | ☐ | |
| L2 | `callback_count` 增、`rx_id=0x605` | ☐ | |
| L3 | `rx_len=8`、短帧计数不变 | ☐ | |
| L4 | ≥1 min `online=1` | ☐ | |
| L5 | `bus≈供电电压`、负载变化跟随、P≈U×I | ☐ | |
| 周期 | 实测周期 < 100 ms | ☐ | |

**结论:CAN 通信 □成功 □失败(失败层:L__,见第 6 节 #__)**。

## 8. 相关代码/符号速查

- 配置宏:`application/robot_def.h`(`POWER_METER_*`)
- 驱动与解码:`modules/power_meter/power_meter.c`、`power_meter.h`
- 总线 debug 变量:`bsp/can/bsp_can.h`(`can_debug_*`)
- 打印实现:`PowerMeterPrintCanFrameUART()`(功率计帧)、`CANPrintDebugUART()`(总线 debug)、
  RTT `LOG`(`bsp/log/bsp_log.h`)
- 功率反馈状态:`modules/powercontrol/power_feedback.h`、`chassis_power_info`
  (`application/chassis/chassis.c`)
- 相关既有文档:`docs/superpowers/specs/2026-08-31-power-meter-feedback-design.md`
