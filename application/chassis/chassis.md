# chassis


@Todo 使用条件编译，选择麦轮(全向轮),舵轮,平衡底盘
## 工作流程

首先进行初始化，`ChasissInit()`会被`RobotInit()`调用，进行裁判系统、底盘电机的初始化。如果为双板模式，则还会初始化IMU，并且将消息订阅者和发布者的初始化改为`CANComm`的初始化。

操作系统启动后，工作顺序为：

1. 从cmd模块获取数据（如果双板则从CANComm获取）
2. 判断当前控制数据的模式，如果为停止则停止所有电机
3. 根据控制数据，计算底盘的旋转速度
4. 根据控制数据中yaw电机的编码器值`angle_offset`，将控制数据映射到底盘坐标系下
5. 进行麦克纳姆轮的运动学解算，得到每个电机的设定值
6. 获取裁判系统的数据，并根据底盘功率限制对输出进行限幅
7. 由电机的反馈数据和IMU（如果有），计算底盘当前的真实运动速度
8. 设置底盘反馈数据，包括运动速度和裁判系统数据
9. 将反馈数据推送到消息中心（如果双板则通过CANComm发送）


### 后续支持平衡底盘

新增一个app balance_chassis

## 四轮方向验证

首次上车前必须把底盘悬空，并在 `application/robot_def.h` 中把
`CHASSIS_WHEEL_TEST_MODE` 设置为 `CHASSIS_WHEEL_TEST_ENABLE`。随后分别将
`CHASSIS_WHEEL_TEST_TARGET` 设置为：

1. `CHASSIS_WHEEL_TEST_LF`：左前轮；
2. `CHASSIS_WHEEL_TEST_RF`：右前轮；
3. `CHASSIS_WHEEL_TEST_LB`：左后轮；
4. `CHASSIS_WHEEL_TEST_RB`：右后轮。

每次只编译并验证一只轮子，其余三只轮子的目标速度为零。USART2 会输出
`wheel`、`target`、`speed` 和 `current`，用于核对物理位置、CAN ID 和方向。
如果目标为正而反馈方向相反，应修改该轮的 `CHASSIS_MOTOR_*_REVERSE`，不要在
运动学公式中临时改符号。四轮全部确认后，必须把测试模式恢复为
`CHASSIS_WHEEL_TEST_DISABLE`，再进行正常麦轮运动测试。

轮测模式仍受遥控器首帧校验、掉线急停和四台底盘电机反馈在线状态约束；任一条件
不满足时测试轮目标为零。运动学解算后的四轮目标会按比例限制在
`CHASSIS_MAX_WHEEL_SPEED_APS` 内，小陀螺输出也受
`CHASSIS_ROTATE_OUTPUT_MAX_WZ` 限制。
