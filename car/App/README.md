# App — 自写应用代码

本目录存放**手写的应用模块**，与 CubeMX 生成的 `Core/`、`Drivers/` 完全分离。
在 CubeMX 里加引脚/外设后重新生成代码，**只会更新 `Core/` 和 `Drivers/`，不会碰本目录**。

## 模块地图

| 目录 | 文件 | 功能 | 硬件资源 |
|---|---|---|---|
| `motor/` | motor.c/h | AT8236 双路电机 PWM 驱动，方向映射、死区补偿、启动助推 | TIM1 CH1~4（PA8/PA9/PA10/PA11） |
| `motor/` | pid.c/h | 双轮速度 PID 闭环 + 前馈，参数存片内 Flash（VET6 Sector7） | TIM2/TIM4 编码器 |
| `imu/` | h30.c/h | WHEELTEC H30 惯导帧解析（姿态/角速度/加速度/四元数） | USART2（PA2/PA3），460800 |
| `imu/` | balance.c/h | 基于 H30 姿态的平衡辅助 PD 修正（叠加到电机输出） | — |
| `remote/` | ps2.c/h | PS2 无线手柄，软件模拟时序读帧，映射为左右轮指令 | PB0=DAT PB1=CMD PB12=CS PB13=CLK |
| `shell/` | cli_shell.c/h | USART3 文本命令行（控制/调参/状态查询/绘图流） | USART3，115200 |
| `lamp/` | lamp.c/h | 示警灯开关（MOS 管驱动） | PC8 |
| `common/` | app_ctrl.c/h | 控制源管理（UART / PS2 切换） | — |
| `common/` | app_state.h | 全局状态声明（急停标志、左右轮指令，定义在 main.c） | — |
| `common/` | delay_us.c/h | 微秒延时（DWT 计数器，供 PS2 时序用） | DWT |

## 其他约定

- 中断回调（UART 接收/错误、EXTI 急停、TIM3 10ms 控制节拍）统一写在
  `Core/Src/main.c` 的 `USER CODE` 块里，再分发给各模块。
- 引脚宏（`ESTOP_Pin`、`LAMP_Pin`、`PS2_*_Pin`）由 CubeMX 生成在 `Core/Inc/main.h`，
  模块代码直接引用，改引脚只需在 CubeMX 里改。
- `PWM_MAX`（4200）定义在 `main.h` 的 USER CODE 块，与 TIM1 ARR 对应。
- 参考代码（不参与编译）放 `../tools/ref/`。

## 在 Keil 中新增模块的步骤

1. 在 `App/` 下建目录放 `.c/.h`；
2. Keil：Project → Manage → Project Items 加 Group `App/<名字>`，添加 `.c`；
3. Options → C/C++ → Include Paths 加 `../App/<名字>`。
