基于 STM32F103C8 的车载电控单元（VCU）启动与升级实验项目，
采用 Bootloader + APP 双工程架构。项目在 Bootloader 侧集成了 FreeRTOS 多任务调度、PWM 电机控制、车速采集显示、超声波避障、MPU6050 姿态预警以及 IAP 在线升级框架，
用于验证嵌入式系统从“启动管理”到“业务控制”再到“固件升级”的完整链路。
1. 项目概述

本仓库包含两个独立工程：

- `Free_bootloader_v1.2`
  - 主工程
  - 基于 FreeRTOS 的 Bootloader/VCU 控制程序
  - 实现系统初始化、任务调度、多外设协同、控制逻辑和 IAP 升级框架

- `APPV1.1`
  - 用户应用验证工程
  - 目前为最小 APP 示例
  - 被链接到 `0x0800F000`
  - 当前功能仅为 LED 闪烁，用于验证 APP 分区、向量表重映射和跳转机制

这个项目的重点不只是“写一个 Bootloader”，而是把以下几个典型嵌入式能力放在同一个系统里进行联调：

- Bootloader 与 APP 解耦
- RTOS 多任务系统设计
- 外部中断 / 定时器 / PWM / DMA / Flash 擦写 / 串口通信协同
- 车速、距离、姿态等多源输入的状态联动
- IAP 固件接收、写入、校验和跳转
2. 已实现功能

2.1 启动与系统架构
- 基于 STM32F103C8 构建 Bootloader + APP 双工程结构
- APP 工程独立链接到 `0x0800F000`
- APP 启动时通过 `SCB->VTOR = FLASH_BASE | 0xF000` 完成向量表重映射
- Bootloader 侧提供 Cortex-M 标准跳转能力：
  - 关闭全局中断
  - 关闭相关外设
  - 清除中断标志
  - 重设 MSP
  - 跳转到 APP Reset Handler

2.2 FreeRTOS 多任务调度
Bootloader 工程中已实现多任务模型，核心任务包括：

- `start_task`
  - 创建队列、信号量和业务任务
  - 初始化定时器中断与输入捕获

- `pwm_task`
  - 根据按键事件调整 PWM 输出，实现电机正向档位调节

- `reverse_task`
  - 根据按键事件调整 PWM 输出，实现电机反向档位调节

- `Motoshow_task`
  - 周期刷新车速显示

- `us105_task`
  - 处理 US-015 返回脉宽并换算距离

- `st_task`
  - 周期触发 US-015 测距

- `mpu6050_task`
  - 读取 DMP 姿态数据并进行倾角判定

- `iap_task`
  - 在开启 IAP 功能后处理 DMA 接收完成、固件校验、Flash 写入和 APP 跳转

2.3 电机控制
- 使用 `TIM3 CH3` 输出 PWM，控制引脚为 `PB0`
- 通过外部中断按键事件触发正向 / 反向档位调节
- 通过修改 `TIM_SetCompare3(TIM3, value)` 调整占空比
- 遇到障碍物或姿态异常时会主动降低输出，实现安全联动

2.4 车速采集与显示
- 车速脉冲由外部中断计数
- `TIM2` 以 1 秒周期触发刷新
- LCD 上显示 `Speed:xx r/min`
- 当前代码中速度值由中断累计计数后换算显示，适合作为霍尔测速/脉冲测速原型验证

2.5 超声波避障
- 使用 `US-015` 模块
- 触发引脚由软件输出启动脉冲
- 回波脉宽通过 `TIM1 输入捕获` 获取
- 距离换算后显示到 LCD
- 当距离小于等于 5 cm 时：
  - 蜂鸣器鸣叫
  - LED 点亮
  - PWM 自动降速

2.6 姿态预警
- 集成 `MPU6050`
- 使用软件 I2C 驱动，并启用 DMP 获取姿态角
- 当前代码主要使用 `roll` 角进行判断
- 当倾角大于 45° 时：
  - LCD 显示 `Caution!`
  - PWM 自动降速

2.7 IAP 在线升级框架
当前工程已经包含 IAP 的完整基础代码，但默认未开启。

已实现能力：
- USART1 + DMA 接收 APP 固件
- 固件缓存放置在固定 RAM 区域：`0x20004000`
- 对接收数据长度进行统计
- 对 APP Reset Vector 做合法性校验
- 按页擦除 Flash 并将 APP 写入 `0x0800F000`
- 写入完成后跳转执行 APP

默认配置：
- `Free_bootloader_v1.2/HARDWARE/IAP/iap.h`
  - `#define ifopen 0`

也就是说：
- 当前仓库默认编译状态下，IAP 框架代码存在
- 但升级功能默认关闭
- 若需要演示在线升级，请手动改为 `1` 并重新编译

---
3. 当前仓库状态说明

这一部分非常重要，建议使用者先看。

3.1 Bootloader 不是“空壳”
当前 `Free_bootloader_v1.2` 并不是一个只负责跳转的极简 Bootloader，而是已经承载了实际车载控制逻辑，包括：
- PWM 电机控制
- 车速统计
- 姿态检测
- 超声波避障
- LCD 状态显示
- IAP 升级框架

因此，它更接近“带启动与升级能力的 VCU 主控程序”。

3.2 APP 工程当前是最小验证程序
`APPV1.1` 当前实现非常简单：
- 重映射 `VTOR`
- 初始化 `PC13 LED`
- 每 500 ms 闪烁一次

这说明当前 APP 工程的定位是：
- 用于验证 APP 链接地址是否正确
- 用于验证 Bootloader 跳转与向量表重映射链路
- 不是最终业务 APP

3.3 IAP 默认关闭
源码中：
- `ifopen = 0`
- 说明当前默认构建并不会启用 DMA 收固件和 Flash 写入逻辑

3.4 Flash 分区存在“实现已写好，但链接保护还需进一步工程化”的情况
当前已确认的信息如下：

- APP 链接地址：`0x0800F000`
- APP Scatter 文件：
  - `APPV1.1/OBJ/APP.sct`
  - 链接区域大小：`0x1000`（4 KB）

- Bootloader 工程当前 Scatter 文件：
  - `Free_bootloader_v1.2/OBJ/PWM.sct`
  - 仍然是从 `0x08000000` 使用整片 Flash

这意味着：
- 从“运行结果”角度，Bootloader 代码大小如果未覆盖到 `0x0800F000`，仍然可以与 APP 共存
- 但从“工程约束”角度，Bootloader 侧最好再单独限制链接区域，避免后续功能增加后侵占 APP 区

建议后续完善：
- 将 Bootloader 的链接区域显式限制在 APP 起始地址之前
- 将 APP 区大小从 4 KB 扩展到更合理空间
- 根据真实芯片容量重新规划 Boot/App 分区

---

4. 硬件清单

根据当前源码已确认的硬件如下：

主控
- STM32F103C8T6

显示与交互
- SPI TFT LCD（128x160）
- 板载 LED
- 蜂鸣器
- 按键输入

传感器
- US-015 超声波测距模块
- MPU6050 六轴传感器

控制对象
- 电机 / 电调 PWM 控制接口

调试与升级
- USART1 串口（115200）
- CH340 / USB 转串口模块
- ST-Link（下载调试）

---

5. 已确认引脚映射

以下内容基于当前源码中的初始化代码和注释整理。

5.1 核心控制与指示
- LED：`PC13`
- 蜂鸣器：`PB15`
- PWM 输出：`PB0` (`TIM3_CH3`)
- 串口 TX：`PA9` (`USART1_TX`)
- 串口 RX：`PA10` (`USART1_RX`)

5.2 按键 / 外部中断
- 加速键：`PB1` -> `EXTI1`
- 反向 / 反向档位键：`PB12` -> `EXTI12`
- 速度脉冲 / 第三路外部输入：`PC14` -> `EXTI14`

说明：
- 代码中 `PC14` 仍沿用了 `KEY2` 的宏命名
- 但从实际逻辑看，它在中断里承担了“速度脉冲计数”的角色，更接近霍尔测速输入

5.3 超声波模块 US-015
- Trigger：`PA12`
- Echo / 输入捕获：`PA8` (`TIM1_CH1`)

 5.4 MPU6050（软件 I2C）
- SCL：`PB10`
- SDA：`PB11`

5.5 LCD（SPI 接口）
根据 LCD/SPI 驱动注释确认：
- LCD SDA：`PA7`
- LCD SCK：`PA5`
- LCD BL/LED：`PB6`
- LCD A0 / DC：`PB7`
- LCD RESET：`PB8`
- LCD CS：`PB9`

---

 6. 软件环境

开发工具
- Keil MDK5
- ARMCC 5
- STM32F1xx DFP

代码框架
- STM32 Standard Peripheral Library
- FreeRTOS

芯片与架构
- STM32F103C8
- ARM Cortex-M3

工程配置
- Bootloader 工程：`Free_bootloader_v1.2/USER/project.uvprojx`
- APP 工程：`APPV1.1/USER/APP.uvprojx`

---

7. 工程结构

```text
STM32_VehicularUnit_Project
├── APPV1.1
│   ├── CORE
│   ├── HARDWARE
│   │   ├── CAN
│   │   ├── KEY
│   │   └── LED
│   ├── OBJ
│   ├── STM32F10x_FWLib
│   ├── SYSTEM
│   │   ├── delay
│   │   ├── sys
│   │   └── usart
│   └── USER
│       ├── APP.uvprojx
│       ├── main.c
│       ├── stm32f10x_it.c
│       └── system_stm32f10x.c
│
├── Free_bootloader_v1.2
│   ├── CORE
│   ├── FreeRTOS
│   ├── HARDWARE
│   │   ├── ADC
│   │   ├── BEEP
│   │   ├── DMA
│   │   ├── EXTI
│   │   ├── IAP
│   │   ├── KEY
│   │   ├── LCD
│   │   ├── LED
│   │   ├── MPU6050
│   │   ├── SPI
│   │   ├── STMFLASH
│   │   ├── TIMER
│   │   └── us105
│   ├── OBJ
│   ├── STM32F10x_FWLib
│   ├── SYSTEM
│   │   ├── delay
│   │   ├── sys
│   │   └── usart
│   └── USER
│       ├── project.uvprojx
│       ├── main.c
│       ├── stm32f10x_it.c
│       └── system_stm32f10x.c
│
└── README.md

适合展示的项目亮点

如果你准备用这个项目写简历或讲项目，这几个点非常值得强调：

基于 STM32F103C8 实现 Bootloader + APP 双工程架构

通过 FreeRTOS 构建多任务车载控制原型

使用 EXTI、TIM、PWM、DMA、USART、Flash 擦写等外设形成完整控制闭环

实现超声波避障、姿态预警与电机降速联动

实现 IAP 在线升级框架与 Cortex-M 跳转机制

通过 APP 独立链接和 VTOR 重映射完成用户程序解耦
