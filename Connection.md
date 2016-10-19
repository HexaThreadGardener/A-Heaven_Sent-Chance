# 连接策略

## Master
- GPIO-OUTPUT 发出SPI电机调速信号
- GPIO-OUTPUT 发出状态查询信号
- USART Debug使用的串口
- IIC 电子罗盘模块接口
- IIC 定向陀螺仪接口
- EXTI-GPIO（*） 红外紧急情况感应模块
- GPIO-INPUT(*) 红外状况感应模块
- EXTI-GPIO 出界警告
- EXTI-GPIO 大量扣血警告
- GPIO-INPUT 少量扣血警告
- GPIO-INPUT 飞机回血信息
- GPIO-INPUT 飞机伤害信息
- EXTI-GPIO 目标点存在信息
- GPIO-INPUT 目标点颜色
- EXTI-GPIO HP++是否存在
- EXTI-GPIO 无人机是否存在
- EXTI-GPIO B&W是否存在

## Slave
- USART 轮询通讯板，死操无人机
- PWM ，GPIO操控电机
- EXTI-GPIO 得到SPI电机调速通讯中断
- SPI获得三位数据
- EXTI-GPIO 得到SPI状态获取通讯中断
- USART Debug使用的串口
- GPIO-OUTPUT 出界警告
- GPIO-OUTPUT 大量扣血警告
- GPIO-OUTPUT 少量扣血警告
- GPIO-OUTPUT 飞机回血信息
- GPIO-OUTPUT 飞机伤害信息
- GPIO-OUTPUT 目标点存在信息
- GPIO-OUTPUT 目标点颜色
- GPIO-OUTPUT HP++是否存在
- GPIO-OUTPUT 无人机是否存在
- GPIO-OUTPUT B&W是否存在

## 说明

### 通信频率（最大通信频率）
- 上位机 10^1 Hz （强制发送）
- 指南针,电子罗盘 10^2 Hz （按需发送）
- SPI 10^6 Hz 
- USART_Debug 10^4 Hz （按需发送）
- MCU主频 8Hz

### EXTI 中断机制
- MCU获得EXTI信号后会优先处理
- 类似Qt中的信号机制
- 一个MCU上目前看最多有16个EXTI

### 问题
- 定出红外的EXTI个数
- 检查这种通信是否合理（有利于策略）

### DDL
- <big><big>[2016-10-14 00:00:00]</big></big>
