# 1.任务概要

> 本次任务，将分为底盘任务和云台任务，两个任务独立且必选，其中云台任务最终所有人将于视觉联合调试，底盘任务将择优选择两人于导航组联合调试。


## 1.主控核心数据：


### 1. 简介
自研主控常规版是一个以STM32F405RGT6为核心的嵌入式开发板，旨在为机器人云台运动控制提供运算平台，该开发板集成了两路CAN通信，两路USART通信，以及可选择接入BMI088等模块。

#### 功能简介
- 通讯接口：CAN1，CAN2，USART1，USART6，虚拟串口
- 可接入模块：BMI088-FPC，DR16
- 输入电压：19-25V
- 面积：35*35mm

---

### 引脚定义表

| 编号 | 功能         | 芯片引脚 | 复用功能                                      | 备注               |
|------|--------------|----------|-----------------------------------------------|--------------------|
| 1    | +24V         |          |                                               | 电源输入           |
| 2    | GND          |          |                                               | 地                 |
| 3    | CAN1_L       |          |                                               | CAN1_RX: PB8       |
| 4    | CAN1_H       |          |                                               | CAN1_TX: PB9       |
| 5    | CAN2_L       |          |                                               | CAN2_RX: PB5       |
| 6    | CAN2_H       |          |                                               | CAN2_TX: PB6       |
| 7    | OTG_FS_DM    | PA11     | USART1_CTS / CAN1_RX / TIM1_CH4               | USB_D+             |
| 8    | OTG_FS_DP    | PA12     | USART1_RTS / CAN1_TX / TIM1_ETR                | USB_D-             |
| 9    | NC           |          |                                               | 无                 |
| 10   | GND_USB      |          |                                               | USB地              |
| 11   | USART6_RX    | PC7      | I2S3_MCK / TIM8_CH2 / SDIO_D7 / DCMI_D1 / TIM3_CH2 |                    |
| 12   | USART6_TX    | PC6      | I2S2_MCK / TIM8_CH1 / SDIO_D6 / DCMI_D0 / TIM3_CH1 |                    |
| 13   | USART1_TX    | PA9      | TIM1_CH2 / I2C3_SMBA / DCMI_D0                |                    |
| 14   | USART1_RX    | PA10     | TIM1_CH3 / DCMI_D1                            |                    |
| 15   | IO_1_SW      | PC3      | SPI2_MOSI / I2S2_SD / ADC123_IN13             | 携带常闭按钮       |
| 16   | IO_2         | PC2      | SPI2_MISO / ADC123_IN12                       |                    |
| 17   | IO_3         | PC1      | ADC123_IN11                                   |                    |
| 18   | GND_DR16     |          |                                               | DR16地             |
| 19   | +5V_DR16     |          |                                               | DR16供电           |
| 20   | DR16         | PD2      |                                               | USART3_RX反相      |

---


### 2.1 BMI088接口

| 针脚定义     | 芯片引脚 | 复用功能    |
|--------------|----------|-------------|
| TEMP_088      | PB10     | TIM2_CH3    |
| SPI1_MOSI     | PA7      | SPI1_MOSI   |
| SPI1_MISO     | PA6      | SPI1_MISO   |
| CSB2_GYRO     | PB1      | IO          |
| CSB1_ACCEL    | PC4      | IO          |
| SPI1_SCK      | PA5      | SPI1_SCK    |
| INT3_GYRO     | PC5      | IO          |
| INT1_ACCEL    | PB0      | IO          |

---

### 2.2 CAN收发器电路
CAN收发芯片与之前相同采用**TJA1044GTK/3Z**，终端使用**120Ω**电阻作为匹配电阻。

---

### 2.3 RGB指示灯
由于使用场景需求，本代主控添加两颗**WS2812B-2020**可编程发光二极管作为指示灯，可通过**PC8（TIM3_CH3）**引脚编程设计灯语。

---

# 2.云台任务
## 2.1  器材
STM32F405RGT6（HAL库）,BMI088陀螺仪（SPI通信），双轴云台(GM6020两个 pitch,yaw),mirco-usb数据线
搭载2台GM6020的双轴云台（yaw - pitch）
使用主控 ：STM32F405RGT6 ACE自研主控模块

## 2.2任务要求
  - 任务总目标：实现云台pid角度闭环控制的世界坐标轴稳定
  - 使用HAl 库开发
  - 使用FreeeRtos 保证代码运行频率和实时性（主控制频率1khz）
  - 使用虚拟串口（usb）于视觉通讯 实现自瞄（世界坐标系）
  - 接受来自视觉发送的目标欧拉角 并控制云台运行
  - 发送视觉需要的消息（欧拉角等）
  - 在未识别到装甲板时，启用哨兵模式，自动搜索装甲板，一旦发现装甲板，启用自瞄
  - 软件电机限位，防止把线扯断
  - 陀螺仪闭环控制
  - 在机体倾斜情况下仍然可以运行
  - 结合实际赛场需求，提出功能并实现

### 云台机械结构
这两台电机（偏航轴电机与俯仰轴电机）的位置关系与连接方式如下：
#### 1. 位置关系：垂直堆叠（串联结构）
**偏航电机（Yaw Motor）**： 

位于最底部，即图片中清晰可见的黑色 GM6020 无刷电机。它的定子固定在黑色的六边形底座上，负责整个云台的水平旋转。

**俯仰电机（Pitch Motor**）： 

位于偏航电机的正上方，隐藏在白色 3D 打印支架的根部连接处。它的定子固定在偏航电机的转子（旋转部分）上，负责驱动白色支架及负载进行上下俯仰运动。

**整体布局**： 

两台电机呈 “下 - 上”垂直堆叠 的串联形态，构成了典型的双轴云台机械结构。

#### 2. 连接方式
**机械连接**：

级联驱动： 俯仰电机直接安装在偏航电机的旋转部件之上。当偏航电机转动时，会带着俯仰电机一起水平旋转；而俯仰电机独立转动时，只带动白色支架和负载上下倾斜。


负载固定： 黑色圆柱形负载（传感器/相机）被固定在白色支架内，随俯仰电机运动。

------
## 代码结构

### 任务划分

| 任务名称              | 优先级（osPriority） | 周期/触发方式          | 核心职责                                      | 备注 |
|-----------------------|----------------------|-------------------------|-----------------------------------------------|------|
| **ControlTask**       | Highest (osPriorityRealtime) | 1ms（vTaskDelayUntil） | IMU读取 → 姿态解算 → PID → 电机CAN发送 → 限位 | **核心1kHz闭环** |
| **VisionTask**        | High                 | 事件触发（信号量）     | USB CDC收发 → 协议解析 → 更新目标欧拉角       | 视觉主线程 |
| **SentryTask**        | Normal               | 200ms                  | 无目标时自动搜索（yaw慢扫）                    | 可与ControlTask合并（推荐） |
| **LEDTask**           | Low                  | 100ms                  | WS2812B灯效（状态指示）                        | 可选 |
| **DebugTask**         | Lowest               | 500ms                  | 打印调试信息（可选，比赛时可删）               | 开发用 |


- ControlTask最高优先级 + 精确1ms，保证云台稳定。
- VisionTask用**信号量**唤醒（收到数据立即处理），避免轮询浪费CPU。
- 所有共享数据（当前Euler角、目标角度、模式）用**全局结构体 + mutex**保护。

### 2. 关键共享数据结构（仅作为参考，非严格对齐）

```c
typedef struct {
    float yaw_angle;      // 世界坐标系 yaw（°）
    float pitch_angle;    // 世界坐标系 pitch（°）
    float yaw_target;     // 视觉给的目标
    float pitch_target;
    uint8_t mode;         // 0=手动 1=自瞄 2=哨兵搜索
    float gyro_yaw_rate;  // 角速度（用于前馈）
    // ... 其他如电机电流、限位标志
} GimbalState_t;

extern GimbalState_t gimbal;
extern osMutexId_t stateMutex;   // 在main里创建
```

1. ControlTask（1kHz核心闭环）

> 仅供参考，需根据具体需求修改函数

```c
void ControlTask(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);  // 1ms

    while(1)
    {
        // 1. 读取BMI088（SPI）
        BMI088_Read(&imu_data);           // 返回gyro/accel
        AHRS_Update(&imu_data);           // Mahony或Madgwick解算世界坐标Euler角
        gimbal.yaw_angle   = ahrs.yaw;
        gimbal.pitch_angle = ahrs.pitch;
        gimbal.gyro_yaw_rate = imu_data.gyro[2];

        // 2. 根据模式计算目标
        if(gimbal.mode == 1)          // 自瞄模式
        {
            // 使用视觉给的target
            PID_SetTarget(&pid_yaw, gimbal.yaw_target);
            PID_SetTarget(&pid_pitch, gimbal.pitch_target);
        }
        else if(gimbal.mode == 2)     // 哨兵模式
        {
            // 自动扫描（例如 yaw += 0.5° 每周期）
            gimbal.yaw_target += 0.5f * 0.001f;  // 慢速扫描
            if(gimbal.yaw_target > 180) gimbal.yaw_target = -180;
        }

        // 3. PID （有能力上前馈）
        float yaw_out  = PID_Calc(&pid_yaw,  gimbal.yaw_angle,  gimbal.gyro_yaw_rate);
        float pitch_out = PID_Calc(&pid_pitch, gimbal.pitch_angle);

        // 4. 软件限位（防止扯线）
        yaw_out   = CLAMP(yaw_out,   -8000, 8000);   // GM6020电流限位示例
        pitch_out = CLAMP(pitch_out, -6000, 6000);

        // 5. CAN发送给两个GM6020（ID 0x205 / 0x206 典型）
        GM6020_SendCurrent(CAN1, 0x205, yaw_out);    // yaw电机
        GM6020_SendCurrent(CAN1, 0x206, pitch_out);  // pitch电机

        // 6. 延时
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
```

2 . VisionTask（USB虚拟串口通信）

```c
void VisionTask(void *argument)
{
    while(1)
    {
        // 等待视觉数据到达（用信号量）
        osSemaphoreAcquire(visionRxSem, osWaitForever);

        // 解析自定义协议（推荐JSON或固定帧）
        ParseVisionProtocol(rx_buffer);

        // 更新目标
        osMutexWait(stateMutex, osWaitForever);
        gimbal.yaw_target   = vision_data.yaw;
        gimbal.pitch_target = vision_data.pitch;
        gimbal.mode = (vision_data.detected) ? 1 : 2;  // 有装甲板=自瞄，无=哨兵
        osMutexRelease(stateMutex);

        // 回传当前世界坐标（视觉需要）
        SendToVision(gimbal.yaw_angle, gimbal.pitch_angle, imu_data.gyro);
    }
}
```

**哨兵模式**：直接放在ControlTask里判断（`if(!vision_detected) { yaw_target += ... }`），不用单独任务，更省资源。
2. **USB中断触发信号量**：
   ```c
   void CDC_ReceiveCallback(uint8_t* buf, uint32_t len) {
       memcpy(rx_buffer, buf, len);
       osSemaphoreRelease(visionRxSem);   // 立即唤醒VisionTask
   }
   ```
3. **PID建议**：用**位置环 + 速度前馈**（gyro_yaw_rate），效果远好于纯位置PID。
4. 机体倾斜稳定：用**世界坐标系**（AHRS解算）解决。
