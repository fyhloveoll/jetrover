# 给 RosRobotControllerM4 固件加 IWDG 看门狗 — 集成指南

目标:控制板固件卡死/HardFault 后 **~2 秒自动复位 MCU**,不用再人工断电重启。
对象:`麦轮底盘出厂例程/rosrobotcontrollerm_麦轮底盘_出厂例程`(STM32F407VET6 + FreeRTOS/CMSIS-OS v2,Keil MDK)。

> 全部改动都在 CubeMX 的 `USER CODE BEGIN/END` 块内 + 新增 2 个文件,**重新生成代码不会被覆盖**。

## 卡死机理(为什么这样治)
上位机发的"乱码/畸形包" → `packet_recv()` 解析越界 → 大概率 **HardFault → CPU 死循环 → 所有任务全停**(IMU/电机/舵机/蜂鸣器一起哑,USB 还在线、只能断电)。
IWDG 跑独立 LSI 时钟,CPU 卡在任何死循环都没人喂狗 → 硬复位。**喂狗任务用低优先级**:HardFault(整机死)它跑不了→复位;高优先级任务忙等(packet_rx 卡)饿死它→复位。两种都覆盖。

## 阶段一(最小、安全,先做这个)

### 1. 加文件
把 `watchdog.c`、`watchdog.h` 拷进工程:
- `watchdog.h` → `Core/Inc/`
- `watchdog.c` → `Core/Src/`
- Keil 里:Project 窗口右键某个 Group(如 `Application/User/Core`)→ Add Existing Files → 选 `watchdog.c`。

### 2. `Core/Src/main.c`
```c
/* USER CODE BEGIN Includes */
#include "lwmem_porting.h"
#include "log.h"
#include <stdio.h>
#include "watchdog.h"          /* +++ */
/* USER CODE END Includes */
```
在 `MX_NVIC_Init();` 之后、`osKernelInitialize();` 之前(即 `USER CODE BEGIN 2` 块里):
```c
  /* USER CODE BEGIN 2 */
  LOG_DEBUG("Start...\r\n");
  watchdog_init();             /* +++ 启动 IWDG(此后约 2s 内必须开始喂狗) */
  /* USER CODE END 2 */
```

### 3. `Core/Src/freertos.c`
```c
/* USER CODE BEGIN Includes */
#include "lvgl.h"
#include "watchdog.h"          /* +++ */
/* USER CODE END Includes */
```
在 `MX_FREERTOS_Init()` 的 `USER CODE BEGIN RTOS_THREADS` 块里:
```c
  /* USER CODE BEGIN RTOS_THREADS */
    watchdog_start_task();     /* +++ 创建低优先级喂狗任务 */
  /* USER CODE END RTOS_THREADS */
```

### 4. 编译、烧录、验证
- Keil 编译出 `.hex`。
- 用 **FlyMcu / ATK-XISP**(或 Linux 上 `stm32flash`)经 **UART bootloader(BOOT0 拉高 + 串口)** 烧录。烧录步骤见 `软件/开发环境搭建` 与 `下载问题解决方法`。
- 验证:正常上电跑一段时间**不应**自动复位(说明喂狗正常,无误复位);然后故意让板子卡死(跑抓取触发那个老卡死),观察 **~2 秒后板子自动重启**(IMU/话题恢复,无需断电)。

> 此阶段**不加任何 checkin**:`watchdog_task` 会无条件喂狗(`seen[]` 全 0,healthy 恒为 1),靠"HardFault 整机死"和"高优先级忙等饿死低优先级看门狗"来触发复位 —— 正好覆盖我们的卡死类型。最安全,不会误复位。

## 阶段二(可选增强,稳定后再加)
若想连"某任务干净阻塞死锁"也抓到,在一个**周期性运行(非阻塞等待)**的任务循环里加报到。
⚠️ **别加在 `packet_rx_task`**(它空闲时阻塞在 `osSemaphoreAcquire(..., osWaitForever)`,没指令时本就不循环,会被误判卡死)。
建议加在 `app_task`(主逻辑,周期性跑)的 for(;;) 里:
```c
/* Hiwonder/System/app.c 的 app_task_entry 循环内 */
watchdog_checkin(WDG_TASK_APP);
```
这样 app_task 若停止周期运行,超过 1.5s 没报到 → 停止喂狗 → 复位。

## 参数(在 `watchdog.c` 里可调)
- IWDG 超时:`Reload=1000` → 标称 ~2.0s(LSI 偏差下 1.4~3.8s)。误复位就调大 Reload。
- `WDG_PERIOD_MS=100`:喂狗任务周期。
- `WDG_TASK_TIMEOUT_MS=1500`:阶段二里某任务多久不报到判卡死。

## 上位机侧配合(已做)
我们 ROS 端已有 `board_alive()`:IWDG 复位后板子会断开重连串口,`ros_robot_controller` 需要重连(或重起 bringup)。IWDG 让"卡死→自动复位"这步无人化;ROS 端再做"检测到复位→自动重连"即闭环。
