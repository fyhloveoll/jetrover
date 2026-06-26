/**
 * @file watchdog.h
 * @brief 外部硬件看门狗 (IWDG) — 检测固件卡死/HardFault 后自动复位 MCU,
 *        免去每次卡死都要人工断电重启。
 *
 * 适用:Hiwonder RosRobotControllerM4 (STM32F407VET6 + FreeRTOS/CMSIS-OS v2)。
 * 由 Claude 为 JetRover 项目添加。仅新增文件 + 在 USER CODE 块里插桩,
 * 不改动 CubeMX 生成的逻辑(重新生成不会被覆盖)。
 *
 * 原理:STM32 的 IWDG 跑在独立的 LSI 时钟上,与主程序无关。一旦超时窗口内
 * 没有"喂狗",就硬复位整个 MCU。我们用一个**低优先级** FreeRTOS 看门狗任务
 * 周期性喂狗:
 *   - HardFault / 整机死循环  -> 看门狗任务也跑不了 -> 不喂狗 -> 复位
 *   - 高优先级任务忙等(如 packet_rx 卡死) -> 饿死低优先级看门狗任务 -> 复位
 *   - 关键任务干净阻塞 -> 由"报到计数"门控发现 -> 停止喂狗 -> 复位
 */
#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 被监控的关键任务编号(报到用) */
typedef enum {
    WDG_TASK_PACKET_RX = 0,   /* 串口收包解析任务(卡死高发点) */
    WDG_TASK_APP,             /* 主应用任务 */
    WDG_TASK_COUNT
} wdg_task_t;

/**
 * @brief 初始化并启动 IWDG。在 main() 里 osKernelStart() 之前调用一次。
 *        超时约 2s(LSI 偏差下约 1.4~3.8s),启动后即开始计时。
 */
void watchdog_init(void);

/**
 * @brief 创建看门狗喂狗任务。在 MX_FREERTOS_Init() 的
 *        USER CODE BEGIN RTOS_THREADS 里调用一次。
 */
void watchdog_start_task(void);

/**
 * @brief 关键任务在自己的循环里调用,表示"我还活着"。
 *        看门狗任务据此判断关键任务是否卡死。
 * @param id 任务编号(wdg_task_t)
 */
void watchdog_checkin(wdg_task_t id);

#ifdef __cplusplus
}
#endif

#endif /* __WATCHDOG_H */
