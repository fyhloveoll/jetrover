/**
 * @file watchdog.c
 * @brief 外部硬件看门狗 (IWDG) 实现。见 watchdog.h。
 *
 * IWDG 配置:LSI(~32kHz)/64 = 500Hz,Reload=1000 => 标称 ~2.0s 超时
 * (LSI 偏差下约 1.4~3.8s)。喂狗任务每 100ms 跑一次,远快于超时,正常时
 * 持续喂狗;一旦系统卡死(任一情形)就停止喂狗 -> 硬复位。
 */
#include "watchdog.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

/* ---- IWDG ---- */
static IWDG_HandleTypeDef hiwdg;

/* ---- 关键任务报到计数(各任务循环里自增) ---- */
static volatile uint32_t s_checkin[WDG_TASK_COUNT];

/* ---- 看门狗任务静态资源 ---- */
static uint32_t s_wdg_stack[96];
static StaticTask_t s_wdg_tcb;
static const osThreadAttr_t s_wdg_attr = {
    .name = "watchdog",
    .cb_mem = &s_wdg_tcb,
    .cb_size = sizeof(s_wdg_tcb),
    .stack_mem = s_wdg_stack,
    .stack_size = sizeof(s_wdg_stack),
    .priority = (osPriority_t) osPriorityLow,   /* 低优先级:被高优先级忙等饿死 -> 触发复位 */
};

#define WDG_PERIOD_MS        100u   /* 喂狗任务周期 */
#define WDG_TASK_TIMEOUT_MS  1500u  /* 某关键任务多久不报到判为卡死 */

void watchdog_init(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 1000;       /* ~2.0s @ 500Hz */
    HAL_IWDG_Init(&hiwdg);          /* 该调用同时启动 IWDG */
}

void watchdog_checkin(wdg_task_t id)
{
    if ((unsigned)id < WDG_TASK_COUNT) {
        s_checkin[id]++;
    }
}

static void watchdog_task(void *argument)
{
    (void)argument;
    uint32_t last[WDG_TASK_COUNT] = {0};
    uint32_t stalled_ms[WDG_TASK_COUNT] = {0};
    uint8_t  seen[WDG_TASK_COUNT] = {0};   /* 该任务是否已至少报到过一次(启动宽限) */

    HAL_IWDG_Refresh(&hiwdg);              /* 进入即先喂一次,覆盖启动间隙 */

    for (;;) {
        uint8_t healthy = 1;
        for (int i = 0; i < WDG_TASK_COUNT; i++) {
            uint32_t cur = s_checkin[i];
            if (cur != last[i]) {          /* 有报到 = 活着 */
                last[i] = cur;
                stalled_ms[i] = 0;
                seen[i] = 1;
            } else if (seen[i]) {          /* 已启动过却停止报到 = 可能卡死 */
                stalled_ms[i] += WDG_PERIOD_MS;
                if (stalled_ms[i] >= WDG_TASK_TIMEOUT_MS) {
                    healthy = 0;
                }
            }
            /* 未 seen 的任务处于启动宽限期,不计入卡死判断 */
        }

        if (healthy) {
            HAL_IWDG_Refresh(&hiwdg);      /* 一切正常 -> 喂狗 */
        }
        /* 不健康 -> 不喂狗 -> IWDG 超时后硬复位整机 */

        osDelay(WDG_PERIOD_MS);
    }
}

void watchdog_start_task(void)
{
    osThreadNew(watchdog_task, NULL, &s_wdg_attr);
}
