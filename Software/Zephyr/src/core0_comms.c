#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "ipc_events.h"

#define STACK_SIZE 2048
#define PRIORITY 5

K_THREAD_STACK_DEFINE(core0_stack, STACK_SIZE);
struct k_thread core0_thread_data;

void core0_entry_point(void *p1, void *p2, void *p3) {
    printk("[Core 0] Communication task active\n");
    
    app_telemetry_t telemetry;

    while (1) {
        if (k_msgq_get(&telemetry_queue, &telemetry, K_MSEC(500)) == 0) {
            printk("[Core 0] Telemetry received: Temp=%.1f C, Hum=%.1f %%\n",
                   (double)telemetry.temperature, (double)telemetry.humidity);
        }
        k_msleep(100);
    }
}

void init_core0_tasks(void) {
    k_thread_create(&core0_thread_data, core0_stack,
                    K_THREAD_STACK_SIZEOF(core0_stack),
                    core0_entry_point, NULL, NULL, NULL,
                    PRIORITY, 0, K_NO_WAIT);
}