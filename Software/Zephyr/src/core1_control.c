#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "ipc_events.h"

#define STACK_SIZE 2048
#define PRIORITY 3

K_THREAD_STACK_DEFINE(core1_stack, STACK_SIZE);
struct k_thread core1_thread_data;

void core1_entry_point(void *p1, void *p2, void *p3) {
    printk("[Core 1] Real-Time Control task active\n");

    app_telemetry_t telemetry = {
        .temperature = 24.5f,
        .humidity = 45.0f,
        .step_count = 0,
        .endstop_1_active = false,
        .atx_power_good = true
    };

    while (1) {
        telemetry.step_count++;
        k_msgq_put(&telemetry_queue, &telemetry, K_NO_WAIT);
        k_msleep(1000);
    }
}

void init_core1_tasks(void) {
    k_thread_create(&core1_thread_data, core1_stack,
                    K_THREAD_STACK_SIZEOF(core1_stack),
                    core1_entry_point, NULL, NULL, NULL,
                    PRIORITY, 0, K_NO_WAIT);
}