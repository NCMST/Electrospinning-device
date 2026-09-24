#ifndef IPC_EVENTS_H
#define IPC_EVENTS_H

#include <zephyr/kernel.h>

typedef enum {
    CMD_START_INJECTION,
    CMD_STOP_INJECTION,
    CMD_SET_FLOW_RATE,
    CMD_TOGGLE_220V
} cmd_type_t;  //

typedef struct {
    cmd_type_t type;
    float flow_rate_ml_h;
    uint8_t channel_220v;
    bool state;
} app_command_t;

typedef struct {
    float temperature;
    float humidity;
    uint32_t step_count;
    bool endstop_1_active;
    bool atx_power_good;
} app_telemetry_t;

extern struct k_msgq cmd_queue;
extern struct k_msgq telemetry_queue;

#endif