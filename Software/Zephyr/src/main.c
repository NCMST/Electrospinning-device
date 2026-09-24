#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define STACK_SIZE 2048
#define PRIORITY 5

K_THREAD_STACK_DEFINE(comm_stack, STACK_SIZE);
struct k_thread comm_thread_data;

void comm_entry_point(void *p1, void *p2, void *p3) {
    printk("Communication thread running on primary CPU core\n");
    while (1) {
        k_msleep(1000);
    }
}

int main(void) {
    printk("Starting Electrospinning System on ESP32-S3 (PRO_CPU)...\n");

    k_thread_create(&comm_thread_data, comm_stack,
                    K_THREAD_STACK_SIZEOF(comm_stack),
                    comm_entry_point, NULL, NULL, NULL,
                    PRIORITY, 0, K_NO_WAIT);

    return 0;
}