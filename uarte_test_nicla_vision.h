#ifndef UARTE_NICLA_VISION_H
#define UARTE_NICLA_VISION_H

#include <sdk_config.h>
#include "nrfx.h"     
#include "nrfx_uarte.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "robot_config.h"
#include "thread_mqttsn.h"
#include "task.h"

// UARTE
#define P1 32
#define UARTE_TX_PIN (14)  // P0.14
#define UARTE_RX_PIN (12)  // P0.12
#define UARTE_BAUDRATE NRF_UARTE_BAUDRATE_115200
#define RX_DATA_LENGTH 8 // Was 6, 8 for testing
#define UARTE_TIMEOUT_MS 250 
#define UART_TEST_REQUEST_LIMIT 1000
#define UART_TEST_LOG_INTERVAL 10
#define MAX_PLAUSIBLE_SEQ_GAP 100

typedef struct __attribute__((packed)) {
    int16_t seq;                          // For test
    int16_t x_start_mm;
    int16_t x_width_mm;
    int16_t distance_mm;
} CameraLineEstimate_t;


void task_camera(void *pvParameters);
CameraLineEstimate_t uarte1_get_line_estimate(void); //for debugging
void uarte1_init(void);

#endif /* UARTE_NICLA_VISION_H */