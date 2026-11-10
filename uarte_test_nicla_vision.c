#include "uarte_nicla_vision.h"

static const nrfx_uarte_t uarte1 = NRFX_UARTE_INSTANCE(1);
static uint8_t rx_buffer[RX_DATA_LENGTH];
static bool rx_done = false;
static bool tx_done = false;

// UART test variables
static bool UART_test = true;
static uint32_t uart_packets_requested = 0;
static uint32_t uart_packets_received = 0;
static uint32_t uart_timeouts = 0;
static uint32_t uart_errors = 0;
static uint32_t uart_missing_packets = 0;
static uint32_t uart_suspicious_seq = 0;

static uint16_t last_seq = 0;
static bool first_packet = true;


void uarte1_event_handler(nrfx_uarte_event_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRFX_UARTE_EVT_TX_DONE:
            tx_done = true;
            break;

        case NRFX_UARTE_EVT_RX_DONE:
            rx_done = true;
            break;

        case NRFX_UARTE_EVT_ERROR:
            if (UART_test){
                uart_errors++;
            }

            NRF_LOG_ERROR("UART ERROR: mask = 0x%08x",p_event->data.error.error_mask);
            NRF_LOG_FLUSH();

            break;

        default:
            break;
    }
}

void uarte1_init(void)
{
    nrfx_uarte_config_t  config = NRFX_UARTE_DEFAULT_CONFIG;
    config.pseltxd = UARTE_TX_PIN;
    config.pselrxd = UARTE_RX_PIN;
    config.baudrate = UARTE_BAUDRATE;
    config.hwfc = NRF_UARTE_HWFC_DISABLED;

    APP_ERROR_CHECK(nrfx_uarte_init(&uarte1, &config, uarte1_event_handler));

    NRF_LOG_INFO("UARTE1 initialized");    
    NRF_LOG_INFO("Baudrate: %d", UARTE_BAUDRATE);
    NRF_LOG_INFO("RX packet size: %d bytes", RX_DATA_LENGTH);
    NRF_LOG_INFO("UART test mode: %s", UART_test ? "ON" : "OFF");
    NRF_LOG_FLUSH();
}

void uarte1_request_data(void)
{
    // Send request
    tx_done = false;
    uint8_t request_byte = 'r';
    if (UART_test){
        uart_packets_requested++;
    }
    ret_code_t err = nrfx_uarte_tx(&uarte1, &request_byte, 1);

    if (err != NRF_SUCCESS){
        if (UART_test){
            uart_errors++;
        }

        NRF_LOG_WARNING("Could not start UART TX: %d", err);
        NRF_LOG_FLUSH();
    }
}

bool uarte1_receive_data(void)
{
    // Receive data
    rx_done = false;
    ret_code_t err = nrfx_uarte_rx(&uarte1, rx_buffer, RX_DATA_LENGTH);
    if (err != NRF_SUCCESS){
        if (UART_test){
            uart_errors++;
        }

        NRF_LOG_WARNING("Could not start UART RX: %d", err);
        NRF_LOG_FLUSH();
        return false;
    }

    // Wait for RX with timeout
    TickType_t start_time = xTaskGetTickCount();
    while (!rx_done)
    {
        __WFE(); // wait for events
        TickType_t elapsed_time = xTaskGetTickCount() - start_time; 
        if (elapsed_time >= pdMS_TO_TICKS(UARTE_TIMEOUT_MS)) // approximate 1 ms per loop
        {
            nrfx_uarte_rx_abort(&uarte1);
            nrfx_uarte_tx_abort(&uarte1);
            if (UART_test){
                uart_timeouts++;
            }

            NRF_LOG_WARNING("UART RX timeout, no connection to camera");
            NRF_LOG_FLUSH();
            return false;
        }
    }

    if (UART_test){
        uart_packets_received++;
    }
    return true;
}

static bool check_sequence_number(uint16_t received_seq)
{
    if (first_packet){
        first_packet = false;
        last_seq = received_seq;
        return true;
    }
    // Calculate how many sequence numbers were skipped
    uint16_t delta = (uint16_t)(received_seq - last_seq);

    // Normal behavior
    if (delta == 1){
        last_seq = received_seq;
        return true;
    }

    // Plausible forward gap
    if (delta > 1 && delta < MAX_PLAUSIBLE_SEQ_GAP){
        uint16_t missed = delta - 1;

        if (UART_test){
            uart_missing_packets += missed;
        }

        NRF_LOG_WARNING("Sequence gap: previous=%u received=%u missed=%u",
                        last_seq,received_seq,missed);

        last_seq = received_seq;
        NRF_LOG_FLUSH();

        return true;
    }

    // Duplicate, backwards, or implausibly large sequence jump
    if (UART_test){
        uart_suspicious_seq++;
    }

    NRF_LOG_WARNING(
        "Suspicious sequence: previous=%u received=%u delta=%u",
        last_seq,
        received_seq,
        delta
    );

    NRF_LOG_FLUSH();

    // No update in recived_seq
    return false;
}



CameraLineEstimate_t uarte1_get_line_estimate(void) 
{
    CameraLineEstimate_t result = {0};
    uarte1_request_data();
    
    if (!uarte1_receive_data())
    {
        return result;
    }
    
    result.seq = rx_buffer[0] | (rx_buffer[1] << 8);  // For test
    result.x_start_mm = rx_buffer[2] | (rx_buffer[3] << 8);
    result.x_width_mm = rx_buffer[4] | (rx_buffer[5] << 8);
    result.distance_mm = rx_buffer[6] | (rx_buffer[7] << 8);
 
    // Reject suspicious sequence numbers before using measurements   
    if (!check_sequence_number(result.seq))
    {
        NRF_LOG_WARNING(
            "Rejecting suspicious packet seq=%u",
            result.seq
        );
        return (CameraLineEstimate_t){0};
    }

    return result;
}

static void send_camera_mqtt_update(CameraLineEstimate_t estimate)
{
    mqttsn_camera_msg_t cameraMsg = {0};
    cameraMsg.identifier = ID;
    cameraMsg.start = estimate.x_start_mm;
    cameraMsg.width = estimate.x_width_mm;
    cameraMsg.distance = estimate.distance_mm;

    publish_camera("v2/robot/cam", cameraMsg, sizeof(cameraMsg), 0, 0);
}

static void uarte1_print_test_statistics(void)
{
    uint32_t requested = uart_packets_requested;
    uint32_t received = uart_packets_received;
    uint32_t timeouts = uart_timeouts;
    uint32_t errors = uart_errors;
    uint32_t missing = uart_missing_packets;
    uint32_t suspicious = uart_suspicious_seq;
    uint32_t lost = requested - received;

    NRF_LOG_INFO("========== UART TEST ==========");
    NRF_LOG_INFO("Packets requested: %d", requested);
    NRF_LOG_INFO("Packets received:  %d", received);
    NRF_LOG_INFO("Timeouts:          %d", timeouts);
    NRF_LOG_INFO("UART errors:       %d", errors);
    NRF_LOG_INFO("Missing packets    %d", missing);
    NRF_LOG_INFO("Suspicious seq.    %d", suspicious);
    NRF_LOG_INFO("Not received:      %d", lost);
    NRF_LOG_INFO("================================");

    NRF_LOG_FLUSH();
}

void task_camera(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    CameraLineEstimate_t estimate;

    uarte1_init();  // Initialize UARTE peripheral

    // Wait until MQTT-SN is connected
    while (!mqttsn_client_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    NRF_LOG_INFO("Camera task started.");

    for (;;) 
    {
        xLastWakeTime = xTaskGetTickCount();

        // Get line estimate from camera
        estimate = uarte1_get_line_estimate();

        // Publish via MQTT-SN
        send_camera_mqtt_update(estimate);

        // Debug logging
        NRF_LOG_INFO("seq: %u, x_start_mm: %d, x_width_mm: %d, distance_mm: %d",
                     estimate.seq, estimate.x_start_mm, estimate.x_width_mm, estimate.distance_mm);
        NRF_LOG_FLUSH();
        
        // Print test progress every log interval
        if (UART_test &&
            uart_packets_requested > 0 &&
            (uart_packets_requested % UART_TEST_LOG_INTERVAL) == 0)
        {
            uarte1_print_test_statistics();
        }

        // Finish test after processing the final request
        if (UART_test &&
            uart_packets_requested >= UART_TEST_REQUEST_LIMIT)
        {
            uarte1_print_test_statistics();

            UART_test = false;

            NRF_LOG_INFO("UART test finished after %lu requests",uart_packets_requested);
            NRF_LOG_FLUSH();
        }

        // Delay until next cycle
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200)); // Was 400, 200 with test
    }
}


