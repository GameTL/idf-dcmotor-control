#include <stdio.h>
#include "esp_log.h"
#include "driver/uart.h" /*header file: uart.h*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
/*=============================================*/
#define UART_QUEUE_SIZE 20
#define UART_NUM      UART_NUM_1
#define BAUD_RATE     115200
#define TX_PIN        23
#define RX_PIN        22
#define BUF_SIZE      2048
/*=============================================*/
static const char *TAG = "UART_NUM";
/*=============================================*/
QueueHandle_t uart_queue;
void get_uart_params(void);
void init_uart(void);

void uart_send_data(const char *data)
{
    uart_write_bytes(UART_NUM, data, strlen(data));
}
void app_main(void)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    init_uart();
    get_uart_params();
    int i = 0;
    while(i < 5)
    {
        uart_send_data("Hello, UART!\n");
        printf("testing >:( %d\r\n", i);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
        i++;
    }
    // stop is Ctrl + ]
}

void init_uart()
{
        // For UART1
    uart_config_t uart_conf = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_conf);
    uart_set_pin(UART_NUM, 
                TX_PIN,    // TX pin
                RX_PIN,    // RX pin
                UART_PIN_NO_CHANGE,  // RTS pin (not used)
                UART_PIN_NO_CHANGE); // CTS pin (not used)
    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);

/*
note:

UART1_TX --> odom data --> ROS
                            |
                            v
                    do ur ros things
                            |
                            v
UART1_RX <------- data <-- ROS
*/

}

void get_uart_params()
{
    uart_parity_t parity;
    uint32_t baudrate;
    size_t free_size;
    // esp_err_t err;

    uart_get_parity(UART_NUM, &parity);
    uart_get_baudrate(UART_NUM, &baudrate);
    uart_get_tx_buffer_free_size(UART_NUM, &free_size);

    ESP_LOGI(TAG, "UART Parameters - Parity: %d, Baudrate: %lu, TX Buffer Free Size: %u", 
    parity, baudrate, free_size);
}

void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t data[BUF_SIZE];

    for (;;) {
        // Wait for UART events
        if (xQueueReceive(uart_queue, &event, 2000)) {
            switch (event.type) {
                case UART_DATA:
                    // Read the received data
                    int len = uart_read_bytes(UART_NUM, data, event.size, 2000);
                    if (len > 0) {
                        printf("Received (%d bytes): %.*s\n", len, len, data);
                    }
                    break;

                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    printf("UART overflow or buffer full\n");
                    uart_flush_input(UART_NUM);
                    xQueueGenericReset(uart_queue, pdFALSE);
                    break;

                default:
                    break;
            }
        }
    }
}