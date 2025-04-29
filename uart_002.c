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
#define TX_PIN        17
#define RX_PIN        16
#define BUF_SIZE      2048
/*=============================================*/
static const char *TAG = "UART_NUM";
/*=============================================*/
QueueHandle_t uart_queue;
void get_uart_params(void);
void init_uart(void);
float tx_send_curr_ang[8] = {0};
float rx_target_rads_arr[4] = {0};
int flag = 0; //to check if all motors and encoder values are updated, prevent partial data from sending
int send = 0;
typedef struct 
{
    float m_target_vel;
    float en_curr_ang_vel;
    float en_curr_ang_pos;
}robot_esp;
/*=============================================*/
static void update_arr(int m_index, float vel, float pos){
    // printf("In update_arr function, with m_index == %d\n", m_index);
    switch (m_index){
        case 1:
            //motor1 encoder1
            tx_send_curr_ang[0] = vel;
            tx_send_curr_ang[1] = pos;
            flag += 1;
            break;  
        case 2:
            //motor2 encoder2
            tx_send_curr_ang[2] = vel;
            tx_send_curr_ang[3] = pos;
            flag += 1;
            break;
        case 3:
            //motor3 encoder3
            tx_send_curr_ang[4] = vel;
            tx_send_curr_ang[5] = pos;
            flag += 1;
            break;
        case 4:
            //motor4 encoder4
            tx_send_curr_ang[6] = vel;
            tx_send_curr_ang[7] = pos;
            flag += 1;
            break;
    }

    if (flag == 4) //make sure all values are populated before transmitting to jetson
    {
        // printf("in writing to uart function.\n");
        char tx_line[256];
        int len = snprintf(tx_line, sizeof(tx_line), "%f %f %f %f %f %f %f %f\n",tx_send_curr_ang[0], tx_send_curr_ang[1],
                                                                                 tx_send_curr_ang[2], tx_send_curr_ang[3],
                                                                                 tx_send_curr_ang[4], tx_send_curr_ang[5],
                                                                                 tx_send_curr_ang[6], tx_send_curr_ang[7]);

        // printf("Writing to UART\n");                                                             
        uart_write_bytes(UART_NUM_1, tx_line, len); //Transmits tx_line via UART (in the case of loopback, Rx pin will receive this data)
        flag = 0; //Reset flag
    }
    
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
    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, UART_QUEUE_SIZE, &uart_queue, 0);

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
    /*not important just for checking*/

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
    
    while (1) {
        // Wait for an event from the UART queue
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    // Data received: event.size tells you how many bytes
                    printf("+++ EVENT +++ DATA RECEIVED\n");
                    int len = uart_read_bytes(UART_NUM_1, data, event.size, portMAX_DELAY);
                    if (len > 0) {
                        // Process the received data
                        
                        data[len] = '\0';
                        /*==============================================================================================================================================*/
                        // float v1, p1, v2, p2, v3, p3, v4, p4;
                        // int parsed = sscanf((char *)data, "%f %f %f %f %f %f %f %f", &v1, &p1, &v2, &p2, &v3, &p3, &v4, &p4);
                        // if (parsed == 8) {
                        //     tx_send_curr_ang[0]=v1;
                        //     tx_send_curr_ang[1]=p1;

                        //     tx_send_curr_ang[2]=v2;
                        //     tx_send_curr_ang[3]=p2;

                        //     tx_send_curr_ang[4]=v3;
                        //     tx_send_curr_ang[5]=p3;

                        //     tx_send_curr_ang[6]=v4;
                        //     tx_send_curr_ang[7]=p4;
                        // }
                        // printf("Received data - Sending Current Angular Values \n%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", tx_send_curr_ang[0], tx_send_curr_ang[1],
                        //                                                                    tx_send_curr_ang[2],tx_send_curr_ang[3],
                        //                                                                    tx_send_curr_ang[4],tx_send_curr_ang[5],
                        //                                                                    tx_send_curr_ang[6],tx_send_curr_ang[7]);
                        /*==============================================================================================================================================*/
                        float t_vel1, t_vel2, t_vel3, t_vel4;
                        int parsed = sscanf((char *)data, "%f %f %f %f", &t_vel1, &t_vel2, &t_vel3, &t_vel4); //string received from jetson
                        if (parsed == 4)
                        {
                            rx_target_rads_arr[0] = t_vel1;
                            rx_target_rads_arr[1] = t_vel2;
                            rx_target_rads_arr[2] = t_vel3;
                            rx_target_rads_arr[3] = t_vel4;
                        }
                        printf("Received data via UART(loopback) Target Rads:\n%.2f %.2f %.2f %.2f\n", rx_target_rads_arr[0], rx_target_rads_arr[1], rx_target_rads_arr[2], rx_target_rads_arr[3]);
                        /*==============================================================================================================================================*/                                                                  
                    }
                    break;

                case UART_FIFO_OVF:
                    printf("Hardware FIFO overflow\n");
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart_queue);
                    break;

                case UART_BUFFER_FULL:
                    printf("Ring buffer full\n");
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart_queue);
                    break;

                default:
                    break;
            }
        }
    }
}

void app_main(void)
{
    init_uart();
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
    
    get_uart_params();    
    while (1)
    {   
        //printf("----- SENDING CURRENT MOTOR AND ENCODER VALUES TO JETSON -----\n");
        // send = 0;
        // for (int i = 1; i <= 4; i++)
        // {
        //     //for loop to cycle through all 4 motors
        //     update_arr(i, (10+i), (20+i)); //updates array
        //     /*
        //     * TO DO: 
        //     * Change (10+i) to CURRENT VELOCITY (variable)
        //     * Change (20+i) to CURRENT POSITION (variable)
        //     */
        //     //printf("Updating array with motor %d valued at: vel = %d, pos = %d\n", i, (10+i),(20+i));
        // }        
        printf("\n===== RECEIVING TARGETTED RADIANS/S FROM JETSON =====\n");
        for (int j = 0; j < 1; j++)
        {
            rx_target_rads_arr[0] = 12345;
            rx_target_rads_arr[1] = 66666;
            rx_target_rads_arr[2] = 90999;
            rx_target_rads_arr[3] = 23441;
            char rx_line[256];
            int len = snprintf(rx_line, sizeof(rx_line), "%f %f %f %f\n", rx_target_rads_arr[0], rx_target_rads_arr[1], rx_target_rads_arr[2], rx_target_rads_arr[3]);
            // printf("Writing to UART\n");
            printf("Transmitting dummy values via UART(loopback):\n %s \n\n", rx_line);                                                             
            uart_write_bytes(UART_NUM_1, rx_line, len); //sends a string that has the values of the target radians, to simulate string received from jetson
        }
        vTaskDelay(250);
    }

    // stop is Ctrl + ]
}