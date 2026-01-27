
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "can_comm.h"
#include "esp_log.h"

static const char *TAG = "uart_events";

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM 3
/*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern
设置接收器接收的连续相同字符的数量，该数量定义了通用异步收发传输器（UART）模式*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

static void cmd_parse_qt(uint8_t *buf)
{
    uint8_t frameHeader = (uint8_t)buf[0]; // 帧头
    uint8_t command = (uint8_t)buf[1];     // 命令
    uint8_t data1 = (uint8_t)buf[2];       // 数据1
    uint8_t data2 = (uint8_t)buf[3];       // 数据2
    uint8_t data3 = (uint8_t)buf[4];       // 数据3
    uint8_t data4 = (uint8_t)buf[5];       // 数据4
    uint8_t checksum = (uint8_t)buf[6];    // 校验和
    uint8_t frameEnd = (uint8_t)buf[7];    // 帧尾
    if (frameHeader != 0xAA)
    {
        return;
    }
    if (frameEnd != 0xEE)
    {
        return;
    }
    uint8_t calcSum = 0;
    for (int i = 0; i < 6; i++)
    {
        calcSum += (uint8_t)buf[i];
    }

    if (calcSum != checksum)
    {
        return;
    }
    switch (command)
    {
    case 0x01: // 启动
        // can发送启动命令
        xEventGroupSetBits(sync_event, MOTOR_START);
        break;
    case 0x02: // 停止
        // can发送停止命令
        xEventGroupSetBits(sync_event, MOTOR_STOP);
        break;
    }
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);
    assert(dtmp);
    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, pdMS_TO_TICKS(500)))
        {
            bzero(dtmp, RD_BUF_SIZE);
            // ESP_LOGD(TAG, "uart[%d] event:", EX_UART_NUM);
            switch (event.type)
            {
            // Event of UART receiving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.

            */
            case UART_DATA:
                ESP_LOGD(TAG, "[UART DATA]: %d", event.size);
                uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                ESP_LOGD(TAG, "[DATA EVT]:");
                cmd_parse_qt(dtmp);

                    uart_write_bytes(EX_UART_NUM, (const char *)dtmp, event.size);
                break;
            // Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGD(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart0_queue);
                break;
            // Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGD(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart0_queue);
                break;
            // Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGD(TAG, "uart rx break");
                break;
            // Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGD(TAG, "uart parity error");
                break;
            // Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGD(TAG, "uart frame error");
                break;
            // UART_PATTERN_DET
            case UART_PATTERN_DET:
                uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                int pos = uart_pattern_pop_pos(EX_UART_NUM);
                ESP_LOGD(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1)
                {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    uart_flush_input(EX_UART_NUM);
                }
                else
                {
                    uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                    uint8_t pat[PATTERN_CHR_NUM + 1];
                    memset(pat, 0, sizeof(pat));
                    uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                    ESP_LOGD(TAG, "read data: %s", dtmp);
                    ESP_LOGD(TAG, "read pat : %s", pat);
                }
                break;
            // Others
            default:
                ESP_LOGD(TAG, "uart event type: %d", event.type);
                break;
            }
        }
        ESP_LOGD(TAG, "for down !!!");
        if (jibu_flag) //有计步信息
        {
            jibu_flag = 0;
            uint8_t data[8] = {0};
            data[0] = 0xAA;
            data[1] = 0x04;
            data[2] = (uint8_t)(total_steps >> 24);
            data[3] = (uint8_t)(total_steps >> 16);
            data[4] = (uint8_t)(total_steps >> 8);
            data[5] = (uint8_t)total_steps;
            uint8_t sum = 0;
            for (int i = 0; i < 6; i++)
            {
                sum += data[i];
            }
            data[6] = sum;
            data[7] = 0xee;
            uart_write_bytes(EX_UART_NUM, (const char *)data, 8);
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Set uart pattern detect function. 设置uart模式检测功能。
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    // 重置模式队列长度，最多记录20个模式位置。
    uart_pattern_queue_reset(EX_UART_NUM, 20);

    // Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 3072, NULL, 3, NULL);
    xTaskCreate(task_motor_comm, "task_motor_comm", 8012, NULL, 4, NULL);
}

void vApplicationIdleHook(void)
{
    // idle_hook里面不能用while死循环，也不能用延时，应为是回调函数
}
