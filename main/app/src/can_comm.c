#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "can_comm.h"
#include "driver/twai.h"
#include "string.h"
#include "sys_twai.h"
#include "esp_log.h"

#define MASTER_TO_SLAVE_LEFT 0x5A2
#define MASTER_TO_SLAVE_RIGHT 0x5A1
#define SLAVE_LEFT_TO_MASTER 0x352
#define SLAVE_RIGHT_TO_MASTER 0x351
#define TX_GPIO_NUM 25
#define RX_GPIO_NUM 26
#define MSG_ID 0x350
#define mask_id ~(0x7FC << 21)
// #define mask_id ~(0x000 << 21)
#define TAG "CAN_COMM"

static QueueHandle_t can_rx_queue;
MOTOR_PARAMS motor_params_;
can_tx_data_t can_tx_data_;
EventGroupHandle_t aging_send_event;
EventGroupHandle_t sync_event;
uint32_t total_steps;
uint8_t jibu_flag;
// 通信上下文结构
typedef struct
{
    uint32_t expected_response_id;  // 期望的响应ID
    twai_message_t response_msg;    // 接收到的响应
    EventGroupHandle_t event_group; // 事件组
    SemaphoreHandle_t mutex;        // 互斥锁
    uint8_t retry_count;            // 重试计数
    uint8_t data[8];
    uint8_t data_len;
} can_communication_t;
// 1. 配置通用配置：GPIO、模式等
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
// 2. 配置定时配置：波特率。必须与发送端匹配，这里以 500kbps 为例
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // TWAI_TIMING_CONFIG_500KBITS  TWAI_TIMING_CONFIG_1MBITS
// 3. 配置过滤器：这里设置接收所有消息（ID 0x0 - 0x7FF）
twai_filter_config_t f_config = {.acceptance_code = (MSG_ID << 21), // 验收码，将MSG_ID 左移21位，放到寄存器的正确位置
                                 .acceptance_mask = mask_id,        // 匹配高9位，低2位任意
                                                                    // 0x7FC << 21:
                                                                    // 0x7FC = 0000 0000 0000 0000 0000 0111 1111 1100 (二进制)
                                                                    // 左移21位: 1111 1111 1000 0000 0000 0000 0000 0000

                                 // ~(0x7FC << 21):
                                 // 取反后: 0000 0000 0111 1111 1111 1111 1111 1111
                                 // 0x350 <<21 = 0110 1010 0000 0000 0000 0000 0000 0000
                                 // 0x351 <<21 = 0110 1010 0010 0000 0000 0000 0000 0000
                                 // 0x352 <<21 = 0110 1010 0100 0000 0000 0000 0000 0000
                                 // 只关心高9位，低2位任意，也就是说 0x350 0x351 0x352 0x353都可以接收
                                 .single_filter = true};

// 创建通信上下文
can_communication_t *can_comm_create(void)
{
    can_communication_t *comm = malloc(sizeof(can_communication_t));
    if (!comm)
        return NULL;

    comm->event_group = xEventGroupCreate();
    comm->mutex = xSemaphoreCreateMutex();
    if (!comm->event_group || !comm->mutex)
    {
        free(comm);
        return NULL;
    }
    aging_send_event = xEventGroupCreate();
    comm->retry_count = 0;
    memset(&comm->response_msg, 0, sizeof(twai_message_t));
    return comm;
}

// 接收任务函数
void can_receive_task(void *pvParameters)
{
    can_communication_t *comm = (can_communication_t *)pvParameters;
    twai_message_t rx_message;
    while (1)
    {
        // 等待接收消息
        if (can_receive_message(&rx_message, portMAX_DELAY) == ESP_OK)
        {

            uint8_t cmd_parser_ = rx_message.data[0];
            switch (cmd_parser_)
            {
            case 0x11: // 回复通信确认
            case 0x06: // 待机确认回复
            case 0x07: // 睡眠确认回复
            case 0x72: // 回复输入标定
            case 0x73: // 回复输出标定
                if (xSemaphoreTake(comm->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    if (rx_message.identifier == comm->expected_response_id)
                    {
                        if (xQueueSend(can_rx_queue, &rx_message, 0) != pdTRUE)
                        {
                            //ESP_LOGD(TAG, "接收队列已满，丢弃响应");
                        }

                        // 设置事件标志
                        xEventGroupSetBits(comm->event_group, RESPONSE_RECEIVED_BIT);
                    }
                    xSemaphoreGive(comm->mutex);
                }
                else
                {
                    //ESP_LOGD(TAG, "收到其他消息,ID: 0x%03X", rx_message.identifier);
                }
                break;
            case 0x71:                              // 上传计步
                if (rx_message.identifier == 0x351) // 右电机
                {
                    uint8_t temp_step = rx_message.data[1];
                    if (temp_step >= motor_params_.r_pre_steps)
                        motor_params_.r_cur_steps += temp_step - motor_params_.r_pre_steps;
                    else
                        motor_params_.r_cur_steps += 255 - motor_params_.r_pre_steps + temp_step;
                    // char addr_str[10] = {0};
                    // sprintf(addr_str, "%02X:%02X", rx_message.data[0], rx_message.data[1]);
                    //ESP_LOGD(TAG, "r_cur_steps: %d,r_pre_steps =%d", motor_params_.r_cur_steps, motor_params_.r_pre_steps);
                    motor_params_.r_pre_steps = temp_step;
                }
                else if (rx_message.identifier == 0x352) // 左电机
                {
                    uint8_t temp_step = rx_message.data[1];
                    if (temp_step >= motor_params_.l_pre_steps)
                        motor_params_.l_cur_steps += temp_step - motor_params_.l_pre_steps;
                    else
                        motor_params_.l_cur_steps += 255 - motor_params_.l_pre_steps + temp_step;
                    // char addr_str[10] = {0};
                    // sprintf(addr_str, "%02X:%02X", rx_message.data[0], rx_message.data[1]);
                    //ESP_LOGD(TAG, "l_cur_steps: %d,l_pre_steps =%d", motor_params_.l_cur_steps, motor_params_.l_pre_steps);
                    motor_params_.l_pre_steps = temp_step;
                }
                else
                {
                }
                total_steps = motor_params_.l_cur_steps + motor_params_.r_cur_steps;
                jibu_flag = 1;
                break;
            case 0x12: // 要求输入标定数据
            case 0x13: // 要求输出标定数据
                break;
            case 0x14: // 告诉主机已经初始化完成
                break;
            case 0x31: // 报错
                break;

            default:
                if (rx_message.identifier == 0x7f1) // 左或者右
                {
                    motor_params_.l_angle = rx_message.data[1];
                    char addr_str[25] = {0};
                    xEventGroupSetBits(aging_send_event, WIFI_RESPONSE_BIT);
                    sprintf(addr_str, "%02X:%02X", rx_message.data[0], rx_message.data[1]);
                    // //ESP_LOGD(TAG, "收到其他响应，数据长度: %d,数据内容 =%s", rx_message.data_length_code, addr_str);
                }
                else if (rx_message.identifier == 0x7f2) // 左或者右
                {
                    xEventGroupSetBits(aging_send_event, WIFI_RESPONSE_BIT);
                    motor_params_.r_angle = rx_message.data[1];
                }
                else
                {
                    char addr_str[25] = {0};
                    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", rx_message.data[0], rx_message.data[1],
                            rx_message.data[2], rx_message.data[3], rx_message.data[4], rx_message.data[5], rx_message.data[6], rx_message.data[7]);
                    //ESP_LOGD(TAG, "收到其他响应，数据长度: %d,数据内容 =%s", rx_message.data_length_code, addr_str);
                }
                break;
            }
        }
        // vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 发送请求并等待响应（带超时和重试）
esp_err_t can_send_with_response(can_communication_t *comm,
                                 uint32_t send_id, uint8_t *send_data, uint8_t send_len,
                                 uint32_t expected_response_id, uint8_t *response_data,
                                 uint8_t *response_len, uint32_t timeout_ms)
{
    if (xSemaphoreTake(comm->mutex, portMAX_DELAY) != pdTRUE)
    {
        return ESP_FAIL;
    }

    // 设置期望的响应ID
    comm->expected_response_id = expected_response_id;
    memset(&comm->response_msg, 0, sizeof(twai_message_t));

    xSemaphoreGive(comm->mutex);

    // 清除事件标志
    xEventGroupClearBits(comm->event_group, RESPONSE_RECEIVED_BIT);

    for (comm->retry_count = 0; comm->retry_count < MAX_RETRY_COUNT; comm->retry_count++)
    {
        //ESP_LOGD(TAG, "发送请求并等待响应  send_id = ID: 0x%lx, 重试: %d/%d\n",
                //  send_id, comm->retry_count + 1, MAX_RETRY_COUNT);

        // 发送请求消息
        if (can_send_message(send_id, Standard_Frame, send_data, send_len) != ESP_OK)
        {
            //ESP_LOGE(TAG, "发送请求失败,重试");
            continue;
        }

        // 等待响应或超时
        EventBits_t bits = xEventGroupWaitBits(comm->event_group,
                                               RESPONSE_RECEIVED_BIT,
                                               pdTRUE, pdTRUE, pdMS_TO_TICKS(timeout_ms));

        if (bits & RESPONSE_RECEIVED_BIT)
        {
            // 成功接收到响应
            if (xSemaphoreTake(comm->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                twai_message_t rx_message;
                if (xQueueReceive(can_rx_queue, &rx_message, 0) == pdTRUE)
                {
                    //ESP_LOGD(TAG, "收到响应，数据长度: %d", rx_message.data_length_code);
                    // 复制响应数据
                    if (response_data != NULL && response_len != NULL)
                    {
                        *response_len = rx_message.data_length_code;
                        memcpy(response_data, rx_message.data, rx_message.data_length_code);
                    }
                    xSemaphoreGive(comm->mutex);
                }
                //ESP_LOGD(TAG, "成功接收到响应\n");
                return ESP_OK;
            }
            else
            {
                //ESP_LOGD(TAG, "响应超时，准备重试\n");
            }
        }
    }
    //ESP_LOGE(TAG, "请求失败，已达到最大重试次数\n");
    return ESP_ERR_TIMEOUT;
}

void can_send_without_response(uint32_t send_id, uint8_t *send_data, uint8_t send_len)
{

    // 发送请求消息
    if (can_send_message(send_id, Standard_Frame, send_data, send_len) != ESP_OK)
    {
        //ESP_LOGE(TAG, "无响应消息发送失败");
    }
}

// 示例回调函数
void communication_callback(can_communication_t *comm, esp_err_t result,
                            uint8_t *response_data, uint8_t response_len)
{
    if (result == ESP_OK)
    {
        // printf("通信成功！响应数据长度: %d\n", response_len);
        // printf("响应数据: ");
        for (int i = 0; i < response_len; i++)
        {
            printf("%02X ", response_data[i]);
        }
        printf("\n");
    }
    else
    {
        printf("通信失败！错误码: 0x%x\n", result);
    }
}

// 1.上电通信，给两个电机发送信号，并等待接收。如果接收到改变状态
// 2. 给左右电机发送通信请求指令，并增加等待接收的回调函数
// 3. 在接收过程中实时检测通信的状态。
void task_motor_comm(void *args)
{
    if (Twai_Init(&g_config, &t_config, &f_config) != ESP_OK)
    {
        //ESP_LOGE(TAG, "CAN初始化失败\n");
        vTaskDelete(NULL);
        return;
    }
    // 创建通信上下文
    can_communication_t *comm = can_comm_create();
    if (!comm)
    {
        //ESP_LOGE(TAG, "创建通信上下文失败\n");
        return;
    }

    sync_event = xEventGroupCreate();
    can_rx_queue = xQueueCreate(5, sizeof(twai_message_t));
    // 创建接收任务
    xTaskCreate(can_receive_task, "CAN_RX", 4096, comm, 5, NULL);
    //ESP_LOGD(TAG, "can 初始化成功\n");
    vTaskDelay(pdMS_TO_TICKS(200));
    uint8_t response_len = 0;
    uint8_t length = 0;
    uint8_t senddata[8] = {0};
    while (1)
    {
        // 1.定时发送，传感器信息
        // 2.当有触发事件时，如通信自检，获取标定参数，整机状态,需要触发发送
        // 3.接收从机消息，例如上传计步，报错。需要反馈给其他任务。
        EventBits_t sync_bits = xEventGroupWaitBits(sync_event,          /*等待时间标志组句柄*/
                                                    CAN_EVENTBIT_ALL,    /* 等待的事件*/
                                                    pdTRUE,              /* 函数退出时清零等待的事件 */
                                                    pdFALSE,             /* 不需要等待事件中的所有事件 */
                                                    pdMS_TO_TICKS(100)); /* 等待时间 */

        if (sync_bits & MOTOR_START) // 开始运行
        {
            senddata[0] = 0x08;
            length = 1;
            uint8_t response_data[8] = {0};

            esp_err_t result = can_send_with_response(
                comm,
                MASTER_TO_SLAVE_LEFT,              // 请求ID
                senddata,                          // 请求数据
                length,                            // 数据长度
                SLAVE_LEFT_TO_MASTER,              // 期望的响应ID
                response_data,                     // 响应数据缓冲区
                &response_len,                     // 响应数据长度
                pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS) // 超时时间
            );
            result = can_send_with_response(
                comm,
                MASTER_TO_SLAVE_RIGHT,             // 请求ID
                senddata,                          // 请求数据
                length,                            // 数据长度
                SLAVE_RIGHT_TO_MASTER,             // 期望的响应ID
                response_data,                     // 响应数据缓冲区
                &response_len,                     // 响应数据长度
                pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS) // 超时时间
            );
        }
        else if (sync_bits & MOTOR_STOP) // 停止
        {
            senddata[0] = 0x06;
            length = 1;
            uint8_t response_data[8] = {0};

            esp_err_t result = can_send_with_response(
                comm,
                MASTER_TO_SLAVE_LEFT,              // 请求ID
                senddata,                          // 请求数据
                length,                            // 数据长度
                SLAVE_LEFT_TO_MASTER,              // 期望的响应ID
                response_data,                     // 响应数据缓冲区
                &response_len,                     // 响应数据长度
                pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS) // 超时时间
            );
            result = can_send_with_response(
                comm,
                MASTER_TO_SLAVE_RIGHT,             // 请求ID
                senddata,                          // 请求数据
                length,                            // 数据长度
                SLAVE_RIGHT_TO_MASTER,             // 期望的响应ID
                response_data,                     // 响应数据缓冲区
                &response_len,                     // 响应数据长度
                pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS) // 超时时间
            );
        }

        xEventGroupClearBits(sync_event, CAN_EVENTBIT_ALL);
        // 调用回调函数处理结果
        // communication_callback(comm, result, response_data, response_len);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
