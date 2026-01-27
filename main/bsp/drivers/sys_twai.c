#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "string.h"
#include "sys_twai.h"
#include "esp_log.h"

#define TAG "TWAI_CAN"

static void (*cmd_handle)(uint8_t *buf, uint8_t len) = NULL;
esp_err_t Twai_Init(twai_general_config_t *g_config_,twai_timing_config_t *t_config_,twai_filter_config_t *f_config_)
{
    // 4. 安装 TWAI 驱动
    esp_err_t ret = twai_driver_install(g_config_, t_config_, f_config_);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install driver: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGD(TAG, "Driver installed successfully.");
    // 5. 启动 TWAI 驱动
    ret = twai_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start driver: %s", esp_err_to_name(ret));
        goto uninstall_driver; // 如果启动失败，跳转到卸载驱动
    }
    ESP_LOGD(TAG, "Driver started successfully.");
    return ret;
// 错误处理：如果启动失败，跳转到这里卸载驱动
uninstall_driver:
    twai_driver_uninstall();
    ESP_LOGD(TAG, "Driver uninstalled.");
    return ret;
}

void Twai_Release()
{
    twai_driver_uninstall();
    ESP_LOGD(TAG, "Driver uninstalled.");
}

esp_err_t can_send_message(uint32_t ID, TWAI_MSG_FLAG msg_flag, uint8_t *data, uint8_t data_len)
{
    // 准备要发送的消息
    twai_message_t message;
    message.identifier = ID;      // CAN 消息 ID (11位标准ID)
    message.flags = msg_flag;     // 标志位，None 表示标准数据帧
    message.data_length_code = 8; // 数据长度 (0-8字节)，这里设为8字节
    memcpy(&message.data, data, 8);
    return twai_transmit(&message, pdMS_TO_TICKS(100));
}

// 接收 CAN 消息（带超时）
esp_err_t can_receive_message(twai_message_t* message, TickType_t timeout_ticks) {
    return twai_receive(message, timeout_ticks);
}


void Twai_Receive()
{
    twai_message_t message;
    int recv_count = 0;
    int ret = twai_receive(&message, pdMS_TO_TICKS(1000));

    if (ret == ESP_OK)
    {
        // 成功接收到消息
        recv_count++;

        // 判断帧类型
        const char *frame_type = "Data";
        if (message.flags & TWAI_MSG_FLAG_RTR)
        {
            frame_type = "Remote";
        }

        // 判断帧格式
        const char *frame_format = "Standard";
        if (message.flags & TWAI_MSG_FLAG_EXTD)
        {
            frame_format = "Extended";
        }

        ESP_LOGD(TAG, "[%d] Received %s %s Frame:", recv_count, frame_format, frame_type);
        ESP_LOGD(TAG, "  ID: 0x%08X", message.identifier);
        ESP_LOGD(TAG, "  DLC: %d", message.data_length_code);

        // 只有数据帧才打印数据内容
        if (!(message.flags & TWAI_MSG_FLAG_RTR) && message.data_length_code > 0)
        {
            ESP_LOGD(TAG, "  Data: ");
            char data_str[50] = {0};
            char temp[10] = {0};

            for (int i = 0; i < message.data_length_code; i++)
            {
                snprintf(temp, sizeof(temp), "%02X ", message.data[i]);
                strcat(data_str, temp);
            }
            ESP_LOGD(TAG, "  %s", data_str);

            // 以十进制形式显示数据（可选）
            ESP_LOGD(TAG, "  Data(dec): ");
            memset(data_str, 0, sizeof(data_str));
            for (int i = 0; i < message.data_length_code; i++)
            {
                snprintf(temp, sizeof(temp), "%3d ", message.data[i]);
                strcat(data_str, temp);
            }
            ESP_LOGD(TAG, "  %s", data_str);
        }
        cmd_handle(message.data, message.data_length_code);
        ESP_LOGD(TAG, "----------------------------------------");
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
        // 接收超时，没有消息到来（这是正常情况，不一定是错误）
        ESP_LOGD(TAG, "No message received within timeout period.");
    }
    else
    {
        // 其他错误
        ESP_LOGE(TAG, "Failed to receive message: %s", esp_err_to_name(ret));
    }
}

void TwaiReceive_Register_Handle(void (*handle)(uint8_t *buf, uint8_t len))
{
    assert(handle);
    if (cmd_handle != NULL)
    {
        ESP_LOGE(TAG, "cmd_handle register failed, handle is exist.");
    }
    cmd_handle = handle;
}
