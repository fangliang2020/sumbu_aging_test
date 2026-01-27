#ifndef SYS_TWAI_H
#define SYS_TWAI_H


typedef enum
{
    Standard_Frame = 0x00,         /**< No message flags (Standard Frame Format) */
    Extended_Frame = 0x01,         /**< Extended Frame Format (29bit ID) */
    Remote_Frame = 0x02,           /**< Message is a Remote Frame */
    Single_Shot_Trans = 0x04,      /**< Transmit as a Single Shot Transmission. Unused for received. */
    Self_Reception_Request = 0x08, /**< Transmit as a Self Reception Request. Unused for received. */
    Dlc_Non_Comp = 0x10,           /**< Message's Data length code is larger than 8. This will break compliance with TWAI */
} TWAI_MSG_FLAG;

typedef enum
{
    TX_SEND_PINGS,
    TX_SEND_START_CMD,
    TX_SEND_STOP_CMD,
    TX_TASK_EXIT,
} tx_task_action_t;

typedef enum
{
    RX_RECEIVE_PING_RESP,
    RX_RECEIVE_DATA,
    RX_RECEIVE_STOP_RESP,
    RX_TASK_EXIT,
} rx_task_action_t;

esp_err_t Twai_Init(twai_general_config_t *g_config_,twai_timing_config_t *t_config_,twai_filter_config_t *f_config_);
void Twai_Release();
esp_err_t can_send_message(uint32_t ID, TWAI_MSG_FLAG msg_flag, uint8_t *data, uint8_t data_len);
esp_err_t can_receive_message(twai_message_t* message, TickType_t timeout_ticks);
void Twai_Receive();
void TwaiReceive_Register_Handle(void (*handle)(uint8_t *buf, uint8_t len));
#endif
