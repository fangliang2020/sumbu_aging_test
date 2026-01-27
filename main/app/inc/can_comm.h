#ifndef CAN_COMM_H
#define CAN_COMM_H
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

// 事件标志位
#define RESPONSE_RECEIVED_BIT (1 << 0)
#define TIMEOUT_BIT (1 << 1)

#define WIFI_RESPONSE_BIT (1 << 0)
// 通信参数
#define RESPONSE_TIMEOUT_MS 100 // 响应超时时间
#define MAX_RETRY_COUNT 3       // 最大重试次数

#define MOTOR_START (1 << 0) //开始
#define MOTOR_STOP (1 << 1) //获取标定参数
#define STATUS_SWITCHING (1 << 2) //状态切换 
#define INTENSITY_MODIFY (1 << 3) //力度改变
#define CAN_EVENTBIT_ALL (MOTOR_START | MOTOR_STOP )
extern EventGroupHandle_t aging_send_event;
extern EventGroupHandle_t sync_event; // 同步事件组
extern uint32_t total_steps; 
extern uint8_t jibu_flag;
typedef enum
{
    Motor_Initial = 0,
    LeftMotor_disconn,  // 左电机断连
    RightMotor_disconn, // 右电机断连
    Dual_Motor_disconn, // 双电机断连
    Dual_Motor_conn,    // 双电机都连接上
} MOTOR_COMM_STATUS;

typedef struct
{
    MOTOR_COMM_STATUS motor_comm_state;

} MOTOR_ARGS;

typedef struct
{
    uint8_t l_angle;
    uint8_t r_angle;
    uint16_t l_cur_steps; //当前的总计步
    uint16_t r_cur_steps;
    uint16_t l_pre_steps; //上次的电机发过来的计步
    uint16_t r_pre_steps;
    uint16_t totle_steps;
} MOTOR_PARAMS;
extern MOTOR_PARAMS motor_params_;


typedef struct
{
    uint8_t Strength_gear;
} can_tx_data_t;

void task_motor_comm(void *args);

#endif // ! CAN_COMM_H
