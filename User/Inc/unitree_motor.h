#ifndef __UNITREE_MOTOR_H
#define __UNITREE_MOTOR_H

#include <stdint.h>
#include "usart.h" // 引用HAL库和UART定义

// 模拟 SDK 的枚举定义
typedef enum {
    MotorType_A1,
    MotorType_B1,
    MotorType_GO_M8010_6
} MotorType;

typedef enum {
    MotorMode_BRAKE = 0,
    MotorMode_FOC = 10,
    MotorMode_CALIBRATE = 11 // 注意 SDK 中可能是其他值，但这里参考 data_format.md: 10: 闭环伺服
} MotorMode;

// 模拟 SDK 的各类参数结构体
// 为了方便 C 语言调用，去除类方法，仅保留数据成员
typedef struct {
    MotorType motorType;
    uint8_t id;           // motorID
    uint8_t mode;         // control mode
    float tau;            // 期望力矩 (Nm)
    float dq;             // 期望角速度 (rad/s)
    float q;              // 期望位置 (rad)
    float kp;             // 位置刚度 (Nm/rad)
    float kd;             // 速度刚度 (Nm/(rad/s))
} MotorCmd;

// 协议数据结构定义 (Refer to data_format.md & motor_msg_A1B1.h)
#pragma pack(1)

// 通用数据联合体，用于方便转换
typedef union {
    int32_t L;
    uint8_t u8[4];
    uint16_t u16[2];
    uint32_t u32;
    float F;
} COMData32;

// 协议头
typedef struct {
    uint8_t start[2];     // 0xFE, 0xEE
    uint8_t motorID;
    uint8_t reserved;
} COMHead;

// 发送指令主体 (Master -> Motor)
typedef struct {
    uint8_t mode;         // 目标模式
    uint8_t ModifyBit;    // 参数修改位
    uint8_t ReadBit;      // 读取位
    uint8_t reserved;
    
    COMData32 Modify;     // 参数修改数据
    
    int16_t T;            // 目标力矩 (T * 256)
    int16_t W;            // 目标速度 (W * 128)
    int32_t Pos;          // 目标位置 (Pos * 16384 / 2Pi)
    
    int16_t K_P;          // (Kp * 2048)
    int16_t K_W;          // (Kw * 1024) - 注意 SDK 命名是 K_W 对应 kd
    
    uint8_t LowHzMotorCmdIndex;
    uint8_t LowHzMotorCmdByte;
    
    COMData32 Res[1];
    
} MasterComdV3;

// 完整的发送包
typedef struct {
    COMHead head;
    MasterComdV3 Mdata;
    COMData32 CRCdata;    // CRC32
} MasterComdDataV3;

#pragma pack()

// 对外接口函数
void Unitree_Init(UART_HandleTypeDef *huart);
void Unitree_SendCmd(MotorCmd *cmd);
float queryGearRatio(MotorType type);
uint8_t queryMotorMode(MotorType type, MotorMode mode);

#endif // __UNITREE_MOTOR_H
