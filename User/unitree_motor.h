#ifndef __UNITREE_MOTOR_H
#define __UNITREE_MOTOR_H

#include <stdint.h>
#include "stm32f4xx_hal.h" 

// 模拟 SDK 的枚举定义
typedef enum {
    MotorType_A1,
    MotorType_B1,
    MotorType_GO_M8010_6
} MotorType;

typedef enum {
    MotorMode_BRAKE = 0,
    MotorMode_FOC = 10,
    MotorMode_SLOW = 5 
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

// 接收数据主体 (Motor -> Master)
typedef struct {
    uint8_t mode;         // 当前运行模式
    uint8_t Temp;         // 电机温度
    uint8_t MError;       // 错误标识
    uint8_t Read[5];      // 内部参数
    
    int16_t T;            // 当前输出力矩 (T * 256)
    int16_t W;            // 当前转速 (W * 128)
    int32_t LW;           // 滤波转速
    
    int16_t W2;           // 预留
    int32_t LW2;          // 预留
    
    int16_t Acc;          // 当前输出加速度
    int16_t OutAcc;       // 预留
    
    int32_t Pos;          // 当前输出位置 (Pos / (16384/2Pi))
    int32_t Pos2;         // 预留
    
    int16_t gyro[3];      // IMU 角速度
    int16_t acc[3];       // IMU 线加速度
    
    int16_t Fgyro[3];     // 预留
    int16_t Facc[3];      // 预留
    int16_t Fmag[3];      // 预留
    
    uint8_t Ftemp;        // 足端温度
    uint16_t Force16;     // 足端力
    uint8_t Force8;       // 足端力
    uint8_t FError;       // 足端错误
    
    uint8_t Res;          // 预留
    
} MasterRecvDataV3;

// 完整的接收包
typedef struct {
    COMHead head;
    MasterRecvDataV3 Mdata;
    COMData32 CRCdata;    // CRC32
} MasterRecvDataPacketV3;

// 用户友好的状态结构体 (解码后)
typedef struct {
    uint8_t  id;
    uint8_t  mode;
    float    T;        // 力矩 (Nm)
    float    W;        // 速度 (rad/s)
    float    Pos;      // 位置 (rad)
    int8_t   Temp;     // 温度 (C)
    uint8_t  MError;   // 错误码
    int16_t  footForce; // 足端压力
    
    // IMU Data
    float    gyro[3];
    float    acc[3];
    
    uint32_t invalid_cnt; // CRC 错误计数
    uint8_t  valid;       // 数据有效标志
} UnitreeMotorState;

#pragma pack()

extern UnitreeMotorState g_unitree_motor_state;

// 对外接口函数
void Unitree_Init(UART_HandleTypeDef *huart);
void Unitree_SendCmd(MotorCmd *cmd);
void Unitree_Poll(void); // 周期性轮询

float queryGearRatio(MotorType type);
uint8_t queryMotorMode(MotorType type, MotorMode mode);

#endif // __UNITREE_MOTOR_H
