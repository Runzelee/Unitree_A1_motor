#include "unitree_motor.h"
#include <string.h>

static UART_HandleTypeDef *motor_huart;

// CRC32 多项式
#define CRC_POLY 0x04C11DB7

/**
 * @brief  初始化 Unitree 电机驱动
 * @param  huart: 连接电机的 UART 句柄
 */
void Unitree_Init(UART_HandleTypeDef *huart) {
    motor_huart = huart;
}


/**
 * @brief  Unitree Go1/A1 参考代码使用的 CRC32 算法 (Word-based)
 *         注意：该算法在 Little Endian 机器上会以 "Byte3, Byte2, Byte1, Byte0" 的顺序处理每个字。
 * @param  ptr: 数据指针
 * @param  len: 字(uint32)的数量
 * @return CRC32
 */
static uint32_t crc32_core(uint32_t *ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;

    for (uint32_t i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else {
                CRC32 <<= 1;
            }
            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

// 包装函数：为了接口兼容，内部转调 word-based 算法
// 注意：这里忽略 len_bytes，强制使用 7 words (28 bytes) 计算，配合 Reference 逻辑
static uint32_t unitree_crc32_safe(uint8_t *data, int len_bytes) {
    // 确保 4 字节对齐访问，虽然 STM32F4 支持非对齐，但这更安全
    // 输入 data 应当指向 MasterComdDataV3 结构体，它是 4 字节对齐的
    return crc32_core((uint32_t*)data, 7);
}

// 辅助函数
float queryGearRatio(MotorType type) {
    switch (type) {
        case MotorType_A1: return 9.1f;
        case MotorType_B1: return 9.1f; 
        default: return 6.33f; // GO-M8010
    }
}

uint8_t queryMotorMode(MotorType type, MotorMode mode) {
    return (uint8_t)mode;
}

/**
 * @brief 发送控制指令
 * @param cmd 用户构建的 MotorCmd
 */
void Unitree_SendCmd(MotorCmd *cmd) {
    if (!motor_huart) return;
    
    MasterComdDataV3 packet;
    memset(&packet, 0, sizeof(packet));
    
    // 1. Fill Head
    packet.head.start[0] = 0xFE;
    packet.head.start[1] = 0xEE;
    packet.head.motorID = cmd->id;
    packet.head.reserved = 0x00;
    
    // 2. Fill Data
    packet.Mdata.mode = cmd->mode;
    packet.Mdata.ModifyBit = 0x00; 
    packet.Mdata.ReadBit = 0x00;
    packet.Mdata.reserved = 0x00;
    
    // 参数转换 (Float -> Fixed Point)
    // 根据 data_format.md 公式
    packet.Mdata.T = (int16_t)(cmd->tau * 256.0f);
    packet.Mdata.W = (int16_t)(cmd->dq * 128.0f);
    packet.Mdata.Pos = (int32_t)(cmd->q * 16384.0f / 6.2831853f); // 2*PI approx 6.2831853
    
    packet.Mdata.K_P = (int16_t)(cmd->kp * 2048.0f);
    packet.Mdata.K_W = (int16_t)(cmd->kd * 1024.0f);
    
    packet.Mdata.LowHzMotorCmdIndex = 0;
    packet.Mdata.LowHzMotorCmdByte = 0;
    
    // 3. CRC32
    // 校验前 30 字节
    packet.CRCdata.u32 = unitree_crc32_safe((uint8_t*)&packet, 30);
    
    // 4. Send (Total 34 bytes)
    HAL_UART_Transmit(motor_huart, (uint8_t*)&packet, sizeof(packet), 10);
}
