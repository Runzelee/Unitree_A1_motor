#include "unitree_motor.h"
#include <string.h>
#include "tim.h" 

static UART_HandleTypeDef *motor_huart;

// CRC32 多项式
#define CRC_POLY 0x04C11DB7

// 全局状态变量
UnitreeMotorState g_unitree_motor_state;

// 防冲突标志
volatile uint8_t g_waiting_reply = 0;
volatile uint32_t g_reply_timeout = 0;

// 保存最后一次发送的指令，用于自动重发 (Auto-Polling)
static MotorCmd last_motor_cmd;
static uint8_t has_sent_once = 0;

// 接收相关
// 增加缓冲区大小以容纳更多数据，防止溢出
#define UNITREE_RX_BUF_SIZE 256 
static uint8_t rx_buffer[UNITREE_RX_BUF_SIZE];


// Forward declaration
static uint32_t crc32_core(uint32_t *ptr, uint32_t len);
static uint32_t unitree_crc32_safe(uint8_t *data, int len_bytes);

/**
 * @brief  初始化 Unitree 电机驱动
 * @param  huart: 连接电机的 UART 句柄
 */
void Unitree_Init(UART_HandleTypeDef *huart) {
    motor_huart = huart;
    
    // 初始化状态结构体
    memset(&g_unitree_motor_state, 0, sizeof(g_unitree_motor_state));
    
    // 启动空闲中断接收 (DMA)
    if (motor_huart) {
        // 1. 启动 DMA 接收 (Circular)
        // 使用 ReceiveToIdle_DMA API，它会自动使能 IDLE 中断
        HAL_UARTEx_ReceiveToIdle_DMA(motor_huart, rx_buffer, UNITREE_RX_BUF_SIZE); 
        
        // 2. 额外确保错误中断开启 (PE, FE, ORE)
        // 这样一旦发生 ORE，就会进入 HAL_UART_ErrorCallback
        __HAL_UART_ENABLE_IT(motor_huart, UART_IT_ERR);
    }
}

static void unitree_extract_data(MasterRecvDataPacketV3 *packet) {
    // 1. CRC 校验
    // 接收包 Total 78 bytes, Data 74 bytes, CRC 4 bytes.
    // Word-based CRC check: 74 bytes is not multiple of 4.
    // 参考 Send 逻辑 (30 bytes data check 28 bytes), 这里 check 72 bytes (18 words)
    // 忽略最后 2 字节 (Res/FError)
    uint32_t cal_crc = crc32_core((uint32_t*)packet, 18);
    
    if (cal_crc != packet->CRCdata.u32) {
        // g_unitree_motor_state.invalid_cnt++;
        return;
    }
    
    // 2. 数据搬运
    g_unitree_motor_state.valid = 1;
    g_unitree_motor_state.id = packet->head.motorID;
    g_unitree_motor_state.mode = packet->Mdata.mode;
    g_unitree_motor_state.Temp = (int8_t)packet->Mdata.Temp;
    g_unitree_motor_state.MError = packet->Mdata.MError;
    
    g_unitree_motor_state.T = (float)packet->Mdata.T / 256.0f;
    g_unitree_motor_state.W = (float)packet->Mdata.W / 128.0f;
    g_unitree_motor_state.Pos = (float)packet->Mdata.Pos * 6.2831853f / 16384.0f;
    
    g_unitree_motor_state.footForce = packet->Mdata.Force16;
    
    for(int i=0; i<3; i++) {
        g_unitree_motor_state.gyro[i] = (float)packet->Mdata.gyro[i] * (2000.0f/32768.0f) * (6.2831853f/360.0f); 
        g_unitree_motor_state.acc[i]  = (float)packet->Mdata.acc[i] * 8.0f * (9.80665f/32768.0f);
    }
}


/**
  * @brief  Rx Transfer completed callback (IDLE or DMA TC)
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == motor_huart->Instance) {
        // Circular DMA 模式下:
        // 简单扫描
        // 注意：为防止非对齐访问导致 HardFault (特别是 CRC 计算时强转 uint32_t*)
        // 我们将找到的包 memcpy 到本地对齐的变量中再处理
        for (int i = 0; i < UNITREE_RX_BUF_SIZE - sizeof(MasterRecvDataPacketV3); i++) {
            if (rx_buffer[i] == 0xFE && rx_buffer[i+1] == 0xEE) {
                // 1. 拷贝到栈上的对齐空间
                MasterRecvDataPacketV3 packet_copy;
                memcpy(&packet_copy, &rx_buffer[i], sizeof(MasterRecvDataPacketV3));
                
                // 2. 检查 ID
                if (packet_copy.head.motorID < 10) {
                     unitree_extract_data(&packet_copy);
                }
            }
        }     
        
        // [防冲突] 收到回复，标志总线空闲，允许下一次发送
        g_waiting_reply = 0;
    }
}

/**
 * @brief 串口错误回调 (处理 Overrun 等错误)
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == motor_huart->Instance) {
        // 清除错误标志
        volatile uint32_t isrflags = huart->Instance->SR;
        volatile uint32_t dr = huart->Instance->DR;
        (void)isrflags; (void)dr;
        
        __HAL_UART_CLEAR_OREFLAG(huart);
        
        // 错误导致接收停止 (State = READY)，必须重启接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer, UNITREE_RX_BUF_SIZE);
        
        g_unitree_motor_state.invalid_cnt++;
        
        // [防冲突] 出错也视为本次事务结束，解除等待，允许重试
        g_waiting_reply = 0;
    }
}


/**
 * @brief 发送控制指令
 * @param cmd 用户构建的 MotorCmd
 */
void Unitree_SendCmd(MotorCmd *cmd) {
    if (!motor_huart) return;
    
    // 保存指令内容以便自动重发
    last_motor_cmd = *cmd;
    has_sent_once = 1;
    
    // [防冲突] 标记正在等待回复，设置超时 (5ms)
    g_waiting_reply = 1;
    g_reply_timeout = 5;
    
    MasterComdDataV3 packet;
    memset(&packet, 0, sizeof(packet));
    
    // 1. Fill Head
    packet.head.start[0] = 0xFE;
    packet.head.start[1] = 0xEE;
    packet.head.motorID = cmd->id;
    packet.head.reserved = 0x00;
    
    // 2. Fill Data
    packet.Mdata.mode = cmd->mode;
    
    // [SDK Compatibility] 
    // 参考 UnitreeMotorSDK，Standard Control 模式下 ModifyBit 应为 0xFF
    packet.Mdata.ModifyBit = 0xFF; 
    packet.Mdata.ReadBit = 0x00;
    packet.Mdata.reserved = 0x00;
    
    // 参数转换 (Float -> Fixed Point)
    packet.Mdata.T = (int16_t)(cmd->tau * 256.0f);
    packet.Mdata.W = (int16_t)(cmd->dq * 128.0f);
    packet.Mdata.Pos = (int32_t)(cmd->q * 16384.0f / 6.2831853f);
    
    packet.Mdata.K_P = (int16_t)(cmd->kp * 2048.0f);
    packet.Mdata.K_W = (int16_t)(cmd->kd * 1024.0f);
    
    packet.Mdata.LowHzMotorCmdIndex = 0;
    packet.Mdata.LowHzMotorCmdByte = 0;
    
    // 3. CRC32
    packet.CRCdata.u32 = unitree_crc32_safe((uint8_t*)&packet, 30);
    
    // 4. Send
    // 使用阻塞发送 (~70us @ 4.8Mbps)，确保在 SysTick 中发送完整。
    HAL_UART_Transmit(motor_huart, (uint8_t*)&packet, sizeof(packet), 10);
}


/**
 * @brief 周期性轮询函数 (建议在 SysTick 或定时器中调用)
 *        用于维持与电机的闭环通信 (Heartbeat/Polling)
 */
void Unitree_Poll(void) {   
    if (has_sent_once) {
        if (g_waiting_reply == 0) {
            // [Ping-Pong] 空闲：发送
            Unitree_SendCmd(&last_motor_cmd);
        } else {
            // [Timeout] 等待中：倒计时
            if (g_reply_timeout > 0) {
                g_reply_timeout--;
            } else {
                // 超时强制复位，允许重发
                g_waiting_reply = 0;
            }
        }
    }
}

/**
 * @brief 定时器中断回调
 *        将电机轮询逻辑挂载到 TIM6 (200Hz) 上，实现与 SysTick/RTOS 的解耦。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        Unitree_Poll();
    }
}


/**
 * @brief  Unitree Go1/A1 参考代码使用的 CRC32 算法 (Word-based)
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

static uint32_t unitree_crc32_safe(uint8_t *data, int len_bytes) {
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
