#ifndef _MODBUS_RTU_H_
#define _MODBUS_RTU_H_
#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <stdbool.h>

/*
XY: mean RMS PP 
Z:	mean RMS PP Displacement_PP Envelope_Vrms Envelope_Peak
*/
// 自定义报文结构体
#pragma pack(push, 1)  // 设置结构体的字节对齐为 1 字节（无填充）

typedef struct
{
    uint8_t header[2];         // 报文头（2字节，固定为 AA 55）
    uint8_t device_id;         // 设备 ID（1字节）
    uint8_t command;           // 命令（1字节）
    uint8_t channel;           // 通道（1字节）
	  float temp;                // 温度（4字节）
    float mean;                // 均值（4字节）
    float rms;                 // RMS（4字节）
    float pp;                  // 峰-峰值（4字节）
    float dpp;                 // 位移峰-峰值（4字节）
    float envelope_vrms;       // 包络有效值（4字节）
    float envelope_peak;       // 包络峰值（4字节）
    uint8_t checksum;          // 校验和（1字节）
    uint8_t footer[2];         // 报文尾（2字节，固定为 FF FE）
} FeatureValuePacket;

// 用于传输波形数据的结构体
typedef struct
{
    uint8_t header[2];          // 报文头（2字节，AA 55）
    uint8_t id_h;               // 设备 ID 高字节（1字节）
    uint8_t id_l;               // 设备 ID 低字节（1字节）
    uint8_t msg_type;           // 消息类型（1字节）
    uint8_t channel;            // 通道（1字节）
    uint8_t total_package[2];   // 总包数量（2字节）
    float X[1024];              // 加速度波形（1024字节，每个数据点为float，即 4 字节）
    uint8_t Sample_Rate[2];     // 采样率（2字节）
    uint8_t footer[2];          // 报文尾（2字节，A5 A5）
} WaveformPacket;
extern FeatureValuePacket fvp;
extern WaveformPacket wfp;

#pragma pack(pop)  // 恢复默认的对齐方式

/* ────────── 报文固定字节 ────────── */
#define PKT_HEAD_H    0xAA
#define PKT_HEAD_L    0x55
#define PKT_FOOT_H    0xA5
#define PKT_FOOT_L    0xA5
/* ────────── Command 定义 ────────── */
#define CMD_CONFIG       0x00     /* 配置请求  */
#define CMD_FEATURE      0x02     /* 特征值请求  */
#define CMD_WAVE_PACK    0x03     /* 波形包请求  */
#define CMD_WAVE         0x04     /* 波形请求    */
#define CMD_TEST         0x77     /* 测试请求    */
#define CMD_DISCOVER     0x41   	 /* 主站广播发现*/
#define CMD_SET_ADDR  	 0x42   	 /* 主站广播给某uid配置地址*/
#define CMD_CALIBRATION  0x60   	 /* 校准请求*/
#define CMD_WRONG        0x80     /* 错误 */

/* ────────── TEST Channel 定义 ────────── */
#define CH_X         		 		0x01
#define CH_Y          	 		  0x02
#define CH_Z         		 		0x03
#define CH_FEATURE           0x04
#define CH_TEMP          		0x05
#define CH_FEATURE_TEST      0x06
#define CH_X3         		 		0x07
#define CH_Y3         	 		  0x08
#define CH_Z3        		 		0x09
#define FIXED_THIRD_BYTE    0xFF
/* ────────── CONfIG Command 定义 ────────── */
#define FREQ          	 		0x01
#define PORINT         		 	0x02

/* ────────── Wave 定义 ────────── */
#define WAVE_PKT_POINTS    64u
#define WAVE_TOTAL_POINTS  FFT_N_Z         // = 4096
#define WAVE_SYNC_H        0x55
#define WAVE_SYNC_L        0xAA
#define WAVE_VER           0x01
/* ── 应答返回码（可扩展） ── */
typedef enum {
    PKT_OK        = 0,
    PKT_ERR_CRC   = 1,
    PKT_ERR_CMD   = 2,
    PKT_ERR_ARG   = 3,
    PKT_ERR_ADDR  = 4,   /* 地址不匹配 */
} PktStatus_t;

/* —— 分时回应参数（9600bps 下）——*/
#define CFG_SLOT_COUNT              64      // 槽数
#define CFG_SLOT_LEN_MS             30      // 每槽 30ms（9600bps）
#define CFG_BASE_GUARD_MS           12      // 主站发完/收发切换保护
#define CFG_JITTER_US_MASK          0x07FF  // 抖动上限 ~ 2.0ms

extern volatile uint8_t g_tx_busy;
/* 上位机发来一帧后调用此函数，len=完整帧长度 */
void Protocol_HandleRxFrame(const uint8_t *rx, uint16_t len, uint8_t local_address);
/* 生成并发送 NACK，用于非法命令或 CRC 错误 */
void Protocol_SendNack(uint8_t dev_id, uint8_t cmd, PktStatus_t err);


#endif
