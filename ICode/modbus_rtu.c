#include "modbus_rtu.h"
#include "usart.h"
#include "bytes.h"
#include "tim.h"

#define RX_MIN_LEN     7          /* Head(2)+Dev(1)+Cmd(1)+Ch(1)+Chk(1)+Foot(2) */

extern float zBuf[FFT_N_Z];
extern uint16_t g_cfg_freq_hz;
extern uint16_t g_cfg_points;
extern uint16_t wave_points;   // 波形发送

extern void Z_Calib_Z_Upright_Neg1G(uint16_t *adcBuf, uint32_t N);
	
extern float* getZBuf(void);

extern uint8_t LOCAL_DEVICE_ADDR;

volatile uint8_t g_tx_busy;

uint8_t uid_me[12];
static inline void UID_Fill_BE_w0w1w2(uint8_t out[12])
{
    uint32_t w0 = HAL_GetUIDw0();
    uint32_t w1 = HAL_GetUIDw1();
    uint32_t w2 = HAL_GetUIDw2();

    /* 大端：高位在前；顺序固定为 w0 | w1 | w2 */
    out[0]  = (uint8_t)(w0 >> 24);
    out[1]  = (uint8_t)(w0 >> 16);
    out[2]  = (uint8_t)(w0 >>  8);
    out[3]  = (uint8_t)(w0 >>  0);

    out[4]  = (uint8_t)(w1 >> 24);
    out[5]  = (uint8_t)(w1 >> 16);
    out[6]  = (uint8_t)(w1 >>  8);
    out[7]  = (uint8_t)(w1 >>  0);

    out[8]  = (uint8_t)(w2 >> 24);
    out[9]  = (uint8_t)(w2 >> 16);
    out[10] = (uint8_t)(w2 >>  8);
    out[11] = (uint8_t)(w2 >>  0);
}

uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;           // 初始值
    const uint16_t poly = 0xA001;    

    while (length--) {
        crc ^= *data++;              // 异或
        for (uint8_t i = 0; i < 8; ++i) {
            if (crc & 0x0001) {      // 最低位
                crc = (crc >> 1) ^ poly;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;                     
}

static HAL_StatusTypeDef uart3_send_dma(uint8_t *buf, uint16_t len)
{
    if (g_tx_busy)
        return HAL_BUSY;

    g_tx_busy = 1;
    return HAL_UART_Transmit_DMA(&huart3, buf, len);
}

/**********************************特征值应答**********************************/
static void send_feature_pkt(uint8_t dev_id,
                              const AxisFeatureValue *X_data,
                              const AxisFeatureValue *Y_data,
                              const AxisFeatureValue *Z_data,
															float temperature)
{
    static uint8_t tx[77];  // 3 + 16 + 16 +36 + 4 + 2 = 77 (三个结构体数据) + CRC
    uint8_t *p = tx;
		memset(tx, 0, sizeof(tx));

    *p++ = dev_id;
    *p++ = CMD_FEATURE;
    *p++ = 0x48;

    // ----- X_data 区域：4 × float (16B) -----
		put_be_f32(&p, X_data->mean);
    put_be_f32(&p, X_data->rms);
    put_be_f32(&p, X_data->pp);
    put_be_f32(&p, X_data->kurt);

    // ----- Y_data 区域：4 × float (16B) -----
    put_be_f32(&p, Y_data->mean);
    put_be_f32(&p, Y_data->rms);
    put_be_f32(&p, Y_data->pp);
    put_be_f32(&p, Y_data->kurt);

    // ----- Z_data 区域：9 × float (36B) -----
    put_be_f32(&p, Z_data->mean);
    put_be_f32(&p, Z_data->rms);
    put_be_f32(&p, Z_data->pp);
    put_be_f32(&p, Z_data->kurt);
    put_be_f32(&p, Z_data->peakFreq);
    put_be_f32(&p, Z_data->peakAmp);
    put_be_f32(&p, Z_data->amp2x);
    put_be_f32(&p, Z_data->envelope_vrms);
    put_be_f32(&p, Z_data->envelope_peak);
		 // -----  temp 区域 -----
    put_be_f32(&p, temperature);
		
		size_t payload_len = (size_t)(p - tx);      // 已写入的真实字节数
    uint16_t crc = Modbus_CRC16(tx, payload_len);  
    *p++ = crc & 0xFF;        
    *p++ = (crc >> 8) & 0xFF; 

//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
		uart3_send_dma(tx, (uint16_t)(p - tx));	
}

/* 测试用：发送特征包，数据区用 00,11,22,...,FF 循环填充 */
static void send_feature_pkt_test(uint8_t dev_id)
{
    enum { HEADER_LEN = 3, DATA_LEN = 72, CRC_LEN = 2, FRAME_LEN = HEADER_LEN + DATA_LEN + CRC_LEN };
    static uint8_t tx[FRAME_LEN];
    uint8_t *p = tx;

    // 头部：dev_id, CMD_FEATURE, 固定 0x48
    *p++ = dev_id;
    *p++ = CMD_FEATURE;
    *p++ = 0x48;

    // 数据区：72 字节固定模式填充（00,11,22,...,FF 循环）
    for (uint32_t i = 0; i < DATA_LEN; ++i) {
        *p++ = (uint8_t)((i & 0x0Fu) * 0x11u);
        // 若你只想在 0x00,0x11,0x22,0x33 四值间循环，可改为：
        // *p++ = (uint8_t)(((i % 4u) * 0x11u) & 0xFFu);
    }

    uint16_t crc = Modbus_CRC16(tx, (size_t)(p - tx));
    *p++ = (uint8_t)(crc & 0xFF);
    *p++ = (uint8_t)((crc >> 8) & 0xFF);

    // 发送实际帧长（应为 77 字节）
//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
		uart3_send_dma(tx, (uint16_t)(p - tx));	
}
/**********************************波形应答**********************************/
static void send_wave_ack(uint8_t dev_id)
{
    static uint8_t tx[7]; 
    uint8_t *p = tx;

    *p++ = dev_id;
    *p++ = CMD_WAVE;    // 
    *p++ = 0x02;        // LEN: 数据长度为2 (即后面跟着的 b2 和 b3)
    *p++ = 0x4F;          // 
    *p++ = 0x4B;          // 返回 ok

    // 计算 CRC
    uint16_t crc = Modbus_CRC16(tx, (size_t)(p - tx));
    *p++ = (uint8_t)(crc & 0xFF);        // Low
    *p++ = (uint8_t)((crc >> 8) & 0xFF); // High

//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
		uart3_send_dma(tx, (uint16_t)(p - tx));	
}

/* ---- 协议常量 ---- */
enum { PTS_PER_PKT   = 64 };                     // 每包 64 点
enum { HEADER_NOCRC  = 4  };                     // dev_id(1) + CMD_WAVE(1) + seq(1) + total_pkts(1) 
enum { DATA_LEN      = PTS_PER_PKT * 4 };        // 64 * 4 = 256
enum { FRAME_NOCRC   = HEADER_NOCRC + DATA_LEN };// 4 + 256 = 260
enum { CRC_LEN       = 2  };
enum { FRAME_LEN     = FRAME_NOCRC + CRC_LEN };  // 260 + 2 = 262

/* 帧：dev_id | CMD_WAVE  | seq(1B) |total_pkts(1B) | 64×float(BE) | CRC(LE) */
static void send_wave_pkt(uint8_t dev_id, const float *buf, uint8_t total_pkts, uint8_t seq)
{
    if (!buf) return;

    static uint8_t tx[FRAME_LEN]; // FRAME_LEN = 262

		uint8_t *p = tx;
		uint32_t offset = (uint32_t)seq * PTS_PER_PKT; 
	
		/* 头部 4B */
		*p++ = dev_id;        // 1B
		*p++ = CMD_WAVE;      // 1B
		*p++ = seq;           // 1B，当前序号
		*p++ = total_pkts;    // 1B，总包数


    /* 数据区：64 个 float，按大端写入；末包不足补 0.0f */
		for (uint16_t i = 0; i < PTS_PER_PKT; ++i) {
        float v = buf[offset + i]; 
        put_be_f32(&p, v); // 大端模式写入
    }

		uint16_t crc = Modbus_CRC16(tx, (size_t)(p - tx));
		*p++ = (uint8_t)(crc & 0xFF);        // Low
		*p++ = (uint8_t)((crc >> 8) & 0xFF); // High

//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
		uart3_send_dma(tx, (uint16_t)(p - tx));	
}

static void dump_uid(const char* tag, const uint8_t* p) {
    printf("%s:", tag);
    for (int i = 0; i < 12; ++i) printf(" %02X", p[i]);
    printf("\r\n");
}

/**********************************广播发现应答**********************************/
static void send_discover_rsp(uint8_t cur_addr)
{
    static uint8_t tx[1 + 1 + 1 + 13 + 2];
    uint8_t *p = tx;

		uint8_t uid[12];
    UID_Fill_BE_w0w1w2(uid);      // ← 统一构造（大端，w0|w1|w2）

    *p++ = cur_addr;            // 我自己的地址（若尚未配置就是 0x00）
    *p++ = CMD_DISCOVER;        // 回同一命令码
    *p++ = 13;                  // len = 12B UID + 1B addr
//		dump_uid("uid", uid);
    memcpy(p, uid, 12); p += 12;
    *p++ = cur_addr;

    uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
    *p++ = (uint8_t)(crc & 0xFF);
    *p++ = (uint8_t)(crc >> 8);

    /* 简单退避，避免广播回包碰撞 */
    HAL_Delay(uid[11] & 0x0Fu);

//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
		uart3_send_dma(tx, (uint16_t)(p - tx));	
}

/**********************************配置地址应答**********************************/
// 只认大端 UID，CRC 仍小端
static bool HandleSetAddr_Broadcast(const uint8_t* rx, uint16_t flen)
{
    if (flen < 2 + 12 + 1) return false; // dev|cmd|UID12|newAddr 最小长度

    if (rx[0] != 0x00 || rx[1] != CMD_SET_ADDR) return false;

    uint8_t off = 2;
    if (flen >= 3 + 12 + 1 && rx[2] == 0x0D) off = 3;

    const uint8_t* uid_in   = &rx[off];
    const uint8_t  new_addr = rx[off + 12];

    // 大端 UID 完全匹配
		static uint8_t uid_me[12];
    UID_Fill_BE_w0w1w2(uid_me);                // 本机 UID（大端，w0|w1|w2）
//		dump_uid("uid_in", uid_in);
//		dump_uid("uid_me", uid_me);
    if (memcmp(uid_in, uid_me, 12) != 0) return true;

//    if (new_addr == 0x00 || new_addr == 0xFF) return true;

    // 幂等：一样就只回 ACK
    if (LOCAL_DEVICE_ADDR == new_addr) {
        static uint8_t tx[1+1+1+1+2];
				uint8_t *p = tx;
        *p++ = new_addr; *p++ = CMD_SET_ADDR; *p++ = 1; *p++ = new_addr;
        uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
        *p++ = (uint8_t)crc; *p++ = (uint8_t)(crc >> 8);
		//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
				uart3_send_dma(tx, (uint16_t)(p - tx));	
        return true;
    }

    if (Flash_WriteDeviceAddr(new_addr) == HAL_OK) {
        LOCAL_DEVICE_ADDR = new_addr;

        static uint8_t tx[1+1+1+1+2];
				uint8_t *p = tx;
        *p++ = new_addr; *p++ = CMD_SET_ADDR; *p++ = 1; *p++ = new_addr;
        uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
        *p++ = (uint8_t)crc; *p++ = (uint8_t)(crc >> 8);
		//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
				uart3_send_dma(tx, (uint16_t)(p - tx));	
    }
    return true;
}


/**********************************解析配置帧**********************************/
static void Config_ParseAndApply_Freq(const uint8_t* rx)
{
		uint16_t f = rd_be16(&rx[3]);        // dev|cmd|sub 之后 4 字节
		if (f == 0) return;             // 0 无效，直接忽略
		if (f > FLASH_CFG_DEFAULT_FREQ) return;
		if (f == g_cfg_freq_hz) return;
    if (Flash_UpdateFreq(f) == HAL_OK) 
		{
        g_cfg_freq_hz = f;
        ACQ_SetFreqHz(g_cfg_freq_hz);
    }
}

static void Config_ParseAndApply_Point(const uint8_t* rx)
{
		uint16_t pts = rd_be16(&rx[3]);     // dev|cmd|sub 之后 2 字节
    if (pts == 0) return;
		if (pts > FLASH_CFG_DEFAULT_POINTS) return;
    if (pts == g_cfg_points) return;

    if (Flash_UpdatePoints(pts) == HAL_OK) {
        g_cfg_points = pts;
        wave_points  = pts;            
    }
}
/**********************************采样配置应答**********************************/
static void Config_SendAck(uint8_t dev_id)
{
    static uint8_t tx[3 + 2 + 2 + 2];      // dev|cmd|len|freq2|points2|crc2
    uint8_t *p = tx;

    *p++ = dev_id;
    *p++ = CMD_CONFIG;              // 0x00（你头文件里的定义）
    *p++ = 4;                       // payload 长度

	  uint16_t freq16 = (uint16_t)g_cfg_freq_hz;
    uint16_t pts    = (uint16_t)g_cfg_points;

    wr_be16(p, freq16); p += 2;
    wr_be16(p, pts ); p += 2;

    uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
    *p++ = (uint8_t)crc;
    *p++ = (uint8_t)(crc >> 8);

//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
		uart3_send_dma(tx, (uint16_t)(p - tx));	
}
/**********************************校准配置应答**********************************/
static void CALIBRATION_Config_SendAck(uint8_t dev_id)
{
		static uint8_t tx[5];
		uint8_t *p = tx;
		*p++ = dev_id;
		*p++ = CMD_CALIBRATION;
		*p++ = 0x00;   // 0 = OK
		
	  uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
    *p++ = (uint8_t)crc;
    *p++ = (uint8_t)(crc >> 8);
//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
		uart3_send_dma(tx, (uint16_t)(p - tx));	
}
/**********************************帧处理**********************************/
void Protocol_HandleRxFrame(const uint8_t *rx, uint16_t len, uint8_t local_address)
{
    if (len < RX_MIN_LEN)                         { return; }
//    if (rx[0]!=PKT_HEAD_H || rx[1]!=PKT_HEAD_L)   { return; }
//    if (rx[len-2]!=PKT_FOOT_H || rx[len-1]!=PKT_FOOT_L) { return; }
//    if (checksum8(rx, len-3) != rx[len-3])        {             /* CRC 错 */
//        Protocol_SendNack(rx[2], rx[3], PKT_ERR_CRC);
//        return;
//    }

    uint8_t dev_id = rx[0];                       // 提取请求中的设备地址
    uint8_t cmd    = rx[1];
		uint8_t b2   	 = rx[2];
		uint8_t b3   	 = rx[3];
    const bool is_broadcast = (dev_id == 0x00);
						
		if (is_broadcast && cmd == CMD_DISCOVER) {
        send_discover_rsp(local_address);  // 回 UID + 当前地址
        return;
    }
		
		if (is_broadcast && cmd == CMD_SET_ADDR) {
        HandleSetAddr_Broadcast(rx, len);
				return;
    }
				
		if (!is_broadcast && dev_id != local_address) 
		{
        return;
    }
				
    switch (cmd)
    {
    case CMD_FEATURE: send_feature_pkt(dev_id, &X_data, &Y_data, &Z_data, Temp); break;
		case CMD_WAVE:memcpy(Tx_Wave_Buffer_Z, g_data_z, sizeof(Tx_Wave_Buffer_Z));send_wave_ack(dev_id); break;
		case CMD_WAVE_PACK:	send_wave_pkt(dev_id, Tx_Wave_Buffer_Z, b2, b3); break;
		case CMD_CONFIG:
			  switch (b2)
        {
        case FREQ: Config_ParseAndApply_Freq(rx);Config_SendAck(dev_id); break;
        case PORINT: Config_ParseAndApply_Point(rx);Config_SendAck(dev_id); break;			
        default: break;
        }
        break;
    case CMD_CALIBRATION:Z_Calib_Z_Upright_Neg1G(ADC_Buffer_Z, 100);CALIBRATION_Config_SendAck(dev_id); break;
    case CMD_TEST: 
        switch (b2)
        {
        case CH_X: print_g_data(g_data_x,200); break;
        case CH_Y: print_g_data(g_data_y,200); break;
        case CH_Z: print_g_data(g_data_z,200); break;
				case CH_FEATURE: print_FEATURE(); break;
        case CH_X3: print_g_data(g_data_x,600); break;
        case CH_Y3: print_g_data(g_data_y,600); break;
        case CH_Z3: print_g_data(g_data_z,600); break;					
        default: break;
        }
        break;	
		default:
//        Protocol_SendNack(dev_id, cmd, PKT_ERR_CMD);            
        break;
    }
}