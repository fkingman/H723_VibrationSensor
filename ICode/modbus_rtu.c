#include "modbus_rtu.h"
#include "usart.h"
#include "bytes.h"
#include "tim.h"

#define RX_MIN_LEN     7          /* Head(2)+Dev(1)+Cmd(1)+Ch(1)+Chk(1)+Foot(2) */

extern float zBuf[FFT_N_Z];
extern uint16_t g_cfg_freq_hz;
extern uint16_t g_cfg_points;
extern uint16_t wave_points;   // 波形发送

extern float* getZBuf(void);

extern uint8_t LOCAL_DEVICE_ADDR;

volatile uint8_t g_tx_busy;

static uint32_t s_received_bytes = 0;

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
    uint32_t tickstart = HAL_GetTick();   
    while (g_tx_busy)
    {
        if ((HAL_GetTick() - tickstart) > 1000u) 
        {
            g_tx_busy = 0; // 强制复位繁忙标志，尝试挽救
            break; 
        }
    }

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
static void send_wave_pkt(uint8_t dev_id, const float *buf, uint8_t seq, uint8_t total_pkts)
{
    if (!buf) return;

    static uint8_t tx[FRAME_LEN]; // FRAME_LEN = 262

		uint8_t *p = tx;
		uint32_t offset = (uint32_t)seq * PTS_PER_PKT; 
	
		/* 头部 4B */
		*p++ = dev_id;        // 1B
		*p++ = CMD_WAVE_PACK;      // 1B
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

/**********************************广播发现应答**********************************/
static void send_discover_rsp(uint8_t cur_addr)
{
    static uint8_t tx[25]; // 稍微开大一点
    uint8_t *p = tx;

    uint8_t uid[12];
    UID_Fill_BE_w0w1w2(uid); // 获取唯一ID

    /* --- 1. 构造报文 --- */
    *p++ = cur_addr;       // 地址
    *p++ = CMD_DISCOVER;   // 功能码
    *p++ = 13;             // 长度
    memcpy(p, uid, 12);    // UID
    p += 12;
    *p++ = cur_addr;       // 地址后缀

    /* --- 2. 计算CRC --- */
    uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
    *p++ = (uint8_t)(crc & 0xFF);
    *p++ = (uint8_t)(crc >> 8);
    
    uint16_t packet_len = (uint16_t)(p - tx);

    /* --- 3. 冲突退避逻辑 (核心修改) --- */
    
    // A. 基础时间片 (Slot Time)
    // 9600波特率发一包(约20字节)耗时约21ms。
    // 为了防止物理层信号拖尾，我们设为 30ms 的安全间隔。
    uint32_t slot_time_ms = 30; 
    
    // B. 生成随机槽位 (Slot Index)
    // 之前只用了 uid[11]，范围太小。
    // 现在我们将 UID 的所有字节相加，确保差异化。
    uint32_t uid_sum = 0;
    for(int i = 0; i < 12; i++) {
        uid_sum += uid[i];
    }
    
		// 加入 SysTick 或 运行时间的低位作为扰动
    // 这样每次扫描，设备的延时都会发生微小变化
    uint32_t tick_jitter = HAL_GetTick() & 0x1F; // 取 Tick 的低5位 (0-31)
		
    // 取模 100，意味着随机产生 0 ~ 99 之间的延时等级
    // 如果有10个设备，分到100个坑里，撞车概率会大幅降低
    uint32_t random_slot = uid_sum % 100; 

    // C. 计算总延时
    // 最大延时 = 99 * 30ms = 2970ms (约3秒)
    uint32_t total_delay = random_slot * slot_time_ms;
    
    // D. 执行延时
    HAL_Delay(total_delay);

    /* --- 4. 发送数据 --- */
    // 发送前再检查一下总线是否空闲会更稳健，但在HAL库里比较麻烦，
    // 只要时间槽错开，直接发通常没问题。
    uart3_send_dma(tx, packet_len); 
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
        static uint8_t tx[5+2];
				uint8_t *p = tx;
        *p++ = new_addr; *p++ = CMD_SET_ADDR; *p++ = 0x02; *p++ = 0x4F;*p++ = 0x4B;
        uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
        *p++ = (uint8_t)crc; *p++ = (uint8_t)(crc >> 8);
		//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
				uart3_send_dma(tx, (uint16_t)(p - tx));	
        return true;
    }

    if (Flash_WriteDeviceAddr(new_addr) == HAL_OK) {
        LOCAL_DEVICE_ADDR = new_addr;
        static uint8_t tx[5+2];
				uint8_t *p = tx;
        *p++ = new_addr; *p++ = CMD_SET_ADDR; *p++ = 0x02; *p++ = 0x4F;*p++ = 0x4B;
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
		uint16_t f = rd_be16(&rx[2]);        // dev|cmd 之后 2 字节
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
static void Cfg_SendAck(uint8_t dev_id)
{
    static uint8_t tx[5 + 2];      
    uint8_t *p = tx;

    *p++ = dev_id;
    *p++ = CMD_CONFIG;              
    *p++ = 0x02;   // 0 = OK
		*p++ = 0x4F;
    *p++ = 0x4B;   //ok                      

    uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
    *p++ = (uint8_t)crc;
    *p++ = (uint8_t)(crc >> 8);

//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
		uart3_send_dma(tx, (uint16_t)(p - tx));	
}
/**********************************校准配置应答**********************************/
static void CALIBRATION_Config_SendAck(uint8_t dev_id)
{
		static uint8_t tx[5 + 2];
		uint8_t *p = tx;
		*p++ = dev_id;
		*p++ = CMD_CALIBRATION;
		*p++ = 0x02;   // 0 = OK
		*p++ = 0x4F;
    *p++ = 0x4B;   //ok
	  uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
    *p++ = (uint8_t)crc;
    *p++ = (uint8_t)(crc >> 8);
//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
		uart3_send_dma(tx, (uint16_t)(p - tx));	
}

/**********************************OTA处理函数**********************************/
static void Flash_ClearErrors(void)
{
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
}

// 1. 处理 OTA 开始命令
// 主机发送: [DevID] [0x50] [Len(4B)] [CRC]
static void Handle_OTA_Start(uint8_t dev_id, const uint8_t *rx_data)
{
    // 解析固件总长度 (大端)
    uint32_t total_len = rd_be32(rx_data);
    
    // 简单检查长度 (H723 下载区 384KB)
    if (total_len == 0 || total_len > (384 * 1024)) return;
		s_received_bytes = 0;
    // 解锁 Flash
    HAL_FLASH_Unlock();
		Flash_ClearErrors(); // 关键：清除之前的错误标志
    
    // 擦除下载区 (Sector 4, 5, 6)
    // 注意：擦除 384KB 可能需要几秒钟，这期间会阻塞主循环
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;

    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Banks         = FLASH_BANK_1;
    EraseInitStruct.Sector        = FLASH_SECTOR_4; // 从 Sector 4 开始
    EraseInitStruct.NbSectors     = 3;              // 擦除 4, 5, 6
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK)
    {
        // 擦除成功，回复 ACK
        static uint8_t tx[7];uint8_t *p = tx;
        *p++ = dev_id; *p++ = CMD_OTA_START; *p++ = 0x02; *p++ = 0x4F; *p++ = 0x4B; // 4F4B OK
        uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
        *p++ = (uint8_t)crc; *p++ = (uint8_t)(crc >> 8);
				uart3_send_dma(tx, (uint16_t)(p - tx));		
    }
    
    HAL_FLASH_Lock();
}



// 2. 处理 OTA 数据包
// 主机发送: [DevID] [CMD] [Offset(4B)] [DataLen(2B)] [Data...] [CRC]
static void Handle_OTA_Data(uint8_t dev_id, const uint8_t *rx_data, uint16_t frame_payload_len)
{
    // frame_payload_len 是除去头部(Dev+Cmd)和尾部(CRC)后的总长度
    
    // 1. 基础长度检查: 至少要有 Offset(4) + DataLen(2) = 6 字节
    if (frame_payload_len < 6) return;
    
    // 2. 解析参数
    uint32_t offset = rd_be32(rx_data);          // 读取 4字节 偏移
    uint16_t expect_len = rd_be16(rx_data + 4);  // 读取 2字节 主机指定的长度
    const uint8_t *pData = rx_data + 6;          // 数据指针向后移 6 字节
    
    // 3. 计算实际剩余的数据字节数
    uint16_t actual_len = frame_payload_len - 6;

    // 4. 校验主机发送的长度 与 实际接收长度是否一致
    // 如果不一致，说明传输过程有丢包或协议解析错误，绝对不能写入，否则会越界或错位
    if (expect_len != actual_len)
    {
        return; 
    }

    // 5. 对齐检查 (32字节对齐)
    // 注意：这里检查的是主机指定的 expect_len
    if ((offset % 32 != 0) || (expect_len % 32 != 0))
    {
        return; 
    }

    // 计算写入目标地址
    uint32_t target_addr = OTA_DOWNLOAD_ADDR + offset;
    
    HAL_FLASH_Unlock();
    Flash_ClearErrors(); // 清除标志
        
    // 循环写入
    static uint32_t flash_word_buf[8]; // 32 bytes
    
    // 使用主机指定的 expect_len 进行循环
    for (uint32_t i = 0; i < expect_len; i += 32)
    {
        // 这里的 pData 已经是偏移过后的正确位置
        memcpy(flash_word_buf, &pData[i], 32);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, target_addr + i, (uint32_t)flash_word_buf) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return; 
        }
    }
    
    HAL_FLASH_Lock();
    s_received_bytes += expect_len; // 累加接收字节数

    // 回复 ACK   
		static uint8_t tx[7];uint8_t *p = tx;
		*p++ = dev_id; *p++ = CMD_OTA_DATA; *p++ = 0x02; *p++ = 0x4F; *p++ = 0x4B; // 4F4B OK
		uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
		*p++ = (uint8_t)crc; *p++ = (uint8_t)(crc >> 8);		
    uart3_send_dma(tx, (uint16_t)(p - tx));    
		
}

// 3. 处理 OTA 结束命令
// 主机发送: [DevID] [0x52] [TotalLen(4B)] [CRC]
static void Handle_OTA_End(uint8_t dev_id, const uint8_t *rx_data)
{
    uint32_t fw_len = rd_be32(rx_data);
		
		if (fw_len == 0 || s_received_bytes != fw_len)
    {
        // 可以在这里回复一个 NACK (错误码)，告诉上位机校验失败
        // 或者直接忽略，不重启，不设置标志
        return; 
    }
    // 1. 回复 ACK (必须先回复，因为一会要重启了)
		static uint8_t tx[7];uint8_t *p = tx;
		*p++ = dev_id; *p++ = CMD_OTA_START; *p++ = 0x02; *p++ = 0x4F; *p++ = 0x4B; // 4F4B OK
		uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
		*p++ = (uint8_t)crc; *p++ = (uint8_t)(crc >> 8);
	
    // 使用阻塞发送，确保重启前发出去
    HAL_UART_Transmit(&huart3, tx, 7, 100); 

    // 2. 设置标志位 (请求 Bootloader 升级)
    Flash_SetOTAInfo(OTA_FLAG_UPDATE_NEEDED, fw_len);

    // 3. 重启
    HAL_Delay(100);
    HAL_NVIC_SystemReset();
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
		case CMD_CONFIG:Config_ParseAndApply_Freq(rx);Cfg_SendAck(dev_id);break;
    case CMD_CALIBRATION:Z_Calib_Z_Upright_Neg1G(g_data_z, 100);CALIBRATION_Config_SendAck(dev_id); break;
		case CMD_OTA_START:	Handle_OTA_Start(dev_id, &rx[2]);break;
		case CMD_OTA_DATA:Handle_OTA_Data(dev_id, &rx[2], len - 4);break;
		case CMD_OTA_END:	Handle_OTA_End(dev_id, &rx[2]);break;
		default:
//        Protocol_SendNack(dev_id, cmd, PKT_ERR_CMD);            
        break;
    }
} 