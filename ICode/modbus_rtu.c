#include "modbus_rtu.h"
#include "usart.h"
#include "bytes.h"
#include "tim.h"

#define RX_MIN_LEN     7          /* Head(2)+Dev(1)+Cmd(1)+Ch(1)+Chk(1)+Foot(2) */
#define MAX_SAMPLE_FREQ_HZ   51200u  // 51.2kHz
#define MIN_SAMPLE_FREQ_HZ   1024u   // 1kHz (淇濊瘉4绉掑唴鑳藉嚭缁撴灉)

extern float zBuf[FFT_N_Z];
extern uint16_t g_cfg_freq_hz;
extern uint16_t g_cfg_points;
extern uint16_t wave_points;   // 娉㈠舰鍙戦€?

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

    /* 澶х锛氶珮浣嶅湪鍓嶏紱椤哄簭鍥哄畾涓?w0 | w1 | w2 */
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
    uint16_t crc = 0xFFFF;           // 鍒濆鍊?
    const uint16_t poly = 0xA001;    

    while (length--) {
        crc ^= *data++;              // 寮傛垨
        for (uint8_t i = 0; i < 8; ++i) {
            if (crc & 0x0001) {      // 鏈€浣庝綅
                crc = (crc >> 1) ^ poly;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;                     
}

// 閽堝 Flash 鐨勬祦寮?CRC32 鏍￠獙鍑芥暟
static uint32_t Calc_Flash_CRC32(uint32_t start_addr, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t *p = (const uint8_t *)start_addr;
    
    while (len--)
    {
        crc ^= *p++; // 浠?Flash 鐩存帴璇诲彇
        for (uint32_t i = 0; i < 8; i++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc = (crc >> 1);
        }
    }
    return ~crc;
}

static HAL_StatusTypeDef uart3_send_dma(uint8_t *buf, uint16_t len)
{
    uint32_t tickstart = HAL_GetTick();   
    while (g_tx_busy)
    {
        if ((HAL_GetTick() - tickstart) > 1000u) 
        {
            g_tx_busy = 0; // 寮哄埗澶嶄綅绻佸繖鏍囧織锛屽皾璇曟尳鏁?
            break; 
        }
    }

    g_tx_busy = 1;
    return HAL_UART_Transmit_DMA(&huart3, buf, len);
}

/**********************************鐗瑰緛鍊煎簲绛?*********************************/
static void send_feature_pkt(uint8_t dev_id,
                              const AxisFeatureValue *X_data,
                              const AxisFeatureValue *Y_data,
                              const AxisFeatureValue *Z_data,
															float temperature)
{
    static uint8_t tx[77];  // 3 + 16 + 16 +36 + 4 + 2 = 77 (涓変釜缁撴瀯浣撴暟鎹? + CRC
    uint8_t *p = tx;
		memset(tx, 0, sizeof(tx));

    *p++ = dev_id;
    *p++ = CMD_FEATURE;
    *p++ = 0x48;

    // ----- X_data 鍖哄煙锛? 脳 float (16B) -----
		put_be_f32(&p, X_data->mean);
    put_be_f32(&p, X_data->rms);
    put_be_f32(&p, X_data->pp);
    put_be_f32(&p, X_data->kurt);

    // ----- Y_data 鍖哄煙锛? 脳 float (16B) -----
    put_be_f32(&p, Y_data->mean);
    put_be_f32(&p, Y_data->rms);
    put_be_f32(&p, Y_data->pp);
    put_be_f32(&p, Y_data->kurt);

    // ----- Z_data 鍖哄煙锛? 脳 float (36B) -----
    put_be_f32(&p, Z_data->mean);
    put_be_f32(&p, Z_data->rms);
    put_be_f32(&p, Z_data->pp);
    put_be_f32(&p, Z_data->kurt);
    put_be_f32(&p, Z_data->peakFreq);
    put_be_f32(&p, Z_data->peakAmp);
    put_be_f32(&p, Z_data->amp2x);
    put_be_f32(&p, Z_data->envelope_vrms);
    put_be_f32(&p, Z_data->envelope_peak);
		 // -----  temp 鍖哄煙 -----
    put_be_f32(&p, temperature);
		
		size_t payload_len = (size_t)(p - tx);      // 宸插啓鍏ョ殑鐪熷疄瀛楄妭鏁?
    uint16_t crc = Modbus_CRC16(tx, payload_len);  
    *p++ = crc & 0xFF;        
    *p++ = (crc >> 8) & 0xFF; 

//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
		uart3_send_dma(tx, (uint16_t)(p - tx));	
}

/* 娴嬭瘯鐢細鍙戦€佺壒寰佸寘锛屾暟鎹尯鐢?00,11,22,...,FF 寰幆濉厖 */
static void send_feature_pkt_test(uint8_t dev_id)
{
    enum { HEADER_LEN = 3, DATA_LEN = 72, CRC_LEN = 2, FRAME_LEN = HEADER_LEN + DATA_LEN + CRC_LEN };
    static uint8_t tx[FRAME_LEN];
    uint8_t *p = tx;

    // 澶撮儴锛歞ev_id, CMD_FEATURE, 鍥哄畾 0x48
    *p++ = dev_id;
    *p++ = CMD_FEATURE;
    *p++ = 0x48;

    // 鏁版嵁鍖猴細72 瀛楄妭鍥哄畾妯″紡濉厖锛?0,11,22,...,FF 寰幆锛?
    for (uint32_t i = 0; i < DATA_LEN; ++i) {
        *p++ = (uint8_t)((i & 0x0Fu) * 0x11u);
        // 鑻ヤ綘鍙兂鍦?0x00,0x11,0x22,0x33 鍥涘€奸棿寰幆锛屽彲鏀逛负锛?
        // *p++ = (uint8_t)(((i % 4u) * 0x11u) & 0xFFu);
    }

    uint16_t crc = Modbus_CRC16(tx, (size_t)(p - tx));
    *p++ = (uint8_t)(crc & 0xFF);
    *p++ = (uint8_t)((crc >> 8) & 0xFF);

    // 鍙戦€佸疄闄呭抚闀匡紙搴斾负 77 瀛楄妭锛?
//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
		uart3_send_dma(tx, (uint16_t)(p - tx));	
}
/**********************************娉㈠舰搴旂瓟**********************************/
static void send_wave_ack(uint8_t dev_id)
{
    static uint8_t tx[7]; 
    uint8_t *p = tx;

    *p++ = dev_id;
    *p++ = CMD_WAVE;    // 
    *p++ = 0x02;        // LEN: 鏁版嵁闀垮害涓? (鍗冲悗闈㈣窡鐫€鐨?b2 鍜?b3)
    *p++ = 0x4F;          // 
    *p++ = 0x4B;          // 杩斿洖 ok

    // 璁＄畻 CRC
    uint16_t crc = Modbus_CRC16(tx, (size_t)(p - tx));
    *p++ = (uint8_t)(crc & 0xFF);        // Low
    *p++ = (uint8_t)((crc >> 8) & 0xFF); // High

//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
		uart3_send_dma(tx, (uint16_t)(p - tx));	
}

/* ---- 鍗忚甯搁噺 ---- */
enum { PTS_PER_PKT   = 64 };                     // 姣忓寘 64 鐐?
enum { HEADER_NOCRC  = 6  };                     // dev_id(1) + CMD_WAVE(1) + seq(2) + total_pkts(2) 
enum { DATA_LEN      = PTS_PER_PKT * 4 };        // 64 * 4 = 256
enum { FRAME_NOCRC   = HEADER_NOCRC + DATA_LEN };// 6 + 256 = 262
enum { CRC_LEN       = 2  };
enum { FRAME_LEN     = FRAME_NOCRC + CRC_LEN };  // 262 + 2 = 264

/* 甯э細dev_id | CMD_WAVE  | seq(2B) |total_pkts(2B) | 64脳float(BE) | CRC(LE) */
static void send_wave_pkt(uint8_t dev_id, const float *buf, uint16_t seq, uint16_t total_pkts)
{
    if (!buf) return;

    const uint16_t expected_total = (uint16_t)((LONG_WAVE_LEN + PTS_PER_PKT - 1U) / PTS_PER_PKT);
    if ((total_pkts == 0U) || (total_pkts > expected_total)) {
        total_pkts = expected_total;
    }
    if ((seq >= total_pkts) || (seq >= expected_total)) {
        return;
    }

    static uint8_t tx[FRAME_LEN]; // FRAME_LEN = 264

    uint8_t *p = tx;
    uint32_t offset = (uint32_t)seq * PTS_PER_PKT;

    /* 澶撮儴 6B */
    *p++ = dev_id;
    *p++ = CMD_WAVE_PACK;
    put_be_u16(&p, seq);
    put_be_u16(&p, total_pkts);

    /* 鏁版嵁鍖猴細64 涓猣loat锛屾寜澶х鍐欏叆锛涙湯鍖呬笉瓒宠ˉ 0.0f */
    for (uint16_t i = 0; i < PTS_PER_PKT; ++i) {
        uint32_t idx = offset + i;
        float v = (idx < LONG_WAVE_LEN) ? buf[idx] : 0.0f;
        put_be_f32(&p, v);
    }

    uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
    *p++ = (uint8_t)(crc & 0xFF);        // Low
    *p++ = (uint8_t)((crc >> 8) & 0xFF); // High

    uart3_send_dma(tx, (uint16_t)(p - tx));
}

/**********************************骞挎挱鍙戠幇搴旂瓟**********************************/
static void send_discover_rsp(uint8_t cur_addr)
{
    static uint8_t tx[25]; // 绋嶅井寮€澶т竴鐐?
    uint8_t *p = tx;

    uint8_t uid[12];
    UID_Fill_BE_w0w1w2(uid); // 鑾峰彇鍞竴ID

    /* --- 1. 鏋勯€犳姤鏂?--- */
    *p++ = 0x00;       // 鍦板潃
    *p++ = CMD_DISCOVER;   // 鍔熻兘鐮?
    *p++ = 0x0D;             // 闀垮害
    memcpy(p, uid, 12);    // UID
    p += 12;
    *p++ = cur_addr;       // 鍦板潃鍚庣紑

    /* --- 2. 璁＄畻CRC --- */
    uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
    *p++ = (uint8_t)(crc & 0xFF);
    *p++ = (uint8_t)(crc >> 8);
    
    uint16_t packet_len = (uint16_t)(p - tx);

    /* --- 3. 鍐茬獊閫€閬块€昏緫 (鏍稿績淇敼) --- */
    
    // A. 鍩虹鏃堕棿鐗?(Slot Time)
    // 9600娉㈢壒鐜囧彂涓€鍖?绾?0瀛楄妭)鑰楁椂绾?1ms銆?
    // 涓轰簡闃叉鐗╃悊灞備俊鍙锋嫋灏撅紝鎴戜滑璁句负 30ms 鐨勫畨鍏ㄩ棿闅斻€?
    uint32_t slot_time_ms = 30; 
    
    // B. 鐢熸垚闅忔満妲戒綅 (Slot Index)
    // 涔嬪墠鍙敤浜?uid[11]锛岃寖鍥村お灏忋€?
    // 鐜板湪鎴戜滑灏?UID 鐨勬墍鏈夊瓧鑺傜浉鍔狅紝纭繚宸紓鍖栥€?
    uint32_t uid_sum = 0;
    for(int i = 0; i < 12; i++) {
        uid_sum += uid[i];
    }
    
		// 鍔犲叆 SysTick 鎴?杩愯鏃堕棿鐨勪綆浣嶄綔涓烘壈鍔?
    // 杩欐牱姣忔鎵弿锛岃澶囩殑寤舵椂閮戒細鍙戠敓寰皬鍙樺寲
    uint32_t tick_jitter = HAL_GetTick() & 0x1F; // 鍙?Tick 鐨勪綆5浣?(0-31)
		
    // 鍙栨ā 100锛屾剰鍛崇潃闅忔満浜х敓 0 ~ 99 涔嬮棿鐨勫欢鏃剁瓑绾?
    // 濡傛灉鏈?0涓澶囷紝鍒嗗埌100涓潙閲岋紝鎾炶溅姒傜巼浼氬ぇ骞呴檷浣?
    uint32_t random_slot = uid_sum % 100; 

    // C. 璁＄畻鎬诲欢鏃?
    // 鏈€澶у欢鏃?= 99 * 30ms = 2970ms (绾?绉?
    uint32_t total_delay = random_slot * slot_time_ms;
    
    // D. 鎵ц寤舵椂
    HAL_Delay(total_delay);

    /* --- 4. 鍙戦€佹暟鎹?--- */
    // 鍙戦€佸墠鍐嶆鏌ヤ竴涓嬫€荤嚎鏄惁绌洪棽浼氭洿绋冲仴锛屼絾鍦℉AL搴撻噷姣旇緝楹荤儲锛?
    // 鍙鏃堕棿妲介敊寮€锛岀洿鎺ュ彂閫氬父娌￠棶棰樸€?
    uart3_send_dma(tx, packet_len); 
}

//dev_id | CMD_VERSION | 0x02 | version | sensor_type | crc_l | crc_h
static void send_version_pkt(uint8_t dev_id)
{
    static uint8_t tx[7];
    uint8_t *p = tx;

    *p++ = dev_id;
    *p++ = CMD_VERSION;
    *p++ = 0x02;
    *p++ = APP_FW_VERSION;
    *p++ = APP_SENSOR_TYPE;

    uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
    *p++ = (uint8_t)(crc & 0xFF);
    *p++ = (uint8_t)(crc >> 8);

    uart3_send_dma(tx, (uint16_t)(p - tx));
}

/**********************************閰嶇疆鍦板潃搴旂瓟**********************************/
// 鍙澶х UID锛孋RC 浠嶅皬绔?
static bool HandleSetAddr_Broadcast(const uint8_t* rx, uint16_t flen)
{
    if (flen < 2 + 12 + 1) return false; // dev|cmd|UID12|newAddr 鏈€灏忛暱搴?

    if (rx[0] != 0x00 || rx[1] != CMD_SET_ADDR) return false;

    uint8_t off = 2;
    if (flen >= 3 + 12 + 1 && rx[2] == 0x0D) off = 3;

    const uint8_t* uid_in   = &rx[off];
    const uint8_t  new_addr = rx[off + 12];

    // 澶х UID 瀹屽叏鍖归厤
		static uint8_t uid_me[12];
    UID_Fill_BE_w0w1w2(uid_me);                // 鏈満 UID锛堝ぇ绔紝w0|w1|w2锛?
//		dump_uid("uid_in", uid_in);
//		dump_uid("uid_me", uid_me);
    if (memcmp(uid_in, uid_me, 12) != 0) return true;

//    if (new_addr == 0x00 || new_addr == 0xFF) return true;

    // 骞傜瓑锛氫竴鏍峰氨鍙洖 ACK
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


/**********************************瑙ｆ瀽閰嶇疆甯?*********************************/
static void Config_ParseAndApply_Freq(const uint8_t* rx)
{
		uint16_t f = rd_be16(&rx[3]);        // dev|cmd cnt涔嬪悗 2 瀛楄妭
		if (f < MIN_SAMPLE_FREQ_HZ || f > MAX_SAMPLE_FREQ_HZ) {
        return; 
    }
		if (f == g_cfg_freq_hz) return;
    if (Flash_UpdateFreq(f) == HAL_OK) 
		{
        g_cfg_freq_hz = f;
				Algo_Update_LPF_Coeff(g_cfg_freq_hz);
        ACQ_SetFreqHz(g_cfg_freq_hz);
    }
}

/*static void Config_ParseAndApply_Point(const uint8_t* rx)
{
		uint16_t pts = rd_be16(&rx[3]);     // dev|cmd|sub 涔嬪悗 2 瀛楄妭
    if (pts == 0) return;
		if (pts > FLASH_CFG_DEFAULT_POINTS) return;
    if (pts == g_cfg_points) return;

    if (Flash_UpdatePoints(pts) == HAL_OK) {
        g_cfg_points = pts;
        wave_points  = pts;            
    }
}
static void Config_SendAck(uint8_t dev_id)
{
    static uint8_t tx[3 + 2 + 2 + 2];      // dev|cmd|len|freq2|crc2
    uint8_t *p = tx;

    *p++ = dev_id;
    *p++ = CMD_CONFIG;              
    *p++ = 4;                       // payload 闀垮害

	  uint16_t freq16 = (uint16_t)g_cfg_freq_hz;
    uint16_t pts    = (uint16_t)g_cfg_points;

    wr_be16(p, freq16); p += 2;
    wr_be16(p, pts ); p += 2;

    uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
    *p++ = (uint8_t)crc;
    *p++ = (uint8_t)(crc >> 8);

//		HAL_UART_Transmit_DMA(&huart3, tx, (uint16_t)(p - tx));
		uart3_send_dma(tx, (uint16_t)(p - tx));	
}*/
/**********************************閲囨牱閰嶇疆搴旂瓟**********************************/
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
/**********************************鏍″噯閰嶇疆搴旂瓟**********************************/
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

/**********************************OTA澶勭悊鍑芥暟**********************************/
static void Flash_ClearErrors(void)
{
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
}

// 1. 澶勭悊 OTA 寮€濮嬪懡浠?
// 涓绘満鍙戦€? [DevID] [0x50] [Len(4B)] [CRC]
static void Handle_OTA_Start(uint8_t dev_id, const uint8_t *rx_data)
{
    // 瑙ｆ瀽鍥轰欢鎬婚暱搴?(澶х)
    uint32_t total_len = rd_be32(rx_data);
    
    // 绠€鍗曟鏌ラ暱搴?(H723 涓嬭浇鍖?384KB)
    if (total_len == 0 || total_len > (384 * 1024)) return;
		s_received_bytes = 0;
    // 瑙ｉ攣 Flash
    HAL_FLASH_Unlock();
		Flash_ClearErrors(); // 鍏抽敭锛氭竻闄や箣鍓嶇殑閿欒鏍囧織
    
    // 鎿﹂櫎涓嬭浇鍖?(Sector 4, 5, 6)
    // 娉ㄦ剰锛氭摝闄?384KB 鍙兘闇€瑕佸嚑绉掗挓锛岃繖鏈熼棿浼氶樆濉炰富寰幆
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;

    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Banks         = FLASH_BANK_1;
    EraseInitStruct.Sector        = FLASH_SECTOR_4; // 浠?Sector 4 寮€濮?
    EraseInitStruct.NbSectors     = 3;              // 鎿﹂櫎 4, 5, 6
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK)
    {
        // 鎿﹂櫎鎴愬姛锛屽洖澶?ACK
        static uint8_t tx[7];uint8_t *p = tx;
        *p++ = dev_id; *p++ = CMD_OTA_START; *p++ = 0x02; *p++ = 0x4F; *p++ = 0x4B; // 4F4B OK
        uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
        *p++ = (uint8_t)crc; *p++ = (uint8_t)(crc >> 8);
				uart3_send_dma(tx, (uint16_t)(p - tx));		
    }
    
    HAL_FLASH_Lock();
}



// 2. 澶勭悊 OTA 鏁版嵁鍖?
// 涓绘満鍙戦€? [DevID] [CMD] [Offset(4B)] [DataLen(2B)] [Data...] [CRC]
static void Handle_OTA_Data(uint8_t dev_id, const uint8_t *rx_data, uint16_t frame_payload_len)
{
    // frame_payload_len 鏄櫎鍘诲ご閮?Dev+Cmd)鍜屽熬閮?CRC)鍚庣殑鎬婚暱搴?
    
    // 1. 鍩虹闀垮害妫€鏌? 鑷冲皯瑕佹湁 Offset(4) + DataLen(2) = 6 瀛楄妭
    if (frame_payload_len < 6) return;
    
    // 2. 瑙ｆ瀽鍙傛暟
    uint32_t offset = rd_be32(rx_data);          // 璇诲彇 4瀛楄妭 鍋忕Щ
    uint16_t expect_len = rd_be16(rx_data + 4);  // 璇诲彇 2瀛楄妭 涓绘満鎸囧畾鐨勯暱搴?
    const uint8_t *pData = rx_data + 6;          // 鏁版嵁鎸囬拡鍚戝悗绉?6 瀛楄妭
    
    // 3. 璁＄畻瀹為檯鍓╀綑鐨勬暟鎹瓧鑺傛暟
    uint16_t actual_len = frame_payload_len - 6;

    // 4. 鏍￠獙涓绘満鍙戦€佺殑闀垮害 涓?瀹為檯鎺ユ敹闀垮害鏄惁涓€鑷?
    // 濡傛灉涓嶄竴鑷达紝璇存槑浼犺緭杩囩▼鏈変涪鍖呮垨鍗忚瑙ｆ瀽閿欒锛岀粷瀵逛笉鑳藉啓鍏ワ紝鍚﹀垯浼氳秺鐣屾垨閿欎綅
    if (expect_len != actual_len)
    {
        return; 
    }

    // 5. 瀵归綈妫€鏌?(32瀛楄妭瀵归綈)
    // 娉ㄦ剰锛氳繖閲屾鏌ョ殑鏄富鏈烘寚瀹氱殑 expect_len
    if ((offset % 32 != 0) || (expect_len % 32 != 0))
    {
        return; 
    }

    // 璁＄畻鍐欏叆鐩爣鍦板潃
    uint32_t target_addr = OTA_DOWNLOAD_ADDR + offset;
    
    HAL_FLASH_Unlock();
    Flash_ClearErrors(); // 娓呴櫎鏍囧織
        
    // 寰幆鍐欏叆
    static uint32_t flash_word_buf[8]; // 32 bytes
    
    // 浣跨敤涓绘満鎸囧畾鐨?expect_len 杩涜寰幆
    for (uint32_t i = 0; i < expect_len; i += 32)
    {
        // 杩欓噷鐨?pData 宸茬粡鏄亸绉昏繃鍚庣殑姝ｇ‘浣嶇疆
        memcpy(flash_word_buf, &pData[i], 32);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, target_addr + i, (uint32_t)flash_word_buf) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return; 
        }
    }
    
    HAL_FLASH_Lock();
    s_received_bytes += expect_len; // 绱姞鎺ユ敹瀛楄妭鏁?

    // 鍥炲 ACK   
		static uint8_t tx[7];uint8_t *p = tx;
		*p++ = dev_id; *p++ = CMD_OTA_DATA; *p++ = 0x02; *p++ = 0x4F; *p++ = 0x4B; // 4F4B OK
		uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
		*p++ = (uint8_t)crc; *p++ = (uint8_t)(crc >> 8);		
    uart3_send_dma(tx, (uint16_t)(p - tx));    
		
}

// 3. 澶勭悊 OTA 缁撴潫鍛戒护
// 涓绘満鍙戦€? [DevID] [0x52] [Len(4B)] [WholeCRC32(4B)] [CRC16]
static void Handle_OTA_End(uint8_t dev_id, const uint8_t *rx_data)
{
	uint32_t fw_len = rd_be32(rx_data);     // 鍓?瀛楄妭鏄暱搴?
    uint32_t host_crc = rd_be32(rx_data + 4); // 鍚?瀛楄妭鏄笂浣嶆満绠楀ソ鐨?CRC32

		if (fw_len == 0 || s_received_bytes != fw_len)
    {
        return; 
    }
		
		uint32_t calc_crc = Calc_Flash_CRC32(OTA_DOWNLOAD_ADDR, fw_len);
		
		if (calc_crc != host_crc)//閿欒鍥炲
    {
        // Flash 閲岀殑鏁版嵁鍜屼笂浣嶆満鍙戠殑涓嶄竴鑷达紒
        // 姝ゆ椂缁濆涓嶈兘閲嶅惎锛屽惁鍒?Bootloader 浼氬埛鍏ユ崯鍧忕殑鍥轰欢
        static uint8_t tx_err[8]; uint8_t *p = tx_err;
        *p++ = dev_id; *p++ = CMD_OTA_END ; *p++ = 0x02; // 寮傚父鍝嶅簲
        *p++ = 0xBA; *p++ = 0xD1; // 閿欒鐮?BAD1
        uint16_t crc = Modbus_CRC16(tx_err, 5);
        *p++ = (uint8_t)crc; *p++ = (uint8_t)(crc >> 8);
        uart3_send_dma(tx_err, 6);
        return; 
    }
		//姝ｇ‘鍥濧CK
		static uint8_t tx[7];uint8_t *p = tx;
		*p++ = dev_id; *p++ = CMD_OTA_END; *p++ = 0x02; *p++ = 0x4F; *p++ = 0x4B; // 4F4B OK
		uint16_t crc = Modbus_CRC16(tx, (uint16_t)(p - tx));
		*p++ = (uint8_t)crc; *p++ = (uint8_t)(crc >> 8);
	
    // 浣跨敤闃诲鍙戦€侊紝纭繚閲嶅惎鍓嶅彂鍑哄幓
    HAL_UART_Transmit(&huart3, tx, 7, 100); 

    // 2. 璁剧疆鏍囧織浣?(璇锋眰 Bootloader 鍗囩骇)
		Flash_SetOTAInfo(OTA_FLAG_UPDATE_NEEDED, fw_len, calc_crc);
		
    // 3. 閲嶅惎
    HAL_Delay(100);
    HAL_NVIC_SystemReset();
}

/**********************************甯у鐞?*********************************/
bool Protocol_HandleRxFrame(const uint8_t *rx, uint16_t len, uint8_t local_address)
{
    if (len < RX_MIN_LEN)                         { return false; }
    
    uint16_t rx_crc = rd_le16(&rx[len - 2U]);
    uint16_t calc_crc = Modbus_CRC16(rx, (uint16_t)(len - 2U));
    if (rx_crc != calc_crc) {
        return false;
    }

    uint8_t dev_id = rx[0];                       // 鎻愬彇璇锋眰涓殑璁惧鍦板潃
    uint8_t cmd    = rx[1];
    const bool is_broadcast = (dev_id == 0x00);
    uint16_t wave_seq = 0U;
    uint16_t wave_total = 0U;
    
    if (cmd == CMD_WAVE_PACK) {
        if (len >= 8U) {
            // 鏂板崗璁細dev|cmd|seq(2B)|total(2B)|crc(2B)
            wave_seq = rd_be16(&rx[2]);
            wave_total = rd_be16(&rx[4]);
        } else {
            return false;
        }
    }

		if (is_broadcast && cmd == CMD_DISCOVER) {
        send_discover_rsp(local_address);  // 鍥?UID + 褰撳墠鍦板潃
        return true;
    }
		
		if (is_broadcast && cmd == CMD_SET_ADDR) {
        HandleSetAddr_Broadcast(rx, len);
				return true;
    }
		
		if (is_broadcast && cmd == CMD_REBOOT) {
        HAL_Delay(100);
				HAL_NVIC_SystemReset();
        return true;
    }
		
		if (!is_broadcast && dev_id != local_address) 
		{
        return false;
    }
								
    switch (cmd)
    {
    case CMD_FEATURE: send_feature_pkt(dev_id, &X_data, &Y_data, &Z_data, Temp); break;
		case CMD_WAVE:Create_Wave_Snapshot();send_wave_ack(dev_id); break;
		case CMD_WAVE_PACK:	send_wave_pkt(dev_id, Tx_Wave_Buffer_Z, wave_seq, wave_total); break;
		case CMD_CONFIG:Config_ParseAndApply_Freq(rx);Cfg_SendAck(dev_id);break;
    case CMD_CALIBRATION:Z_Calib_Z_Upright_Neg1G();CALIBRATION_Config_SendAck(dev_id); break;
		case CMD_VERSION:send_version_pkt(dev_id);break;
		case CMD_OTA_START:	Handle_OTA_Start(dev_id, &rx[2]);break;
		case CMD_OTA_DATA:Handle_OTA_Data(dev_id, &rx[2], len - 4);break;
		case CMD_OTA_END:	Handle_OTA_End(dev_id, &rx[2]);break;
		default:
//        Protocol_SendNack(dev_id, cmd, PKT_ERR_CMD);            
        break;
    }
    return true;
} 



