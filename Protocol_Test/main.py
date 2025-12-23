import serial
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd  # 用于保存 CSV
from datetime import datetime
import os

# --- 🚀 配置区域 (已根据您的要求修改) ---
COM_PORT = 'COM7'  # 电脑上的串口号
BAUD_RATE = 9600  # ⚠️注意：9600波特率传输4096点大约需要 20-25秒，请耐心等待
DEV_ADDR = 0x00  # 0x00 为广播地址，单机调试时可用
TOTAL_POINTS = 4096  # FFT_POINTS
PTS_PER_PKT = 64  # 协议定义的每包点数

# --- 📋 协议命令码 ---
CMD_FEATURE = 0x02  # 特征值请求 (新增)
CMD_WAVE = 0x03  # 波形请求 (Snapshot)
CMD_WAVE_PACK = 0x04  # 波形包读取

# --- 保存路径 ---
SAVE_DIR = "wave_data"
if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)


def calc_crc16(data):
    """计算 Modbus CRC16"""
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc


def build_frame(addr, cmd, b2=0, b3=0):
    """构建发送帧: [Addr, Cmd, b2, b3, 00, CRC_L, CRC_H]"""
    payload = struct.pack('BBBBB', addr, cmd, b2, b3, 0x00)
    crc = calc_crc16(payload)
    return payload + struct.pack('<H', crc)


def parse_features_and_print(raw_data):
    """
    解析特征值包 (77 bytes)
    结构: [Dev] [Cmd] [0x48] [X:4f] [Y:4f] [Z:9f] [Temp:1f] [CRC:2]
    """
    if len(raw_data) != 77:
        print(f"❌ 特征值包长度错误: {len(raw_data)} (预期 77)")
        return False

    # 提取数据区 (跳过头部3字节，最后2字节CRC)
    # 格式: 18个 float (Big Endian)
    payload = raw_data[3:-2]
    floats = struct.unpack('>18f', payload)

    print("\n" + "=" * 40)
    print(f"📊 传感器特征值报告 (设备 0x{raw_data[0]:02X})")
    print("=" * 40)

    # 打印 X 轴 (前4个)
    print(f"【X 轴】")
    print(f"  均值(Mean): {floats[0]:.4f} g")
    print(f"  有效值(RMS): {floats[1]:.4f} g")
    print(f"  峰峰值(P-P): {floats[2]:.4f} g")
    print(f"  峭度(Kurt):  {floats[3]:.4f}")

    # 打印 Y 轴 (接4个)
    print(f"【Y 轴】")
    print(f"  均值(Mean): {floats[4]:.4f} g")
    print(f"  有效值(RMS): {floats[5]:.4f} g")
    print(f"  峰峰值(P-P): {floats[6]:.4f} g")
    print(f"  峭度(Kurt):  {floats[7]:.4f}")

    # 打印 Z 轴 (接9个)
    print(f"【Z 轴】(主轴)")
    print(f"  均值(Mean): {floats[8]:.4f} g (预期≈1.0)")
    print(f"  有效值(RMS): {floats[9]:.4f} g")
    print(f"  峰峰值(P-P): {floats[10]:.4f} g")
    print(f"  峭度(Kurt):  {floats[11]:.4f}")
    print(f"  主频:       {floats[12]:.1f} Hz")
    print(f"  主频幅值:    {floats[13]:.4f} g")

    print(f"【其他】")
    print(f"  温度:       {floats[17]:.2f} (预留)")
    print("=" * 40 + "\n")
    return True


def main():
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=2.0)  # 9600波特率超时设长一点
        print(f"✅ 串口 {COM_PORT} 打开成功，波特率: {BAUD_RATE}")
    except Exception as e:
        print(f"❌ 串口打开失败: {e}")
        return

    # ---------------------------------------------------------
    # 1. 获取特征值 (CMD_FEATURE)
    # ---------------------------------------------------------
    print(f"[1/3] 请求特征值 (CMD: 0x{CMD_FEATURE:02X})...")
    ser.write(build_frame(DEV_ADDR, CMD_FEATURE))

    # 接收 77 字节
    try:
        feat_resp = ser.read(77)
    except Exception:
        print("❌ 读取超时")
        ser.close();
        return

    if len(feat_resp) == 77:
        if not parse_features_and_print(feat_resp):
            ser.close();
            return
    else:
        print(f"❌ 未收到完整特征包 (Len={len(feat_resp)})")
        ser.close();
        return

    # ---------------------------------------------------------
    # 2. 请求波形快照 (CMD_WAVE)
    # ---------------------------------------------------------
    print(f"[2/3] 请求波形快照 (CMD: 0x{CMD_WAVE:02X})...")
    ser.write(build_frame(DEV_ADDR, CMD_WAVE))
    ack = ser.read(7)

    if len(ack) == 7 and ack[3] == 0x4F:  # Check 'O' of "OK"
        print("✅ 快照锁定成功")
    else:
        print(f"❌ 快照请求失败: {ack.hex()}")
        ser.close();
        return

    # ---------------------------------------------------------
    # 3. 读取波形数据 (CMD_WAVE_PACK)
    # ---------------------------------------------------------
    total_pkts = TOTAL_POINTS // PTS_PER_PKT
    all_data = []

    print(f"[3/3] 开始读取波形 ({TOTAL_POINTS}点, 预计耗时 {(TOTAL_POINTS * 4 * 10 / BAUD_RATE):.1f}s)...")
    start_time = time.time()

    expected_len = 4 + (PTS_PER_PKT * 4) + 2  # 262 bytes

    for seq in range(total_pkts):
        ser.write(build_frame(DEV_ADDR, CMD_WAVE_PACK, seq, total_pkts))
        resp = ser.read(expected_len)

        if len(resp) != expected_len:
            print(f"\n❌ 包 {seq} 丢失/超时 (Len={len(resp)})")
            break

        # 提取数据
        floats = np.frombuffer(resp[4:-2], dtype='>f4')
        all_data.extend(floats)
        print(f"\r⏳ 进度: {seq + 1}/{total_pkts}", end='')

    ser.close()
    print(f"\n✅ 读取完成，耗时 {time.time() - start_time:.2f}s")

    # ---------------------------------------------------------
    # 4. 数据保存与绘图
    # ---------------------------------------------------------
    if len(all_data) > 0:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # 4.1 保存为 CSV (最适合细致了解波形)
        csv_filename = f"{SAVE_DIR}/wave_{timestamp}.csv"
        df = pd.DataFrame(all_data, columns=["Acceleration_g"])
        df.to_csv(csv_filename, index_label="Index")
        print(f"💾 数据已保存至: {csv_filename}")

        # 4.2 绘制并保存图片
        plt.figure(figsize=(14, 7))
        plt.plot(all_data, label='Z-Axis', color='#1f77b4', linewidth=0.8)
        plt.title(f"Vibration Waveform (N={len(all_data)}) - {timestamp}")
        plt.xlabel("Sample Index")
        plt.ylabel("Acceleration (g)")
        plt.grid(True, which='both', linestyle='--', alpha=0.5)
        plt.legend()

        # 局部放大图 (可选，画一个子图看前200点细节)
        if len(all_data) > 200:
            plt.axes([0.65, 0.65, 0.2, 0.2])  # [left, bottom, width, height]
            plt.plot(all_data[:200], color='#ff7f0e')
            plt.title("Zoom (First 200 pts)")
            plt.grid(True)

        img_filename = f"{SAVE_DIR}/plot_{timestamp}.png"
        plt.savefig(img_filename, dpi=150)
        print(f"🖼️ 图片已保存至: {img_filename}")
        plt.show()


if __name__ == "__main__":
    main()