import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# 串口配置
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.1
)

# 数据缓冲区
buffer = bytearray()
points_queue = deque(maxlen=1000)  # 存储点云坐标

def parse_packet(packet):
    """解析单个数据包"""
    try:
        # 解析包头PH (0x66AA)
        ph = packet[0:2]
        
        # 解析S&C和LSN
        s_c = packet[2]
        lsn = packet[3]
        
        # 解析角度信息
        fsa = struct.unpack('>H', packet[4:6])[0]
        lsa = struct.unpack('>H', packet[6:8])[0]
        
        # 计算起始和结束角度
        angle_fsa = (fsa >> 1) / 64.0
        angle_lsa = (lsa >> 1) / 64.0
        
        # 计算角度差
        angle_diff = (angle_lsa - angle_fsa) if (angle_lsa >= angle_fsa) else (angle_lsa + 360 - angle_fsa)
        
        # 解析采样数据
        sample_data = packet[8:-2]
        points = []
        
        for i in range(lsn):
            idx = i * 3
            b1, b2, b3 = sample_data[idx], sample_data[idx+1], sample_data[idx+2]
            
            # 打印十六进制数据
            print(f"Sample {i}: {bytes([b1, b2, b3]).hex().upper()}")
            
            # 计算光强
            intensity = b1 + ((b2 & 0x03) << 8)
            
            # 计算距离
            distance = (b3 << 6) | (b2 >> 2)
            
            # 计算当前点角度
            if lsn > 1:
                angle = angle_fsa + (angle_diff / (lsn - 1)) * i
            else:
                angle = angle_fsa
                
            # 转换为笛卡尔坐标
            rad = np.deg2rad(angle)
            x = distance * np.cos(rad)
            y = distance * np.sin(rad)
            points.append((x, y))
            
        return points
    
    except Exception as e:
        print(f"Parse error: {str(e)}")
        return []

# 实时绘图初始化
fig, ax = plt.subplots()
ax.set_xlim(-8000, 8000)
ax.set_ylim(-8000, 8000)
scat = ax.scatter([], [], s=1)

def update(frame):
    """更新点云图"""
    global buffer
    # 读取串口数据
    while ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        buffer.extend(data)
        print(f"Raw HEX: {data.hex().upper()}")  # 打印原始十六进制数据
    
    # 数据包解析
    while True:
        # 查找包头
        start = buffer.find(b'\x66\xAA')
        if start == -1:
            break
            
        # 检查最小包长度
        if len(buffer) < start + 8:
            break
            
        # 获取LSN确定包长度
        lsn = buffer[start + 3]
        pkt_len = 2 + 1 + 1 + 2 + 2 + 2 + 3 * lsn
        
        if len(buffer) < start + pkt_len:
            break
            
        # 提取完整数据包
        packet = buffer[start:start + pkt_len]
        buffer = buffer[start + pkt_len:]
        
        # 解析数据包
        points = parse_packet(packet)
        points_queue.extend(points)
    
    # 更新散点图数据
    if points_queue:
        xy = np.array(points_queue)
        scat.set_offsets(xy)
    
    return scat,

# 启动动画
ani = animation.FuncAnimation(fig, update, interval=50)

try:
    plt.show()
except KeyboardInterrupt:
    ser.close()
    print("程序已终止")
