from machine import ADC, Pin
from time import sleep_us
import math
import neopixel

# === 参数配置 ===
ADC_PIN = 0             # 音频采样 ADC 引脚
POT_PIN = 5             # 滑动变阻器 ADC 引脚
SAMPLE_RATE = 1000       # 采样频率 (Hz)
FFT_SIZE = 64            # FFT 采样点数（2 的幂）
HOP_SIZE = FFT_SIZE // 2 # 窗口滑动步长
ALPHA = 0.995            # 高通滤波系数
NOTCH_FREQ = 50          # 陷波频率（工频）
Q = 5.0                  # 陷波品质因数

# WS2812 灯带配置
LED_PIN = 6              # WS2812 数据引脚 (GPIO6)
NUM_LEDS = 50            # 灯珠数量
np = neopixel.NeoPixel(Pin(LED_PIN), NUM_LEDS)

# === ADC 初始化 ===
adc_audio = ADC(Pin(ADC_PIN))
adc_audio.atten(ADC.ATTN_11DB)  # 音频输入范围 0~3.3V
adc_pot   = ADC(Pin(POT_PIN))
adc_pot.atten(ADC.ATTN_11DB)    # 滑动变阻器范围 0~3.3V

# === 高通滤波状态 ===
dc_estimate = 0.0       # 直流分量估计值

# === Notch 陷波器系数 ===
fs = SAMPLE_RATE
f0 = NOTCH_FREQ
w0 = 2 * math.pi * f0 / fs
cos_w0 = math.cos(w0)
sin_w0 = math.sin(w0)
alpha_n = sin_w0 / (2 * Q)

b0 = 1
b1 = -2 * cos_w0
b2 = 1
a0 = 1 + alpha_n
a1 = -2 * cos_w0
a2 = 1 - alpha_n
# 归一化系数
b0 /= a0; b1 /= a0; b2 /= a0
a1 /= a0; a2 /= a0
# 状态变量
x1 = x2 = y1 = y2 = 0.0

# === 汉明窗函数 ===
def hamming_window(N):
    return [0.54 - 0.46 * math.cos(2 * math.pi * n / (N - 1)) for n in range(N)]
window = hamming_window(FFT_SIZE)

# === 简单递归 FFT ===
def fft_simple(x):
    N = len(x)
    if N <= 1:
        return x
    even = fft_simple(x[0::2])
    odd  = fft_simple(x[1::2])
    T = [complex(math.cos(-2*math.pi*k/N), math.sin(-2*math.pi*k/N)) * odd[k] for k in range(N//2)]
    return [even[k] + T[k] for k in range(N//2)] + \
           [even[k] - T[k] for k in range(N//2)]

# === HSV -> RGB 转换函数 ===
def hsv_to_rgb(h, s, v):
    c = v * s
    x = c * (1 - abs((h / 60) % 2 - 1))
    m = v - c
    if h < 60:
        rp, gp, bp = c, x, 0
    elif h < 120:
        rp, gp, bp = x, c, 0
    elif h < 180:
        rp, gp, bp = 0, c, x
    elif h < 240:
        rp, gp, bp = 0, x, c
    elif h < 300:
        rp, gp, bp = x, 0, c
    else:
        rp, gp, bp = c, 0, x
    return (int((rp + m) * 255), int((gp + m) * 255), int((bp + m) * 255))

# === 环形缓冲区 & 状态变量 ===
buffer = [0.0] * FFT_SIZE
idx = 0
hop = 0
last_energy = 0.0       # 用于节拍检测
beat_thresh = 20.0      # 能量突变阈值，可调
decay_factor = 0.8      # 节拍后亮度衰减系数
beat_scale = 0.0        # 当前节拍强度
hue_offset = 0.0        # 节拍触发的色相偏移

# === 主循环 ===
while True:
    # 1) 读取滑动变阻器，映射为总体亮度基准 0.1~1.0
    pot_val = adc_pot.read()
    base_brightness = 0.1 + 0.9 * (pot_val / 4095)

    # 2) 采样音频并去直流、陷波
    raw = adc_audio.read()
    voltage = raw * 3.3 / 4095
    dc_estimate = ALPHA * dc_estimate + (1 - ALPHA) * voltage
    ac = voltage - dc_estimate
    x0 = ac
    y0 = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2
    x2, x1 = x1, x0
    y2, y1 = y1, y0

    buffer[idx] = y0
    idx = (idx + 1) % FFT_SIZE
    hop += 1

    # 3) 滑动窗口到达，做 FFT 分析
    if hop >= HOP_SIZE:
        win_data = [buffer[(idx + i) % FFT_SIZE] * window[i] for i in range(FFT_SIZE)]
        m = sum(win_data) / FFT_SIZE
        win_data = [v - m for v in win_data]
        spec = fft_simple(win_data)
        mags = [abs(c) for c in spec[:FFT_SIZE//2]]

        # 主频 & 能量
        peak_bin = mags.index(max(mags))
        freq = peak_bin * SAMPLE_RATE / FFT_SIZE
        energy = sum([v*v for v in win_data])

        # 节拍检测
        delta = energy - last_energy
        last_energy = energy
        if delta > beat_thresh:
            beat_scale = 1.0
            hue_offset = (hue_offset + 90) % 360  # 每次节拍跳色相
        beat_scale *= decay_factor
        if beat_scale < 0.01:
            beat_scale = 0.0

        # 颜色与亮度映射
        base_hue = min(freq / (SAMPLE_RATE/2), 1.0) * 360
        hue = (base_hue + hue_offset * beat_scale) % 360  # 频率+节拍动态偏移
        saturation = 1.0
        value = base_brightness * (0.2 + 0.8 * beat_scale)
        final_color = hsv_to_rgb(hue, saturation, value)

        # 打印关键参数
        print("Freq:{:.1f}Hz ΔE:{:.2f} Beat:{:.2f} Hue:{:.1f}".format(freq, delta, beat_scale, hue))

        # 更新所有灯
        for i in range(NUM_LEDS):
            np[i] = final_color
        np.write()

        hop = 0

    sleep_us(int(1_000_000 / SAMPLE_RATE))

