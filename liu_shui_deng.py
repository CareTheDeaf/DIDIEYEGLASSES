from machine import Pin, ADC
from time import sleep, ticks_ms
import neopixel
import network
import socket
import struct

# === WS2812 灯带配置 ===
LED_PIN = 6              # WS2812 数据引脚 (GPIO6)
NUM_LEDS = 50            # 灯珠数量
np = neopixel.NeoPixel(Pin(LED_PIN), NUM_LEDS)

# === 滑动变阻器 (亮度调节) 配置 ===
BRIGHTNESS_PIN = 5
adc = ADC(Pin(BRIGHTNESS_PIN))
adc.atten(ADC.ATTN_11DB)  # 适应 0~3.3V

# === 基本控制函数 ===
def get_brightness():
    raw = adc.read()
    return min(max(raw / 4095, 0.0), 1.0)

def set_color_all(hex_r, hex_g, hex_b, duration=0.1):
    brightness = get_brightness()
    r = int(hex_r * brightness)
    g = int(hex_g * brightness)
    b = int(hex_b * brightness)
    color = (g, r, b)  # WS2812 顺序为 GRB
    for i in range(NUM_LEDS):
        np[i] = color
    np.write()
    sleep(duration)

# === 贪吃蛇流水灯效果 ===
def green_snake_cycle(wait_ms=50):
    brightness = get_brightness()
    length = 5
    for i in range(NUM_LEDS + length):
        for j in range(NUM_LEDS):
            if i - length < j <= i:
                np[j] = (0, int(255 * brightness), 0)
            else:
                np[j] = (0, 0, 0)
        np.write()
        sleep(wait_ms / 1000)

# === 彩虹流水灯效果 ===
def rainbow_cycle(wait_ms=20):
    brightness = get_brightness()
    for j in range(256):
        for i in range(NUM_LEDS):
            pixel_index = (i * 256 // NUM_LEDS + j) & 255
            r, g, b = wheel(pixel_index)
            np[i] = (int(g * brightness), int(r * brightness), int(b * brightness))
        np.write()
        sleep(wait_ms / 1000)

def wheel(pos):
    if pos < 85:
        return (255 - pos * 3, pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return (0, 255 - pos * 3, pos * 3)
    else:
        pos -= 170
        return (pos * 3, 0, 255 - pos * 3)

# === WiFi STA 连接（加入错误处理和尝试次数） ===
ssid = 'my-wifi'
password = '12345678'
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

max_attempts = 10
attempt = 0

while not wlan.isconnected() and attempt < max_attempts:
    try:
        wlan.connect(ssid, password)
        print("尝试连接WiFi...")
    except OSError as e:
        print("连接出错，重试:", e)
    for _ in range(20):
        if wlan.isconnected():
            break
        green_snake_cycle(30)
    attempt += 1

if wlan.isconnected():
    print("WiFi连接成功:", wlan.ifconfig())
else:
    print("WiFi连接失败，进入离线模式")

# === UDP 接收配置 ===
UDP_PORT = 8266
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', UDP_PORT))
sock.settimeout(0.1)  # 非阻塞等待接收

# === 主循环 ===
last_command_time = ticks_ms()
COMMAND_TIMEOUT = 2000  # 2 秒无命令则认为空闲

while True:
    try:
        data, addr = sock.recvfrom(4)  # 4字节数据: R G B DURATION
        if len(data) == 4:
            r, g, b, d = struct.unpack('BBBB', data)
            set_color_all(r, g, b, d / 10)  # d 单位为100ms
            last_command_time = ticks_ms()
    except:
        # 若空闲，则运行彩虹灯效
        if ticks_ms() - last_command_time > COMMAND_TIMEOUT:
            rainbow_cycle(10)