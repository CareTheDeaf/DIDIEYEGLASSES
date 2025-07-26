''' main.py
ESP32-C3 WS2812 炫动眼镜主程序
功能：
 - 滑动变阻器调节全局亮度
 - 用户按键切换 2 种动画模式，并切换时指示 LED 状态
   • 模式 1：基于 MPU6050 Roll 的彩虹追逐动画
   • 模式 2：统一彩色旋转流水动画
 - 指示灯：GPIO8，显示当前模式
 - 平滑渐变，30ms 固定帧率
'''
import utime
import math
from machine import Pin, ADC
import neopixel
from mpu6050 import MPU6050

# === 硬件引脚 ===
PIN_NEOPIXEL = 6
NUM_LEDS = 50
PIN_SLIDER   = 5  # 亮度调节
PIN_BUTTON   = 7  # 模式切换
PIN_LED_IND  = 8  # 模式指示 LED

# 初始化外围
np = neopixel.NeoPixel(Pin(PIN_NEOPIXEL), NUM_LEDS)
slider = ADC(Pin(PIN_SLIDER)); slider.atten(ADC.ATTN_11DB)
btn    = Pin(PIN_BUTTON, Pin.IN, Pin.PULL_DOWN)
led_ind= Pin(PIN_LED_IND, Pin.OUT)
mpu    = MPU6050()

# 模式定义
MODE_RAINBOW = 0
MODE_ROTATE  = 1
mode = MODE_RAINBOW
led_ind.value(mode)

# 防抖变量
button_last = 0
debounce_ms = 50
last_switch = utime.ticks_ms()

# 动画状态
step    = 0.0
last_ms = utime.ticks_ms()

# HSV 转 RGB
from math import floor

def hsv_to_rgb(h, s, v):
    i = int(h * 6)
    f = h * 6 - i
    p = int(255 * v * (1 - s))
    q = int(255 * v * (1 - s * f))
    t = int(255 * v * (1 - s * (1 - f)))
    v = int(255 * v)
    i %= 6
    if i == 0: return v, t, p
    if i == 1: return q, v, p
    if i == 2: return p, v, t
    if i == 3: return p, q, v
    if i == 4: return t, p, v
    return v, p, q

# 按键扫描并切换模式

def scan_button():
    global mode, button_last, last_switch
    cur = btn.value()
    now = utime.ticks_ms()
    if cur != button_last and utime.ticks_diff(now, last_switch) > debounce_ms:
        if cur == 1:
            mode = MODE_ROTATE if mode == MODE_RAINBOW else MODE_RAINBOW
            led_ind.value(mode)
            last_switch = now
        button_last = cur

# 读取亮度

def get_brightness():
    return slider.read() / 4095

# 模式1：基于 Roll 角度的彩虹追逐

def mode_rainbow():
    global step, last_ms
    now = utime.ticks_ms()
    dt = (now - last_ms) / 1000.0
    last_ms = now

    accel = mpu.read_accel_data()
    roll = math.degrees(math.atan2(accel['y'], accel['z']))
    factor = min(max((roll + 90) / 180, 0), 1)
    speed = 0.002 + factor * 0.018

    bri = get_brightness()
    for i in range(NUM_LEDS):
        h = (i / NUM_LEDS + step) % 1.0
        r, g, b = hsv_to_rgb(h, 1.0, bri)
        np[i] = (r, g, b)
    np.write()
    step = (step + speed) % 1.0

# 模式2：统一彩色旋转流水

def mode_rotate():
    global step
    bri = get_brightness()
    for i in range(NUM_LEDS):
        # 将环形位置映射为 hue
        h = ((i / NUM_LEDS) + step) % 1.0
        r, g, b = hsv_to_rgb(h, 1.0, bri)
        np[i] = (r, g, b)
    np.write()
    # 固定缓动速度
    step = (step + 0.005) % 1.0

# 主循环
while True:
    scan_button()
    if mode == MODE_RAINBOW:
        mode_rainbow()
    else:
        mode_rotate()
    utime.sleep_ms(30)