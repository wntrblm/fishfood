import time
import board
import digitalio
import neopixel
import colorsys
import busio

PIN_CAM_LED = board.GP12
PIN_AUX_PIN = board.GP13
PIN_UART_TX = board.GP8
PIN_UART_RX = board.GP9
PIN_M0_DIR = board.GP24
PIN_M0_STEP = board.GP25
PIN_M0_DIAG = board.GP26
PIN_M0_EN = board.GP27
PIN_M1_DIR = board.GP20
PIN_M1_STEP = board.GP21
PIN_M1_DIAG = board.GP22
PIN_M1_EN = board.GP23
PIN_M2_DIR = board.GP16
PIN_M2_STEP = board.GP17
PIN_M2_DIAG = board.GP18
PIN_M2_EN = board.GP19

TMC_UART = busio.UART(PIN_UART_TX, PIN_UART_RX, baudrate=9600)

CAM_LEDS = neopixel.NeoPixel(PIN_CAM_LED, 8, brightness=1, auto_write=True, pixel_order=neopixel.GRB)

M0_DIR = digitalio.DigitalInOut(PIN_M0_DIR)
M0_DIR.switch_to_output()
M0_DIAG = digitalio.DigitalInOut(PIN_M0_DIAG)
M0_DIAG.switch_to_input(digitalio.Pull.UP)
M0_EN = digitalio.DigitalInOut(PIN_M0_EN)
M0_EN.switch_to_output()
M0_STEP = digitalio.DigitalInOut(PIN_M0_STEP)
M0_STEP.switch_to_output()
M0_EN.value = False
M0_DIR.value = True

M1_DIR = digitalio.DigitalInOut(PIN_M1_DIR)
M1_DIR.switch_to_output()
M1_DIAG = digitalio.DigitalInOut(PIN_M1_DIAG)
M1_DIAG.switch_to_input(digitalio.Pull.UP)
M1_EN = digitalio.DigitalInOut(PIN_M1_EN)
M1_EN.switch_to_output()
M1_STEP = digitalio.DigitalInOut(PIN_M1_STEP)
M1_STEP.switch_to_output()
M1_EN.value = False
M1_DIR.value = True

M2_DIR = digitalio.DigitalInOut(PIN_M2_DIR)
M2_DIR.switch_to_output()
M2_DIAG = digitalio.DigitalInOut(PIN_M2_DIAG)
M2_DIAG.switch_to_input(digitalio.Pull.UP)
M2_EN = digitalio.DigitalInOut(PIN_M2_EN)
M2_EN.switch_to_output()
M2_STEP = digitalio.DigitalInOut(PIN_M2_STEP)
M2_STEP.switch_to_output()
M2_EN.value = False
M2_DIR.value = True

hue = 0
last = time.monotonic()
steps = 0
max_steps = 4000

while True:
    M0_STEP.value = not M0_STEP.value
    M1_STEP.value = not M1_STEP.value
    M2_STEP.value = not M2_STEP.value

    steps += 1
    if (steps == max_steps):
    	M1_DIR.value = not M1_DIR.value
    	steps = 0

    if last < time.monotonic() - 5:
        CAM_LEDS.fill([int(x * 255) for x in colorsys.hsv_to_rgb(hue % 1.0, 1.0, 0.5)])
        hue += 0.1
        last = time.monotonic()
    else:
        time.sleep(0.0005)
