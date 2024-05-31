# MicroPython
# TJ Wiegman
# ECE 568, Lab 1
# 2024-03-22

from machine import Pin, PWM, RTC, Timer
from time import sleep

# Print current datetime every 10 seconds
rtc = RTC()
weekdays = {
    0: "Monday",   1: "Tuesday", 2: "Wednesday",
    3: "Thursday", 4: "Friday",  5: "Saturday",
    6: "Sunday"
}

def clock_chime(x):
    yyyy, mo, dd, wd, hh, mn, ss, _ = rtc.datetime()
    datetime = f"{yyyy}-{mo:02}-{dd:02} {hh:02}:{mn:02}:{ss:02}"
    print(f"It is now {weekdays[wd]}, {datetime}")

tim0 = Timer(0)
tim0.init(period=10*1000, mode=Timer.PERIODIC, callback=clock_chime)

# Blink LED at 256/1023 brightness, 1 Hz
p13 = PWM(Pin(13))
brightness = 256
tim1 = Timer(1)

def blink(x):
    if p13.duty() == brightness:
        p13.duty(1023)
    else:
        p13.duty(brightness)

def blinker(hz):
    P = int(500/hz)
    tim1.init(period=P, mode=Timer.PERIODIC, callback=blink)
    print(f"Blinking LED at {hz} Hz")

blinker(1)

# Read & debounce switch input
p21 = Pin(21, Pin.IN, Pin.PULL_DOWN)
tim2 = Timer(2)
fastmode = False
holding = False

def toggle(x):
    if p21.value() == 1:
        global fastmode
        fastmode = not fastmode
        if fastmode: blinker(5)
        else: blinker(1)

while True:
    if p21.value() == 1:
        if not holding:
            tim2.init(period=50, mode=Timer.ONE_SHOT, callback=toggle)
            holding = True
            sleep(0.5)
    else:
        holding = False
