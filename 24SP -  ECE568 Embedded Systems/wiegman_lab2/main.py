# MicroPython
# TJ Wiegman
# ECE 568, Lab 2
# 2024-04-05

from esp32 import wake_on_ext0, WAKEUP_ANY_HIGH
from machine import Pin, Timer, TouchPad
from time import sleep
import machine, network, ntptime

HARDCODE_WIFI = {
    "use"  : False,
    "ssid" : "ssid",
    "pword": "password"
}

# Connect to wifi
def wifi_connect():
    print("Connecting to Wi-Fi...")
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        if HARDCODE_WIFI["use"]:
            ssid = HARDCODE_WIFI["ssid"]
            pword = HARDCODE_WIFI["pword"]
        else:
            ssid = input("SSID: ")
            pword = input("Password: ")
        wlan.connect(ssid, pword)
        while not wlan.isconnected():
            pass
    print(f"--------\nConnected to {wlan.config('ssid')}")
    print(f"IP Address: {wlan.ifconfig()[0]}\n--------")
    return wlan

wlan = wifi_connect()

# Set up RTC with NTP time synchronization
rtc = machine.RTC()
ntptime.host = "pool.ntp.org"
ntptime.settime()
weekdays = {
    0: "Monday",   1: "Tuesday", 2: "Wednesday", 3: "Thursday", 4: "Friday",
    5: "Saturday", 6: "Sunday"
}

# Set timer to print datetime every 15 seconds
def clock_chime(x):
    UTC_OFFSET = -4 # for Indiana Daylight Time
    yyyy, mo, dd, wd, hh, mn, ss, _ = rtc.datetime()
    if hh+UTC_OFFSET < 0: hh += 24 # so that 8pm doesn't become negative4am :)
    print("--------")
    print(f"Date: {yyyy}-{mo:02}-{dd:02}")
    print(f"Time: {hh+UTC_OFFSET:02}:{mn:02}:{ss:02} hrs")
    print("--------")

tim0 = Timer(0)
tim0.init(period=15*1000, mode=Timer.PERIODIC, callback=clock_chime)

# Use capacitive touch to control green LED
greenLED = Pin(14, Pin.OPEN_DRAIN)
touch = TouchPad(Pin(12))
def touch_control(x):
    if touch.read() > 400: # not touching == LED off
        greenLED.value(1)
    else:                  # touching == LED on
        greenLED.value(0)

tim1 = Timer(1)
tim1.init(period=50, mode=Timer.PERIODIC, callback=touch_control)

# Turn on red LED while awake
redLED = Pin(32, Pin.OPEN_DRAIN)
redLED.value(0)

# Send to sleep unless pushbutton is pressed
pushbutton = Pin(15, Pin.IN, Pin.PULL_DOWN)
wake_on_ext0(pin = pushbutton, level = WAKEUP_ANY_HIGH)

def naptime(x):
    print("I am going to sleep for 1 minute.")
    redLED.value(1)
    machine.deepsleep(60*1000)

tim2 = Timer(2)
tim2.init(period=30*1000, mode=Timer.PERIODIC, callback=naptime)

# Print wake reason
if machine.wake_reason() == machine.PIN_WAKE:
    print("I woke up because you pushed the button!")
elif machine.wake_reason() == 4:
    print("I woke up because I slept for 1 minute.")
