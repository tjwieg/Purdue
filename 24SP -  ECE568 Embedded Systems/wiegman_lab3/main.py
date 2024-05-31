# MicroPython
# TJ Wiegman
# ECE 568, Lab 3
# 2024-05-04

from imu import MPU6050
from machine import Pin, Timer
import machine, network, requests

# WiFi Configuration
WIFI_CREDENTIALS = {"ssid" : "TJW_hotspot", "pass" : "password6789"}
def wifi_connect():
    print("Connecting to Wi-Fi...")
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        ssid = WIFI_CREDENTIALS["ssid"]
        pword = WIFI_CREDENTIALS["pass"]
        wlan.connect(ssid, pword)
        while not wlan.isconnected():
            pass
    print(f"--------\nConnected to {wlan.config('ssid')}")
    print(f"IP Address: {wlan.ifconfig()[0]}\n--------")
    return wlan

wlan = wifi_connect()

# LED Configuration
redLED = Pin(14, Pin.OPEN_DRAIN)
def red(state): # red(True) to activate red LED
    if state: redLED.off()
    else: redLED.on()

greenLED = Pin(15, Pin.OPEN_DRAIN)
def green(state): # green(True) to activate green LED
    if state: greenLED.off()
    else: greenLED.on()

## Initialize both LEDs to be off
red(False)
green(False)

# IMU Configuration
i2c = machine.I2C(0, scl=Pin(22), sda=Pin(23))
mpu = MPU6050(i2c)

# Motion detection setup
armed = False
calibration = ()
def calibrate():
    global calibration
    global armed
    calibration = mpu.accel.xyz
    armed = True
    print("Activation state calibrated motion sensor")

# Set up activation status checking routine
activated = False
def check_activation(x):
    x = requests.get("https://api.thingspeak.com/channels/2536074/fields/1/last.json?api_key=YB49JXB2E4FHME6C&results=2")
    if x.status_code == 200:
        status = x.json()["field1"]
        if status == "1":
            global activated
            activated = True
        elif status == "0":
            global activated
            global armed
            activated = False
            armed = False
            red(False)
        green(activated)
    else:
        print("Failed to connect to Thingspeak; cannot determine activation status")

activator = Timer(0)
activator.init(period=30*1000, mode=Timer.PERIODIC, callback=check_activation)

# Motion detection routine
def check_differences(acc):
    x,y,z = acc
    return any([
        mpu.accel.x < x-0.2,
        mpu.accel.x > x+0.2,
        mpu.accel.y < y-0.2,
        mpu.accel.y > y+0.2,
        mpu.accel.z < z-0.2,
        mpu.accel.z > z+0.2
    ])

def detect_motion(x):
    if activated:
        if not armed:
            calibrate()
        else:
            if check_differences(calibration):
                red(True)
                acc = mpu.accel
                message = f"{acc.x:0.3f}x, {acc.y:0.3f}y, {acc.z:0.3f}z"
                requests.get("https://maker.ifttt.com/trigger/motion_detected/with/key/WOjoZh7vKoufPI2T93NBq?value1="+message)
            else:
                red(False)

motion_detector = Timer(1)
motion_detector.init(period=15*1000, mode=Timer.PERIODIC, callback=detect_motion)
