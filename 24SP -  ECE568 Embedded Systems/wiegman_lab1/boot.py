# MicroPython
# TJ Wiegman
# ECE 568, Lab 1
# 2024-03-22

from machine import RTC

rtc_prompts = ["Year? ", "Month? ", "Day? ",
    "Weekday? ", "Hour? ", "Minute? ",
    "Second? ", "Microsecond? "
]

print("") # an extra line, for neatness
rtc_results = [int(input(x)) for x in rtc_prompts]

rtc = RTC()
rtc.datetime(tuple(rtc_results))
print("") # an extra line, for neatness
