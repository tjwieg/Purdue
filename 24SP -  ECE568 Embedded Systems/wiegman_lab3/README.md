# Hardware Setup
This project requires five external components (and wires to connect them to the ESP32 board):
 - Green LED
 - Red LED
 - (2x) Resistors (~100 Ohms)
 - MPU-6050 breakout board

The red and green LEDs are each connected on one side (through the resistors) to GPIO pins 14 and 15, respectively, and to 3.3V on the other side. GPIO pins are toggled to LOW in order to activate the lights, as microcontrollers are usually better able to sink current than output it.  
The MPU-6050 has four pins to attach: SCL and SDA to pins 22 and 23, respectively, VIN to 3.3V, and GND tied to the ESP32's GND.

# Software
This project utilizes a [3rd-party micropython library](https://github.com/micropython-IMU/micropython-mpu9x50) to interface with the [MPU-6050 acceleration and gyro sensor](https://www.adafruit.com/product/3886) over I2C.  
The latest version of this library can be cloned from its public GitHub hyperlinked above, though the necessary files (versions available 2024-05-04) are included in this folder.

# Video Demonstration
A video demonstrating this codebase working on a physical ESP-32 is available here:
https://purdue0-my.sharepoint.com/:v:/g/personal/wiegman_purdue_edu/Ec7Jxbr12LhBl4Lap4t5Ea8B7hfAabK9ve0PyBhM0pRILA?e=FXKpsj
