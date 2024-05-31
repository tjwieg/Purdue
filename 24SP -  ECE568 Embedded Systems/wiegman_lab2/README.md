# Hardware Connections
This project requires five discrete components, in addition to the ESP32 development board and jumper wires to connect them:
 - Red LED
 - Green LED
 - (2x) 100 Ohm resistor (or similar resistance)
 - Momentary Switch (e.g. pushbutton)

## Circuit Connections
 - 3.3V --> Red LED --> Resistor --> ESP32 Pin 32 (Set: Output)
 - 3.3V --> Green LED --> Resistor --> ESP32 Pin 14 (Set: Output)
 - 3.3V --> Pushbutton --> ESP32 Pin 15 (Set: Input, Pulldown)
 - Jumper Wire --> ESP32 Pin 12 (Set: TouchPad Input)

## Video Demonstration
Available at [https://photos.app.goo.gl/WmhFPGrnZVRdU1Sv8](https://photos.app.goo.gl/WmhFPGrnZVRdU1Sv8)
