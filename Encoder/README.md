Hardware Requirements - ESP32, Encoder, jumper wires, USB C cable, laptop

This package takes input from an Encoder to an ESP32 C6 through pins 4 & 5 connected to pin C1 and C2 of encoder respectively. The ESP32 then records and publishes the direction as well as the magnitude of motion to a serial monitor every 100 milliseconds

The code is done using the ISR handling triggering on any edge of the motion input(C1), then verifying and comparing the direction input(C2) to determine the direction or rotation.
