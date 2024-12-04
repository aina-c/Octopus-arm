# Octopus-arm

This is the code to control 2 DC motors through a driver. The flex sensor is read and mapped to the range delimited for the motors, that position is inserted into a PID controller (at the moment only P as the overshoot is not too bad) and the position of the motors is changed dynamically as the sensor input changes.
This code works for the current prototype however, other iterations may have to be modified to work within correct ranges.


For this project were used: 1 Arduino Uno Wifi Rev2, 1 L298N dual H-Bridge motor driver, 2 DC 12V 140rpm-i45 motors with gearbox and encoder integrated and a strain gauge module with positive and negative bending angle sensor.
