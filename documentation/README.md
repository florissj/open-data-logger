# Documentation
This document serves as the main guide on how to work with the datalogger. It is roughly divided into three parts:

+ How to use the datalogger
+ Data processing
+ Further development

For a less detailed manual, refer to the quick start guide.

## Technical Details
### Main Components
+ Controller box
    + User interface
    + Teensy 4.1 Microcontroller
    + ICM20948 IMU
+ Analog Splitter:
    + 10-pin in -> 4x 4-pin out
+ Digital Splitter:
    + 6-pin in -> 4x 6-pin out

While there is a distinction made between digital and analog sensors, the sensors are moreso seperated by their ability to use I2C or not. Digital sensors that cannot use this protocol, like the wheel speed sensor, also need to be connected to the analog splitter.

### Auxiliary Components
+ Steer angle sensor (*analog*)
    + 10 kOhm potentiometer
+ Wheel speed sensor (*digital*)
    + XX Hall sensor

## Wiring
The logger currently has a capacity of 4 analog sensors and 4 digital sensors. The digital sensors are controller via I2C and don't need pin assignment, but the analog pins are assigned as follows:

+ Slot 1 -- Pin A1 / 15
+ Slot 2 -- Pin A2 / 16
+ Slot 3 -- Pin A3 / 17
+ Slot 4 -- Pin A6 / 20

*The jump is due to pin 18 and 19 being used for I2C*

The aviator outputs exiting the controller box have labeled pin numbers and are colour coded inside the housing:

### Analog Main Pinout *(10 pin)*

| Connector Pin | Teensy Pin | Colour     |
|---------------|------------|------------|
| 1             | TBD        | White      |
| 2             | TBD        | Purple     |
| 3             | TBD        | Gray       |
| 4             | 3V3        | Light Blue |
| 5             | GND        | Black      |
| 6             | 5V         | Red        |
| 7             | Slot 4     | Green      |
| 8             | Slot 3     | Yellow     |
| 9             | Slot 2     | Dark Blue  |
| 10            | Slot 1     | Orange     |


### Digital Main Pinout *(6 pin)*

| Connector Pin | Teensy Pin | Colour |
|---------------|------------|--------|
|               |            |        |
|               |            |        |
|               |            |        |
|               |            |        |
|               |            |        |
|               |            |        |

Furthermore, the outputs from both of the splitters are consistent for all four slots. According to the labeling on the respective aviator connectors, the assignment is as follows:

### Analog Splitter Pinout *(4 pin)*
| Connector Pin | Usage  |
|---------------|--------|
| 1             | Data   |
| 2             | Ground |
| 3             | VCC    |
| 4             | TBD    |

The fourth pin is connected to 3V3 for slot 1, as some sensors, like the potentiometers operate at 3V3. The rest of the slots are left vacant at this pin for future applications

### Digital Splitter Pinout *(6 pin)*
The output pins share the same configuration as the input pin, as they both have 6 pins. Refer to Digital Main Pinout.

### Interface
The 10 pin dupont wire used for the interface is also colour coded, which ideally should not be changed but is listed solely for reference:
*NB. the colours are those of the cable, not the wires soldered on the board*

| Dupont Pin (left-to-right) | Teensy Pin | Usage         | Colour |
|----------------------------|------------|---------------|--------|
| 1                          | 36         | Select Button | Brown  |
| 2                          | 37         | Rotary Button | Red    |
| 3                          | 33         | Rotary B      | Orange |
| 4                          | 34         | Rotary A      | Yellow |
| 5                          | 19         | SCL           | Green  |
| 6                          | 18         | SDA           | Blue   |
| 7                          | 35         | LED GREEN     | Purple |
| 8                          | 39         | LED RED       | Gray   |
| 9                          | VCC        |               | White  |
| 10                         | GND        |               | Black  | 
