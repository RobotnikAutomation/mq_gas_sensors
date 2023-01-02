# Gas Concentration Monitoring with MQ-135 and MQ-2 Sensors
This Arduino sketch reads the gas concentration of CO2 and LPG (Liquified Petroleum Gas) values from two sensors: the MQ-135 and MQ-2 and then communicates the values to ROS using the following libraries:

- [MQUnifiedsensor by Miguel Califa](https://github.com/miguel5612/MQSensorsLib) to interface with the MQ-135 and MQ-2 sensors.
- [Rosserial Arduino Library by Michael Ferguson](https://github.com/frankjoshua/rosserial_arduino_lib) to communicate the values to ROS.

The sketch publishes the gas concentration values to two ROS topics: `mq135_CO2` and `mq2_GLP`.

### Hardware
 
The following hardware is required to run this sketch:

- Arduino board (e.g. Uno).
- MQ-135 and MQ-2 sensor modules.
- Jumper wires to connect the sensor modules to the Arduino.
- USB cable to connect the Arduino board to the device running ROS.

### Prerequisites

Before you can use this sketch, you need to install the following software:

- [Arduino IDE](https://www.arduino.cc/en/Main/Software) to upload the sketch to the Arduino board.
- [ROS](http://wiki.ros.org/Installation) to receive the gas concentration values.
- [rosserial](http://wiki.ros.org/rosserial) package to establish a connection between ROS and the Arduino.

To install the rosserial package, open a terminal and run the following command:

```
sudo apt-get install ros-melodic-rosserial
sudo apt-get install ros-melodic-rosserial-arduino
```

Replace <rosdistro> with the name of your ROS distribution (e.g. melodic, noetic).

### Installation

  To use this sketch, follow these steps:

1. Connect the MQ-135 and MQ-2 sensor modules to the Arduino using jumper wires. The specific connections will depend on the modules and the Arduino board you are using.

    - Connect the Vcc and GND pins of each sensor to the 5V and GND pins of the Arduino board, respectively.
    - The code is implemented such that the analog input A0 corresponds to the MQ-135 sensor and analog input A1 corresponds to the MQ-2 sensor.

2. Connect the Arduino board to the device running ROS using a USB cable.

3. Download and install the required libraries: MQUnifiedsensor and Rosserial Arduino Library. You can download them directly in the Arduino IDE.

4. Open the sketch in the Arduino IDE and upload it to the Arduino board.

5. Close the Arduino IDE. In two separate terminal windows, run the following commands:

```
(Terminal 1) roscore
(Terminal 2) rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
```
Replace /dev/ttyUSB0 with the correct serial port for your setup (e.g. /dev/ttyACM0) and replace 57600 with the correct baud rate (e.g. 115200).

### Usage

Once the sketch is running on the Arduino and you have set up the ROS environment, you can start monitoring the gas concentration values. The values will be published to two ROS topics: `mq135_CO2` and `mq2_GLP`. You can use the rostopic command to view the values in real time:

```
(Terminal 3) rostopic echo mq135_CO2
(Terminal 4) rostopic echo mq2_GLP
```
### Troubleshooting

If you are having trouble getting the sketch to work, here are a few things to check:

- Make sure the Arduino is connected to your computer and the correct port is selected in the Arduino IDE.

- Make sure the USB cable is securely connected between the Arduino board and the device running ROS.

- Make sure the required libraries (MQUnifiedsensor and Rosserial Arduino Library) are installed and included in the sketch.

- Check the connections between the sensor modules and the Arduino. Make sure they are correct and secure.

- If you are using the MQ-135 sensor, make sure it has warmed up for at least 20 minutes before taking readings.

- If you are using the MQ-2 sensor, make sure it is not exposed to open flames or high temperatures, as this can damage the sensor.
