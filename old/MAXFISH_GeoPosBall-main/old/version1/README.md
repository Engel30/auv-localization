# MAXFISH
Multi agents systems and Max-Plus algebra theoretical frameworks for a robot-fish shoal modelling and control.

<p align="center">
    <a href="https://www.maxfish.it/home">
<img src="https://github.com/LabMACS/MAXFISH_Simulator/assets/64741263/17e16013-9c0a-459e-b3f7-c2a549fa2d81" width=20% height=20%> </a> <a href="https://www.mur.gov.it/it"> <img src="https://github.com/LabMACS/MAXFISH_GeoPosBall/blob/main/media/LOGO_MUR%20Social.jpg" width=10% height=10%> </a> 
</p>

<p align="center">
<a href="https://www.unicas.it/"> <img src="https://github.com/LabMACS/MAXFISH_GeoPosBall/blob/main/media/Logo_Unicas.png" width=9% height=9%> </a> <a href="https://www.univpm.it/Entra/"> <img src="https://github.com/LabMACS/MAXFISH_GeoPosBall/blob/main/media/Logo_UNIVPM.png" width=9% height=9%> </a> <a href="https://www.unibo.it/it"> <img src="https://github.com/LabMACS/MAXFISH_GeoPosBall/blob/main/media/Logo_Unibo.png" width=9% height=9%> </a>
</p>


# Table of contents
1. [Preface](#preface)
2. [Requirements](#requirements)
    1. [Hardware](#hardware)
    2. [Software](#software)
3. [Configuration](#configuration)
    1. [Pre-Run](#prerun)
    2. [Run the software](#run)
4. [Examples](#examples)
   1. [Button](#button)
   2. [LED](#led)
   3. [Servo](#servo)
6. [Legal](#legal)
    1. [Credits](#credits)
    2. [License](#license)
     

## Preface <a name="preface"></a>

This guide explains how to establish a bidirectional connection between an ESP32S3 and MATLAB, using MQTT and ROS2. 

The Arduino *pub_sub_esp32s3.ino* code initializes the WiFi connection and the MQTT broker, then publishes messages to a MQTT topic (/mqtt1), subscribes another MQTT topic (/mqtt2) and waits for messages.

The *BIDbridge.sh* script creates a bridge between MQTT topics and ROS2 topics. Specifically, it subscribes to /mqtt1, converts messages and sends them to the /locdata ROS2 topic, while simultaneously subscribing to the ROS2 topic /servocmd and sending messages to /mqtt2.

The MATLAB *pub_sub_MATLAB.m* script starts a ROS2 node, publishes messages to the ROS2 /servocmd topic, subscribes to the ROS2 /locdata topic and waits for messages.
<p align="center">
<img src="https://github.com/LabMACS/MAXFISH_GeoPosBall/assets/64741263/cf9f28a2-51a7-413a-b161-2df6793d2801" width=50% height=50%>
</p>
  
## Requirements <a name="requirements"></a>
### Hardware <a name="hardware"></a>
* [M5CoreS3](https://shop.m5stack.com/products/m5stack-cores3-esp32s3-lotdevelopment-kit)

### Software <a name="software"></a>
* [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
* [Arduino IDE 2.3.2](https://www.arduino.cc/en/software):
   * [M5Stack library](https://www.arduino.cc/reference/en/libraries/m5stack/)
   * [PubSubClient library](https://www.arduino.cc/reference/en/libraries/pubsubclient/)
* [MATLAB R2023b](https://it.mathworks.com/downloads/):
   * [ROS Toolbox](https://it.mathworks.com/products/ros.html)
* [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* [Mosquitto 2.0.18](https://mosquitto.org/download/)

***For further information about the Installation, please look at the [user manual](https://github.com/LabMACS/MAXFISH_GeoPosBall/blob/main/old/user%20manual.docx).***
  
## Configuration <a name="configuration"></a>
### Pre-Run <a name="prerun"></a>
1. The default config is only bind to localhost, so to allow listeners to bind to external IP address, add in *mosquitto.conf* file:
   ```sh
   allow_anonymous true
   listener 1883 0.0.0.0
   ```
   and run mosquitto, specifying the conf file:
   ```sh
   mosquitto -c mosquitto.conf -p 1884
   ```
2. Modify in *pub_sub_esp32s3.ino* the ssid and password with your WiFi network. Modify also mqtt_server with your machine’s IP address.

   <img src="https://github.com/LabMACS/MAXFISH_GeoPosBall/assets/64741263/1bc0a13d-3d39-420a-b7d4-470779f807ac" width=50% height=50%>
 
   ***Note: you should connect both the PC and the ESP32S3 to the same WiFi network.***
4. In *BIDbridge.sh* change “source /opt/ros/humble/setup.bash” with the path of your ROS2 setup file.
   
   Change MQTT_HOST with your machine IP address.

   <img src="https://github.com/LabMACS/MAXFISH_GeoPosBall/assets/64741263/dd436add-c039-46f6-bc9e-7c996f3ae3c4" width=45% height=45%>
   
### Run the software <a name="run"></a>
* Start Mosquitto with your modified configuration file.
* Connect ESP32S3 to the PC and launch the Arduino code.
* Run the BIDbridge.sh file.
* Run the MATLAB code.

## Examples <a name="examples"></a>

This section provides three examples of how to use the architecture. The Arduino codes, MATLAB scripts, and the bridge.sh scripts are in the [Examples](https://github.com/LabMACS/MAXFISH_GeoPosBall/tree/main/old/Examples) folder.

### Button <a name="button"></a>

This example allows you to send a message to MATLAB indicating that a specific button (Button A or Button B) from M5StickCPlus was pressed.

*This example requires the use of the [M5StickCPlus](https://shop.m5stack.com/products/m5stickc-plus-esp32-pico-mini-iot-development-kit).*

### LED <a name="led"></a>

This example allows you to send a message from MATLAB to turn on the LED of the M5StickCPlus. When the word 'accendi' (turn on) is sent via a bridge that connects ROS2 and MQTT, the message reaches the ESP32 and turns on the LED. Conversely, when 'spegni' (turn off) is sent, the light on the device turns off.

*This example requires the use of the [M5StickCPlus](https://shop.m5stack.com/products/m5stickc-plus-esp32-pico-mini-iot-development-kit).*

### Servo <a name="servo"></a>

This example allows you to send a desired angle from MATLAB to the M5StickCPlus, which then moves the connected servo motor to the specified angle.

*This example requires the use of the [M5StickCPlus](https://shop.m5stack.com/products/m5stickc-plus-esp32-pico-mini-iot-development-kit), the [Micro Servo 9g SG90](https://www.robotstore.it/Servo-micro-TowerPro-SG90-9g) and the [8Servos HAT v1.1](https://docs.m5stack.com/en/hat/hat_8servos_1.1).*

## Legal <a name="legal"></a>
### Credits <a name="credits"></a>
If you have any suggestions or comments related to this GitHub project, please contact:

*LabMACS, DII, Università Politecnica delle Marche, Via Brecce Bianche, 12, Ancona, 60131, Italy* - [https://www.labmacs.university/](https://www.labmacs.university/)

* **Project Leader**: [David Scaradozzi](mailto:d.scaradozzi@staff.univpm.it)
* **Project Developer**: [Flavia Gioiello](mailto:f.gioiello@staff.univpm.it)
* **MAXFISH Project Coordinator**: [Gianluca Antonelli](mailto:antonelli@unicas.it)

### License <a name="license"></a>
[![CC BY-NC-SA 4.0][cc-by-nc-sa-shield]][cc-by-nc-sa]

This work is licensed under a
[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License][cc-by-nc-sa].

[![CC BY-NC-SA 4.0][cc-by-nc-sa-image]][cc-by-nc-sa]

[cc-by-nc-sa]: http://creativecommons.org/licenses/by-nc-sa/4.0/
[cc-by-nc-sa-image]: https://licensebuttons.net/l/by-nc-sa/4.0/88x31.png
[cc-by-nc-sa-shield]: https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg

To acknowledge the material, the following information must be reported:
* Attribution: MAXFISH PRIN project (20225RYMJE)
             <[https://www.maxfish.it/](https://www.maxfish.it/)> 
* Title of the Work: “MAXFISH GeoPosBall”
* Source: [https://www.labmacs.university/maxfish-simulator/](https://www.labmacs.university/maxfish-simulator/)
* License information: CC BY-NC-SA 4.0

The “MAXFISH GeoPosBall” is free available.
