# MAXFISH GeoPosBall
<div style="display: flex; flex-wrap: wrap; gap: 10px;">

![version](https://img.shields.io/badge/version-2.2-blue.svg)

  <a href="https://www.arduino.cc/" target="_blank">
    <img src="https://img.shields.io/badge/Framework-Arduino-00979D?style=flat&logo=arduino&logoColor=white" alt="Arduino">
  </a>

  <a href="https://isocpp.org/" target="_blank">
    <img src="https://img.shields.io/badge/C++-%2300599C.svg?logo=c%2B%2B&logoColor=white" alt="cpp">
  </a>

  
Multi agents systems and Max-Plus algebra theoretical frameworks for a robot-fish shoal modelling and control.

<p align="center">
    <a href="https://www.maxfish.it/home">
<img src="https://github.com/LabMACS/MAXFISH_Simulator/assets/64741263/17e16013-9c0a-459e-b3f7-c2a549fa2d81" width=20% height=20%> </a> <a href="https://www.mur.gov.it/it"> <img src="https://github.com/LabMACS/MAXFISH_GeoPosBall/blob/main/media/LOGO_MUR%20Social.jpg" width=10% height=10%> </a> 
</p>

<p align="center">
<a href="https://www.unicas.it/"> <img src="https://github.com/LabMACS/MAXFISH_GeoPosBall/blob/main/media/Logo_Unicas.png" width=9% height=9%> </a> <a href="https://www.univpm.it/Entra/"> <img src="https://github.com/LabMACS/MAXFISH_GeoPosBall/blob/main/media/Logo_UNIVPM.png" width=9% height=9%> </a> <a href="https://www.unibo.it/it"> <img src="https://github.com/LabMACS/MAXFISH_GeoPosBall/blob/main/media/Logo_Unibo.png" width=9% height=9%> </a>
</p>

  

| Version | Date       | Description                                             |
|---------|------------|---------------------------------------------------------|
| v1      | 2024-03-30 | First version with only the communication architecture  |
| v2.1    | 2025-04-24 | Stable version (communication + sensors)                |
| v2.2    | 2025-09-10 | Added depth + bug fixing                                |

# Table of contents
1. [Preface](#preface)
2. [Folder Structure](#folder)
3. [Requirements](#requirements)
    1. [Hardware](#hardware)
    2. [Software](#software)
4. [Run the software](#run)
5. [Legal](#legal)
    1. [Credits](#credits)
    2. [License](#license)
     

## Preface <a name="preface"></a>

A smart buoy system based on M5CoreS3, equipped with GPS and IMU, designed to collect positioning and motion data, send it via serial communication to an underwater unit (“the fish”), and publish data over MQTT via Wi-Fi for remote monitoring.

<p align="center">
<img src="https://github.com/LabMACS/MAXFISH_GeoPosBall/blob/main/media/Architettura.png">
</p>


## Folder Structure <a name="folder"></a>

The project is organized into the following main directories:

-> **`media`**:  
  Contains images, technical documentation, datasheets, schematics, and support files relevant to the project.

-> **`old`**:  
  Contains an earlier version of the communication architecture, with some examples of usage.
  
## Requirements <a name="requirements"></a>
### Hardware <a name="hardware"></a>
* [M5CoreS3](https://shop.m5stack.com/products/m5stack-cores3-esp32s3-lotdevelopment-kit)
* [Witmotion WTGAHRS2](https://www.bing.com/search?pglt=299&q=Witmotion+WTGAHRS2&cvid=c2675eb13adc4f2ba98c7643ab47bbf3&gs_lcrp=EgRlZGdlKgYIABBFGDkyBggAEEUYOTIGCAEQRRg7MgYIAhBFGDzSAQczMjJqMGoxqAIAsAIA&FORM=ANNTA1&ucpdpc=UCPD&adppc=EDGEDBB&PC=EDGEDBB)
* [Unit CatM GNSS](https://docs.m5stack.com/en/unit/catm_gnss)

### Software <a name="software"></a>
* [Arduino IDE 2.3.2](https://www.arduino.cc/en/software):
   * [M5Stack library](https://www.arduino.cc/reference/en/libraries/m5stack/)
   * [PubSubClient library](https://www.arduino.cc/reference/en/libraries/pubsubclient/)
   
## Run the software <a name="run"></a>
* Start Arduino with your modified configuration file (WiFi ssid and password and EMQX Cloud).
* Modify Arduino project settings: Tools > USB CDC On Boot > Select "Disabled".
* Connect ESP32S3 to the PC and launch the Arduino code.

## Legal <a name="legal"></a>
### Credits <a name="credits"></a>
If you have any suggestions or comments related to this GitHub project, please contact:

*LabMACS, DII, Università Politecnica delle Marche, Via Brecce Bianche, 12, Ancona, 60131, Italy* - [https://www.labmacs.university/](https://www.labmacs.university/)

* **Project Leader**: [David Scaradozzi](mailto:d.scaradozzi@staff.univpm.it)
* **Project Developer**: [Flavia Gioiello](mailto:f.gioiello@staff.univpm.it), [Gianluca Eremita](mailto:s1129029@studenti.univpm.it)
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
