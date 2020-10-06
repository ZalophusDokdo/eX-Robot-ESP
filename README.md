# eX-Robot
ESP8266 Self Balancing Robot
<p align="center">
	<a href="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Photo_20170421_0027.jpg?raw=true">
    <img src="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Photo_20170421_0027.jpg?raw=true" width="100%">
  </a>
</p>

Self Balancing Robot (eX-Robot, B-Robot, Roverbot, ...
- 1 https://youtu.be/yUEGPzcLrT0
- 2 https://youtu.be/GBzCG3s8HUw
- 3 https://youtu.be/doZN54FWyQQ
- 4 https://youtu.be/MALJrZCLNYo
- 5 https://youtu.be/9zdlfkKI_mE
- 6 https://youtu.be/HBjeSd7bPsw
- 7 https://youtu.be/UA2rX-h0y3c

Features:
-
- Arduino IDE (ESP8266 ESP-12E/F)
- Support SoftAP and Station mode
- Support OTA (at Local network)
- Support mDNS (at Local network)
- TouchOSC UDP control

Fuctions:
-
- mDNS
- WiFi Manager
- TouchOSC UDP control

BOM
-
- 1 x ESP8266 Witty Cloud or WeMos D1 mini (eX-Robot)
- 1 x GY-521 (MPU6050)
- 2 x A4988 Step Motor Drive
- 1 x LM1117-5.0 5,0V 1A Regulator
- 1 x LM1117-3,3 3,3V 1A Regulator
- 6 x 100uF 25V Capacitor
- 4 x 0.1uF Capacitor
- 1 x 220KOhm Resistor
- 1 x 100KOhm Resistor
- 4 x 10KOhm Resistor
- 7 x 8P Female Pin Header Connector 2.54mm Pitch
- 3 x 4P Male Pin Header Connector 2.54mm Pitch
- 2 x 3P Male Pin Header Connector 2.54mm Pitch
- 3 x 2P Male Pin Header Connector 2.54mm Pitch
- 1 x 2EDGK 5.08mm 2P Plug-in terminal connectors set
- 1 x Right Angle SPDT 4 Pin On-On I/O Boat Rocker Switch
- 2 x NEMA 17 - Phase: 4, Step Angle: 1.8 Deg/Step, Holding Torque: 2.6Kg.cm
- 1 x SG90 Metal Servo or Standard Servo (Option)

Power Requirements:
-
8.4VDC ~ 12 VDC

Battery:
-
2 x 18650 Litum Ion Battery

Configuring the upload environment
==================================
How to configure standard Arduino IDE to use as Arduino ESP8266 IDE.

First download Arduino IDE and install it on computer.

IMPORTANT: Download old version Arduino IDE Version 1.8.1.
(Tested Version 1.8.9 and Version 1.8.13)

------------
Open Arduino IDE and go to File - Preferences.

Enter http://arduino.esp8266.com/stable/package_esp8266com_index.json under Adittional boards manager.

------------
Select Boards Manager in Tools - Board

Find ESP8266 and press Install

IMPORTANT: Download old version esp8266 by ESP8266 Community Version 2.3.0.

------------
IMPORTANT: Additional Libraries

go to Sketch - Include Library - Manage Libraries

Find and Install
- WiFiManager by tzapu,tablatronix Version 0.15.0

--------------
After install select ESP9266 board.
- Board: NodeMCU 1.0 (ESP-12E Module)
- Upload Speed: "115200"
- CPU Frequency: "80 MHz"
- Flash Size: "4M (3M SPIFFS)"
- Port: COM?? 

---------------
Schematic is available here:
<p align="center">
  <a href="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Shield_REV0_sch.png">
    <img src="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Shield_REV0_sch.png" width="100%">
  </a>
  <br>
  <a href="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Shield_REV0_brd.png">
    <img src="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Shield_REV0_brd.png" width="100%">
  </a>
</p>

---------------
3D Parts:
- Pinshape : https://pinshape.com/items/49151-3d-printed-profileblock%E2%84%A2-balancing-robot-diy-robot-platform
- Thingiverse : https://www.thingiverse.com/thing:2269502

---------------
Design History:
<p align="center">
  <a href="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Concept_002.jpg">
    <img src="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Concept_002.jpg" width="100%">
  </a>
  <br>
  <a href="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Concept_003.jpg">
    <img src="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Concept_003.jpg" width="100%">
  </a>
  <a href="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Concept_004.jpg">
    <img src="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Concept_004.jpg" width="100%">
  </a>
  <a href="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Exploded_000e.jpg">
    <img src="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Exploded_000e.jpg" width="100%">
  </a>
  <a href="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Exploded_001e.jpg">
    <img src="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Exploded_001e.jpg" width="100%">
  </a>
  <a href="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Exploded_002e.jpg">
    <img src="https://github.com/ZalophusDokdo/eX-Robot-ESP/blob/master/images/ESP8266_ESP-12E_WeMos_D1_mini_eX-Robot_Exploded_002e.jpg" width="100%">
  </a>
</p>
