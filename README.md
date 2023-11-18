# ESP32 Linky Teleinfo

![total commits](https://img.shields.io/github/commit-activity/t/GammaTroniques/TICMeter?style=flat-square)
![last commit](https://img.shields.io/github/last-commit/GammaTroniques/TICMeter?style=flat-square)
![release](https://img.shields.io/github/v/release/GammaTroniques/TICMeter?style=flat-square)
![issues](https://img.shields.io/github/issues/GammaTroniques/TICMeter?style=flat-square)
![pullrequests](https://img.shields.io/github/issues-pr/GammaTroniques/TICMeter?style=flat-square)

## License

![](https://img.shields.io/badge/license--blue?style=flat-square)

## About the project

**Linky Teleinfo** is a project to collect data from a **Linky meter** and send them to a **web server** or a **home automation server**. The system is powered by the pins A of the Linky meter and with the help of a supercapacitor.

- Created with : 

[![espidf](https://img.shields.io/badge/espressif%20idf-E7352C?style=for-the-badge&logo=espressif&logoColor=white)](https://github.com/espressif/esp-idf)
[![vscode](https://img.shields.io/badge/visual%20studio%20code-0078d7?style=for-the-badge&logo=visual-studio-code&logoColor=white)](https://www.espressif.com/)

- Work with :

[![zigbee](https://img.shields.io/badge/zigbee-F22547?style=for-the-badge&logo=zigbee&logoColor=white)](https://en.wikipedia.org/wiki/Zigbee)
[![mqtt](https://img.shields.io/badge/mqtt-660066?style=for-the-badge&logo=mqtt&logoColor=white)](https://mqtt.org/)
[![web](https://img.shields.io/badge/web-0050C9?style=for-the-badge&logo=web&logoColor=white)](https://gammatroniques.fr/)
[![tuya](https://img.shields.io/badge/tuya-FF4800?style=for-the-badge&logo=tuya&logoColor=white)](https://www.tuya.com/)

- Compatible with :

[![home-assistant](https://img.shields.io/badge/home%20assistant-%2341BDF5.svg?style=for-the-badge&logo=home-assistant&logoColor=white)](https://www.home-assistant.io/)
[![jeedom](https://img.shields.io/badge/jeedom-94CA02?style=for-the-badge&logo=jeedom&logoColor=white)](https://www.jeedom.com/)
[![domoticz](https://img.shields.io/badge/domoticz-0078C1?style=for-the-badge&logo=domoticz&logoColor=white)](https://www.domoticz.com/)

## Electronic

[`esp32/src`](/esp32/src) : The code is made to be sent on an ESP32 on an electronic card

| Top Side | Bottom Side | PCB in the Linky meter |
|-----|-----|-----|
| ![indev](https://shields.io/badge/-in%20the%20next%20update-inactive?style=flat-square) | ![indev](https://shields.io/badge/-in%20the%20next%20update-inactive?style=flat-square) | ![indev](https://shields.io/badge/-in%20the%20next%20update-inactive?style=flat-square) | 

#### Schematic

[`pcb/Schematic_LINKY_ESP32.pdf`](pcb/Schematic_LINKY_ESP32.pdf)

#### PCB

[`pcb/Gerber_PCB_LINKY_ESP32.zip`](pcb/Gerber_PCB_LINKY_ESP32.zip)

| Top Side  | Bottom Side |
| ------------- | ------------- |
| ![indev](https://shields.io/badge/-in%20the%20next%20update-inactive?style=flat-square) | ![indev](https://shields.io/badge/-in%20the%20next%20update-inactive?style=flat-square) |

## Installation

#### ESP32 :satellite:
Compile and send [`esp32/src`](/esp32/src) files to the ESP32

#### DataBase MySQL :globe_with_meridians:
Create a `Linky` Database
Copy the [.env_sample](/webserver/.env_sample) to `.env` and fill with your database logins

#### WebPage :computer:
Host the website available on [`webserver/`](/webserver)
```bash
cd webserver
#Installation
npm install
npx prisma generate
npx prisma db push
#Start
npm start
```

## Running Tests
Start the system with USB cable and serial monitor opened and you should see

```
Starting...
Connecting to [WIFI_SSID] ...
WiFi connected
IP address: 192.168.43.185
Getting config from server...OK
Getting time from NTP...OK
Disconecting from wifi...OK
Data stored: 0 - BASE:1647
Data stored: 1 - BASE:1647
Data stored: 2 - BASE:1647
Preparing json data... OK
Connecting to [WIFI_SSID] ...
WiFi connected
IP address: 192.168.43.185
Getting config from server...OK
Getting time from NTP...OK
Sending data to server... OK: 200
Disconecting from wifi...OK
```
## Demo

#### Web example

![webpageimg](img/WebPage.png)

#### MQTT example - Home Assistant

![indev](https://shields.io/badge/-in%20the%20next%20update-inactive?style=flat-square)

#### Zigbee example - Home Assistant

![indev](https://shields.io/badge/-in%20the%20next%20update-inactive?style=flat-square)

#### Tuya example

![indev](https://shields.io/badge/-in%20the%20next%20update-inactive?style=flat-square)

## Authors

- [@Dorian.local/](https://github.com/xmow49)
- [@Noah_](https://github.com/NoahJst)

[![siteweb](https://img.shields.io/badge/GammaTroniques-EE6B00?style=for-the-badge&logoColor=white)](https://gammatroniques.fr/)
[![youtube](https://img.shields.io/youtube/channel/subscribers/UCnUqy6VAEgcNR745mNsyTHg?style=for-the-badge&logo=youtube&label=YouTube&labelColor=FF0000&color=333333)](https://www.youtube.com/gammatroniques)

>__Note__  
This program is still under development, it is possible that errors and problems are found in the code

:shipit:
