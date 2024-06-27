# **Wavin-AHC-9000-mqtt**


A Esp8266 mqtt interface for more Wavin AHC-9000 controllers connected together and controlled by one Wavin display.

The goal is to be able to control the Wavin AHC 9000 heating controllers from [Home Assistant](https://www.home-assistant.io/) (HA), and still be able to use the Wavin display as well.

The origilan goal of being able to controle the Wavin AHC 9000 heating controllers from [OpenHAP](https://www.openhab.org/), has been abandoned because I gave up finding a discovery function for MQTT integrations, like the one in HA.

### **Current project status**
In the current version, the Esp8266 mqtt interface will controle one or more Wavin AHC-9000 controllers. The Wavin AHC-9000 controllers are initial installed and configured using the Wavin display, which configures each controller's MOC-BUS device number.

Because the MODBUS specifications does not allow more than one master, the Wavin display will not be working together with the Esp8266 mqtt interface in the current version. Investigations will be done to verify if it will be possible to build a solution, using a 2 Circuit IC Switch [CD4052B](https://www.ti.com/product/CD4052B-MIL#tech-docs) to controle which controller (Wavin display or Esp8266) will be in action.

The project include a [Kicad](https://www.kicad.org/) PCB layout and a docker based mosquitto broker based on [eclipse-mosquitto](https://hub.docker.com/_/eclipse-mosquitto).

### **History**

This propject has started as a git clone of https://github.com/dkjonas/Wavin-AHC-9000-mqtt.git but restructured for Firmware (FW), Software (SW) and Hardware (HW).

This project takes on the challange mentioned in [Issue #3](https://github.com/dkjonas/Wavin-AHC-9000-mqtt/issues/3#issuecomment-435690672)

##### "connect two AHC's together and control them by one display. I think this is rather unusual setup, so my code doesn't handle this case; It can only control one AHC."

## **Esp8266 mqtt interface**

See the cloned Firmware markdown [here](Firmware/README.md).

### **SOFTWARE**

### Compiling

Over The Air update (OTA) is implemented in main.cpp. 

Compiled with [PlatformIO](https://platformio.org/) installed in VS Code and adding the follow to platformio.ini the code will provide the possability of setting PlatformIO Project Environment to   `env:nodemcu_ota` and upload will happen OTA.

````bash
[env:nodemcu_ota]
extends = env:nodemcu
upload_protocol = espota
upload_port = <IP Address of NodeMCU>
````


## **Mosquitto broker**

### **Requirements**

- Raspberry Pi model B v1.2. Installed with Raspberry Pi OS Lite (32-bit) <span id="a1">[[1]](#f1)</span>.
- Internet access.
- Docker Engin and Docker Compose installed <span id="a2">[[2]](#f2)</span>.


### **Create docker-compose file**

In folder `Wavin-AHC-9000-mqtt` create file `docker-compose.yaml`

```bash
vi docker-compose.yaml
```

Add the following contend:

```bash
# Docker-compose file for building Wavin-AHC-9000-mqtt program stack.

# Generel note about environment settings - see README.md file about .env file

version: '3.8'

services:
  mosquitto-mqtt:
  container_name: mosquitto-mqtt
  image: eclipse-mosquitto:2
  restart: always
  command: mosquitto -c /mosquitto-no-auth.conf
  ports:
    - 1883:1883
  volumes:
    - mosquitto-conf:/mosquitto/config
    - mosquitto-data:/mosquitto/data
    - mosquitto-log:/mosquitto/log

volumes:
  mosquitto-conf:
    name: mosquitto-conf
  mosquitto-data:
    name: mosquitto-data
  mosquitto-log:
    name: mosquitto-log

````


 Start the docker container with `docker-compose up -d`.

```bash
docker-compose up -d
```

## Change LOG

|       | Module              | Version | Change
|---    |---                  | ----    |----
|       |                     | 0.0.1   | Initial commit. On-board LED at PIN 16 added to indicate active boot up. It turns off at successful connect to Wireless Lan. 
| FW    | WavinController.h   | 0.0.2   | MODBUS_DEVICE changed from = 0x01 to 0x02
| FW    | WavinController.h   | 0.0.3   | MODBUS_DEVICE changed from = 0x02 to 0x03
| FW    | WavinController.h   | 0.0.4   | Added MODBUS_DEVICES and NUMBER_OF_DEVIDES
| FW    | WavinController.cpp | 0.0.4   | Objects modified to handle MODBUS_DEVICE as an argument instad of fixed value.
| FW    | All modules         | 0.0.4   | Calls to objects, which has changed because of the objects being modified for handle MODBUS_DEVICE as an argument. MQTT Publish functions has been added Modbus Device.
| FW    | main.cpp            | 0.0.5   | publishConfiguration() changed to comply with homeassistant topic.
| FW    | main.cpp            | 0.0.6   | Implementing OTA update.
| FW    | main.cpp            | 0.0.7   | OTA and temerature setting issues.
| FW    | main.cpp & PrivateConfig.h | 0.0.8   | Configurable names for chanels added to PrivateConfig.h and implemented in mail.cpp
| FW    | main.cpp            | 0.0.9 | Re-arrange code structure. Insert function definitions, startup() and loot() in beginning.
| FW    | main.cpp            | 0.0.10 | #4: Insert WiFi reconnect postpone time <br>#5: call WiFi.disconnect() before "reconnect" (WiFi.mode(WIFI_STA) og WiFi.begin() <br>#2: Publish device info to MQTT broker
| FW    | mail.cpp            | 0.0.11 | #3: implement Arduino JSON

###### "#n" Refers to github issue number





## Credits
- dkjonas/Wavin-AHC-9000-mqtt

#### **Footnotes**
=======
###### 1. <span id="f1"></span> See document: [Setting up and configure Raspberry Pi 3 MODEL B+ for the Docker Traefik router project](./docs/SettingUpRaspberryPi.md). [$\hookleftarrow$](#a1)

###### 2. <span id="f2"></span> See document [Install Docker Engin and Docker Compose](./docs/InstallDockerEnginAndCompose.md)[$\hookleftarrow$](#a3)
